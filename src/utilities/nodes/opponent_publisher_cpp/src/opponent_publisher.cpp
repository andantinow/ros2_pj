#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

using std::placeholders::_1;

class OpponentPublisher : public rclcpp::Node {
public:
  OpponentPublisher()
  : Node("opponent_follower")
  {
    // Parameters
    lookahead_s_ = declare_parameter<double>("lookahead_s", 0.75);
    speed_       = declare_parameter<double>("speed",       2.5);  // Reduced from 3.0 for easier ego overtaking
    wheelbase_   = declare_parameter<double>("wheelbase",   0.325);
    odom_topic_  = declare_parameter<std::string>("odom_topic", "/opp_racecar/odom");
    path_topic_  = declare_parameter<std::string>("path_topic", "/global_raceline");
    
    // Opponent width and wall margin parameters
    // Per user request: opponent should stay farther from walls but not exactly centered
    opponent_width_ = declare_parameter<double>("opponent_width", 0.35);  // Vehicle width (m)
    wall_margin_ = declare_parameter<double>("wall_margin", 0.35);  // Increased from 0.25m for more visible margin from walls
    lateral_offset_ = declare_parameter<double>("lateral_offset", 0.15);  // Offset from exact center (m) - creates overtake lanes

    // Launch compatibility parameters (declared for compatibility, not actively used)
    declare_parameter<double>("start_s", 0.0);
    declare_parameter<std::string>("trajectory", "min_curv"); // centerline/min_curv/shortest_path/min_time
    declare_parameter<bool>("constant_speed", false);
    declare_parameter<std::string>("type", "lidar"); // virtual/lidar

    // Pub/Sub
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/opp_drive", 10);
    // Subscribe to nav_msgs/Path (from raceline_server) instead of WpntArray
    path_sub_  = create_subscription<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(10).transient_local(), 
        std::bind(&OpponentPublisher::pathCB, this, _1));
    odom_sub_  = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&OpponentPublisher::odomCB, this, _1));

    // Timer at 10 Hz
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OpponentPublisher::timerCB, this));

    RCLCPP_INFO(get_logger(), "Opponent Publisher ready, subscribing to path: %s", path_topic_.c_str());
    RCLCPP_INFO(get_logger(), 
                "Opponent config: speed=%.2fm/s, width: %.2fm, wall margin: %.2fm, lateral offset: %.2fm", 
                speed_, opponent_width_, wall_margin_, lateral_offset_);
    RCLCPP_INFO(get_logger(), 
                "Opponent behavior: Stays %.2fm from walls, %.2fm offset from center (creates overtake lanes)", 
                wall_margin_, lateral_offset_);
  }

private:
  // ===== Path handling (nav_msgs/Path from raceline_server) =====
  void pathCB(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (got_waypoints_) return;

    const auto & poses = msg->poses;
    if (poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path.");
      return;
    }

    xs_.clear();
    ys_.clear();
    psis_.clear();
    ss_.clear();
    
    xs_.reserve(poses.size());
    ys_.reserve(poses.size());
    psis_.reserve(poses.size());
    ss_.reserve(poses.size());

    double cumulative_s = 0.0;
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto & pose = poses[i].pose;
      
      // Extract yaw from quaternion (with normalization for numerical stability)
      double qx = pose.orientation.x;
      double qy = pose.orientation.y;
      double qz = pose.orientation.z;
      double qw = pose.orientation.w;
      // Normalize quaternion to handle potentially unnormalized input
      double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
      if (norm > 1e-9) {
        qx /= norm; qy /= norm; qz /= norm; qw /= norm;
      }
      double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      psis_.push_back(yaw);
      
      // Apply lateral offset to opponent's path to keep away from raceline/walls
      // Positive lateral_offset_ shifts LEFT (perpendicular to heading)
      double offset_x = pose.position.x - lateral_offset_ * std::sin(yaw);
      double offset_y = pose.position.y + lateral_offset_ * std::cos(yaw);
      
      xs_.push_back(offset_x);
      ys_.push_back(offset_y);
      
      // Calculate cumulative arc length using std::hypot for numerical stability
      if (i > 0) {
        double dx = xs_[i] - xs_[i-1];
        double dy = ys_[i] - ys_[i-1];
        cumulative_s += std::hypot(dx, dy);
      }
      ss_.push_back(cumulative_s);
    }

    track_length_ = cumulative_s;
    got_waypoints_ = true;

    RCLCPP_INFO(get_logger(), "Path loaded from %s: %zu points, track length = %.2f m", 
                path_topic_.c_str(), poses.size(), track_length_);
  }

  // ===== Odom handling =====
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & pose = msg->pose.pose;

    curr_x_ = pose.position.x;
    curr_y_ = pose.position.y;

    // yaw 추출 (tf_transformations의 yaw 추출과 동일 수식)
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
    curr_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    have_pose_ = true;

    // 진행거리 s 추정: 가장 가까운 웨이포인트의 s_m 사용 (간단·안정)
    if (got_waypoints_) {
      size_t idx_min = 0;
      double dmin = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < xs_.size(); ++i) {
        double dx = curr_x_ - xs_[i];
        double dy = curr_y_ - ys_[i];
        double d2 = dx*dx + dy*dy;
        if (d2 < dmin) {
          dmin = d2;
          idx_min = i;
        }
      }
      curr_s_ = wrapS(ss_[idx_min]);
      have_s_ = true;
    }
  }

  // ===== Timer control loop =====
  void timerCB()
  {
    if (!got_waypoints_ || !have_pose_ || !have_s_) return;
    if (xs_.size() < 2) return;

    // 1) 목표 s
    double target_s = wrapS(curr_s_ + lookahead_s_);

    // 2) s -> (x, y) 복원 (구간 선형보간)
    double tx = 0.0, ty = 0.0;
    sToXY(target_s, tx, ty);

    // 3) 차량 좌표계로 변환
    const double lx = tx - curr_x_;
    const double ly = ty - curr_y_;
    const double ct = std::cos(-curr_yaw_);
    const double st = std::sin(-curr_yaw_);
    const double local_x = ct * lx - st * ly;
    const double local_y = st * lx + ct * ly;

    // 4) Pure Pursuit 조향
    double steering = 0.0;
    if (std::fabs(local_x) >= 1e-6 || std::fabs(local_y) >= 1e-6) {
      const double alpha = std::atan2(local_y, local_x);
      const double Ld    = std::hypot(local_x, local_y);
      steering = (Ld > 1e-6) ? std::atan2(2.0 * wheelbase_ * std::sin(alpha), Ld) : 0.0;
    }

    // 5) Publish drive
    ackermann_msgs::msg::AckermannDriveStamped ack;
    ack.header.stamp = now();
    ack.header.frame_id = "base_link";
    ack.drive.speed = speed_;
    ack.drive.steering_angle = steering;
    drive_pub_->publish(ack);
  }

  // ===== Utilities =====
  double wrapS(double s) const
  {
    if (track_length_ <= 0.0) return s;
    // [0, track_length) 로 래핑
    double m = std::fmod(s, track_length_);
    if (m < 0.0) m += track_length_;
    return m;
  }

  // target_s가 속한 구간 [i, i+1]을 찾아 선형 보간
  void sToXY(double target_s, double &out_x, double &out_y) const
  {
    // s가 단조 증가한다고 가정 (일반적인 raceline)
    // 경계 케이스
    if (target_s <= ss_.front()) {
      out_x = xs_.front();
      out_y = ys_.front();
      return;
    }
    if (target_s >= ss_.back()) {
      out_x = xs_.back();
      out_y = ys_.back();
      return;
    }

    // 이분 탐색으로 구간 찾기: ss_[i] <= target_s < ss_[i+1]
    auto it = std::upper_bound(ss_.begin(), ss_.end(), target_s);
    size_t j = std::distance(ss_.begin(), it);
    size_t i = (j == 0) ? 0 : j - 1;

    const double s0 = ss_[i];
    const double s1 = ss_[j];
    const double t  = (std::fabs(s1 - s0) > 1e-9) ? (target_s - s0) / (s1 - s0) : 0.0;

    out_x = (1.0 - t) * xs_[i] + t * xs_[j];
    out_y = (1.0 - t) * ys_[i] + t * ys_[j];
  }

private:
  // Parameters
  double lookahead_s_{0.5};
  double speed_{2.5};                // Reduced from 3.0 for easier ego overtaking
  double wheelbase_{0.2};
  std::string odom_topic_{"/opp_racecar/odom"};
  std::string path_topic_{"/global_raceline"};
  
  // Opponent dimensions and margins
  double opponent_width_{0.35};      // Vehicle width (m)
  double wall_margin_{0.35};         // Minimum distance from walls (m) - farther from walls
  double lateral_offset_{0.15};      // Lateral offset from raceline (m), positive = left - not exactly centered

  // Waypoints storage (from Path)
  std::vector<double> xs_, ys_, psis_, ss_;
  double track_length_{0.0};
  bool   got_waypoints_{false};

  // Current state
  double curr_x_{0.0}, curr_y_{0.0}, curr_yaw_{0.0};
  double curr_s_{0.0};
  bool   have_pose_{false};
  bool   have_s_{false};

  // ROS interfaces
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpponentPublisher>());
  rclcpp::shutdown();
  return 0;
}
