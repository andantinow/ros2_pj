#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

/*
  Raceline Server Node (C++)
  - Reads an offline-generated raceline.csv and publishes:
    1) /global_raceline as nav_msgs/Path
    2) /global_vref as std_msgs/Float32MultiArray (aligned speeds)
  - QoS: Transient Local (latched) + Reliable, so late subscribers receive the last message.
  - frame_id defaults to "map".
*/

struct RacelinePoint {
  double s;      // arc length [m]
  double x;      // map X [m]
  double y;      // map Y [m]
  double psi;    // heading yaw [rad]
  double kappa;  // curvature [1/m]
  double v_ref;  // reference speed [m/s]
};

class RacelineServer : public rclcpp::Node {
public:
  RacelineServer() : Node("raceline_server") {
    // Parameters
    declare_parameter<std::string>("raceline_file", "data/raceline.csv");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("publish_vref", true);

    file_ = this->get_parameter("raceline_file").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_vref_ = this->get_parameter("publish_vref").as_bool();

    if (!load_csv(file_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open raceline file: %s", file_.c_str());
      throw std::runtime_error("Raceline load failed");
    }

    setup_publishers();
    publish_once();
  }

private:
  std::string file_, frame_id_;
  bool publish_vref_{true};
  std::vector<RacelinePoint> points_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vref_pub_;

  static bool parse_row(const std::string &line, RacelinePoint &p) {
    // Accept comma-separated (preferred) or whitespace-separated
    std::stringstream ss(line);
    char comma;
    if ((ss >> p.s >> comma >> p.x >> comma >> p.y >> comma >> p.psi >> comma >> p.kappa >> comma >> p.v_ref)) {
      return true;
    }
    ss.clear(); ss.str(line);
    if ((ss >> p.s >> p.x >> p.y >> p.psi >> p.kappa >> p.v_ref)) {
      return true;
    }
    return false;
  }

  bool load_csv(const std::string &file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    std::string line;

    // First line: header or data
    if (!std::getline(in, line)) return false;
    const bool has_header = (line.find("s") != std::string::npos && line.find("x") != std::string::npos);
    if (!has_header) {
      RacelinePoint p;
      if (parse_row(line, p)) points_.push_back(p);
    }

    // Parse remaining rows
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      RacelinePoint p;
      if (!parse_row(line, p)) {
        RCLCPP_WARN(this->get_logger(), "Malformed row skipped: %s", line.c_str());
        continue;
      }
      points_.push_back(p);
    }

    if (points_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid raceline points: %s", file.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded raceline points: %zu", points_.size());
    return true;
  }

  void setup_publishers() {
    rclcpp::QoS qos(1);
    qos.transient_local(); // latched
    qos.reliable();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_raceline", qos);
    if (publish_vref_) {
      vref_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/global_vref", qos);
    }
  }

  void publish_once() {
    const auto stamp = this->now();

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = frame_id_;
    path.poses.reserve(points_.size());

    std_msgs::msg::Float32MultiArray vref;
    if (publish_vref_) vref.data.reserve(points_.size());

    for (const auto &p : points_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = frame_id_;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;

      // yaw -> quaternion (z,w for yaw-only)
      const double half = 0.5 * p.psi;
      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = std::sin(half);
      ps.pose.orientation.w = std::cos(half);

      path.poses.push_back(ps);
      if (publish_vref_) vref.data.push_back(static_cast<float>(p.v_ref));
    }

    path_pub_->publish(path);
    if (publish_vref_) vref_pub_->publish(vref);

    RCLCPP_INFO(this->get_logger(), "Published /global_raceline (N=%zu)%s",
                points_.size(), publish_vref_ ? " + /global_vref" : "");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelineServer>());
  rclcpp::shutdown();
  return 0;
}
