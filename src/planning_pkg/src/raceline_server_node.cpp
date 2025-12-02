#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "planning_pkg/path_utils.hpp"

struct RacelinePoint { double s,x,y,psi,kappa,v; };

class RacelineServer : public rclcpp::Node {
 public:
  RacelineServer(): Node("raceline_server") {
    declare_parameter<std::string>("raceline_file", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("publish_vref", true);
    declare_parameter<bool>("enable_smoothing", false);
    declare_parameter<int>("smoothing_window", 5);
    declare_parameter<double>("resample_spacing", 0.0);  // 0.0 = no resampling
    file_ = resolve_file(get_parameter("raceline_file").as_string());
    frame_id_ = get_parameter("frame_id").as_string();
    publish_vref_ = get_parameter("publish_vref").as_bool();
    enable_smoothing_ = get_parameter("enable_smoothing").as_bool();
    smoothing_window_ = get_parameter("smoothing_window").as_int();
    resample_spacing_ = get_parameter("resample_spacing").as_double();
    qos_.keep_last(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_raceline", qos_);
    if (publish_vref_) vref_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/global_vref", qos_);
    
    auto cb = [this](const std::vector<rclcpp::Parameter> &params) {
      bool reload=false;
      for(const auto &p: params) {
        if(p.get_name()=="raceline_file" && p.get_type()==rclcpp::ParameterType::PARAMETER_STRING) {
          file_=resolve_file(p.as_string());
          reload=true;
        }
        if(p.get_name()=="frame_id" && p.get_type()==rclcpp::ParameterType::PARAMETER_STRING) {
          frame_id_=p.as_string();
        }
        if(p.get_name()=="publish_vref" && p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL) {
          publish_vref_=p.as_bool();
          if(publish_vref_ && !vref_pub_) vref_pub_=create_publisher<std_msgs::msg::Float32MultiArray>("/global_vref", qos_);
        }
        if(p.get_name()=="enable_smoothing" && p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL) {
          enable_smoothing_=p.as_bool();
          reload=true;
        }
        if(p.get_name()=="smoothing_window" && p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER) {
          smoothing_window_=p.as_int();
          reload=true;
        }
        if(p.get_name()=="resample_spacing" && p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE) {
          resample_spacing_=p.as_double();
          reload=true;
        }
      }
      if(reload) publish_once();
      rcl_interfaces::msg::SetParametersResult r;
      r.successful=true;
      return r;
    };
    param_cb_ = add_on_set_parameters_callback(cb);
    
    // Publish immediately on startup
    publish_once();
    
    // Also publish periodically (1 Hz) to ensure RViz receives it even if it starts late
    publish_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish_once(); }
    );
  }

 private:
  std::string file_, frame_id_;
  bool publish_vref_{true};
  bool enable_smoothing_{false};
  int smoothing_window_{5};
  double resample_spacing_{0.0};
  rclcpp::QoS qos_{1};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vref_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  static bool parse_row(const std::string &line, RacelinePoint &p) {
    std::stringstream ss(line);
    char c;
    if((ss>>p.s>>c>>p.x>>c>>p.y>>c>>p.psi>>c>>p.kappa>>c>>p.v)) return true;
    ss.clear();
    ss.str(line);
    if((ss>>p.s>>p.x>>p.y>>p.psi>>p.kappa>>p.v)) return true;
    return false;
  }

  bool load(std::vector<RacelinePoint> &pts) {
    pts.clear();
    std::ifstream in(file_);
    if(!in.is_open()) return false;
    std::string line;
    if(!std::getline(in,line)) return false;
    bool header=line.find("s,")!=std::string::npos;
    if(!header) {
      RacelinePoint p;
      if(parse_row(line,p)) pts.push_back(p);
    }
    while(std::getline(in,line)) {
      if(line.empty()) continue;
      RacelinePoint p;
      if(parse_row(line,p)) pts.push_back(p);
    }
    return !pts.empty();
  }

  void publish_once() {
    std::vector<RacelinePoint> pts;
    if(!load(pts)) {
      RCLCPP_ERROR(get_logger(), "Failed to load raceline: %s", file_.c_str());
      return;
    }
    
    // Validate and filter NaN/Inf values
    std::vector<RacelinePoint> valid_pts;
    valid_pts.reserve(pts.size());
    for(const auto &p: pts) {
      if(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.psi) && 
         std::isfinite(p.kappa) && std::isfinite(p.v) && std::isfinite(p.s)) {
        valid_pts.push_back(p);
      } else {
        RCLCPP_WARN(get_logger(), "Filtered invalid point: x=%.3f, y=%.3f", p.x, p.y);
      }
    }
    
    if(valid_pts.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid points after filtering NaN/Inf values");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Raceline loaded: %zu points (filtered from %zu)", valid_pts.size(), pts.size());
    
    // Enforce continuity to avoid large jumps (> 2m) between points
    auto enforce_continuity = [](const std::vector<RacelinePoint> & input) {
      std::vector<RacelinePoint> output;
      if (input.empty()) {
        return output;
      }
      constexpr double kMaxGap = 2.0;  // meters
      output.push_back(input.front());
      for (size_t i = 1; i < input.size(); ++i) {
        const auto & prev = output.back();
        const auto & curr = input[i];
        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;
        double gap = std::hypot(dx, dy);
        if (gap > kMaxGap) {
          int segments = static_cast<int>(std::floor(gap / kMaxGap));
          double step = 1.0 / (segments + 1);
          for (int s = 1; s <= segments; ++s) {
            double frac = step * s;
            RacelinePoint interp;
            interp.s = prev.s + frac * (curr.s - prev.s);
            interp.x = prev.x + frac * dx;
            interp.y = prev.y + frac * dy;
            double yaw_diff = curr.psi - prev.psi;
            while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
            interp.psi = prev.psi + frac * yaw_diff;
            interp.kappa = prev.kappa + frac * (curr.kappa - prev.kappa);
            interp.v = prev.v + frac * (curr.v - prev.v);
            output.push_back(interp);
          }
        }
        output.push_back(curr);
      }
      return output;
    };
    
    auto continuous_pts = enforce_continuity(valid_pts);
    if (continuous_pts.empty()) {
      RCLCPP_ERROR(get_logger(), "Continuity enforcement produced no points");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Raceline loaded: %zu points (filtered from %zu, continuity corrected)",
                continuous_pts.size(), pts.size());
    
    // Use zero timestamp for static data (RViz will always display it)
    auto stamp=rclcpp::Time(0);
    nav_msgs::msg::Path path;
    path.header.stamp=stamp;
    path.header.frame_id="map";  // Force frame_id to "map" for coordinate consistency
    path.poses.reserve(continuous_pts.size());
    std_msgs::msg::Float32MultiArray vref;
    if(publish_vref_) vref.data.reserve(continuous_pts.size());
    
    for(const auto &p: continuous_pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp=stamp;  // Zero timestamp for static data
      ps.header.frame_id="map";  // Force frame_id to "map"
      ps.pose.position.x=p.x;
      ps.pose.position.y=p.y;
      ps.pose.position.z=0.0;  // Ensure z is set
      double half=0.5*p.psi;
      ps.pose.orientation.z=std::sin(half);
      ps.pose.orientation.w=std::cos(half);
      ps.pose.orientation.x=0.0;
      ps.pose.orientation.y=0.0;
      path.poses.push_back(ps);
      if(publish_vref_) vref.data.push_back((float)p.v);
    }
    
    // Apply smoothing if enabled
    if(enable_smoothing_ && path.poses.size() >= 3) {
      planning_pkg::smooth_path(path, smoothing_window_);
      RCLCPP_INFO(get_logger(), "Applied smoothing with window size: %d", smoothing_window_);
    }
    
    // Resample if spacing is specified
    if(resample_spacing_ > 0.0) {
      path = planning_pkg::resample_path(path, resample_spacing_);
      RCLCPP_INFO(get_logger(), "Resampled path with spacing: %.3f m", resample_spacing_);
    }
    
    // Validate path before publishing
    if(!planning_pkg::validate_path(path)) {
      RCLCPP_ERROR(get_logger(), "Path validation failed, not publishing");
      return;
    }
    
    double path_len = planning_pkg::path_length(path);
    RCLCPP_WARN(get_logger(), "=== PUBLISHING RACELINE === Points: %zu, Length: %.2f m, Frame: %s",
                path.poses.size(), path_len, frame_id_.c_str());
    if (!path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "First point: (%.3f, %.3f), Last: (%.3f, %.3f)",
                  path.poses[0].pose.position.x, path.poses[0].pose.position.y,
                  path.poses.back().pose.position.x, path.poses.back().pose.position.y);
    }
    path_pub_->publish(path);
    if(publish_vref_) vref_pub_->publish(vref);
    RCLCPP_WARN(get_logger(), "Path published to /global_raceline");
  }

  std::string resolve_file(const std::string &raw) {
    namespace fs = std::filesystem;
    std::string candidate = raw;
    if(candidate.empty()) {
      candidate = (fs::path(ament_index_cpp::get_package_share_directory("planning_pkg")) / "data" / "raceline.csv").string();
    }
    fs::path p(candidate);
    if(p.is_relative()) {
      auto base = fs::path(ament_index_cpp::get_package_share_directory("planning_pkg"));
      p = base / p;
    }
    return p.lexically_normal().string();
  }
};

int main(int argc,char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelineServer>());
  rclcpp::shutdown();
  return 0;
}
