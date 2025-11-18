#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

struct RacelinePoint { double s,x,y,psi,kappa,v; };

class RacelineServer : public rclcpp::Node {
 public:
  RacelineServer(): Node("raceline_server") {
    declare_parameter<std::string>("raceline_file", "data/raceline.csv");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("publish_vref", true);
    file_ = get_parameter("raceline_file").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_vref_ = get_parameter("publish_vref").as_bool();
    qos_.keep_last(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_raceline", qos_);
    if (publish_vref_) vref_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/global_vref", qos_);
    
    auto cb = [this](const std::vector<rclcpp::Parameter> &params) {
      bool reload=false;
      for(const auto &p: params) {
        if(p.get_name()=="raceline_file" && p.get_type()==rclcpp::ParameterType::PARAMETER_STRING) {
          file_=p.as_string();
          reload=true;
        }
        if(p.get_name()=="frame_id" && p.get_type()==rclcpp::ParameterType::PARAMETER_STRING) {
          frame_id_=p.as_string();
        }
        if(p.get_name()=="publish_vref" && p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL) {
          publish_vref_=p.as_bool();
          if(publish_vref_ && !vref_pub_) vref_pub_=create_publisher<std_msgs::msg::Float32MultiArray>("/global_vref", qos_);
        }
      }
      if(reload) publish_once();
      rcl_interfaces::msg::SetParametersResult r;
      r.successful=true;
      return r;
    };
    param_cb_ = add_on_set_parameters_callback(cb);
    publish_once();
  }

 private:
  std::string file_, frame_id_;
  bool publish_vref_{true};
  rclcpp::QoS qos_{1};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vref_pub_;
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
    RCLCPP_INFO(get_logger(), "Raceline loaded: %zu points", pts.size());
    auto stamp=now();
    nav_msgs::msg::Path path;
    path.header.stamp=stamp;
    path.header.frame_id=frame_id_;
    path.poses.reserve(pts.size());
    std_msgs::msg::Float32MultiArray vref;
    if(publish_vref_) vref.data.reserve(pts.size());
    for(const auto &p: pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp=stamp;
      ps.header.frame_id=frame_id_;
      ps.pose.position.x=p.x;
      ps.pose.position.y=p.y;
      double half=0.5*p.psi;
      ps.pose.orientation.z=std::sin(half);
      ps.pose.orientation.w=std::cos(half);
      path.poses.push_back(ps);
      if(publish_vref_) vref.data.push_back((float)p.v);
    }
    path_pub_->publish(path);
    if(publish_vref_) vref_pub_->publish(vref);
  }
};

int main(int argc,char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelineServer>());
  rclcpp::shutdown();
  return 0;
}