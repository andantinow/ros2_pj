#ifndef SIMPLE_CONTROLLER_HPP_
#define SIMPLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/utils.h>
#include <vector>
#include <string>

class SimpleController : public rclcpp::Node
{
public:
  SimpleController();

  enum class CRSMState {
    ST_NORMAL,            
    ST_CRASH_DETECTED,    
    ST_RECOVERY_REVERSE,  
    ST_RECOVERY_REALIGN   
  };
  
  static constexpr double SAFETY_MARGIN = 0.15;             
  static constexpr double WALL_SAFETY_MARGIN = 0.15;        
  static constexpr double MIN_OVERTAKE_DISTANCE = 0.6;      
  static constexpr double OVERTAKE_VIS_LENGTH = 5.0;        
  static constexpr double OVERTAKE_VIS_STEP = 0.15;         
  static constexpr double IMU_CRASH_ACCEL_THRESHOLD = 9.5;  
  static constexpr double STALL_VELOCITY_THRESHOLD = 0.1;   
  static constexpr double STALL_CMD_VELOCITY_THRESHOLD = 0.5; 

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;  
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;   
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr safety_zone_pub_;  
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Path current_path_;
  sensor_msgs::msg::LaserScan current_scan_;
  sensor_msgs::msg::Imu current_imu_;           
  std::string current_racing_mode_ = "CRUISE";  
  bool odom_received_ = false;
  bool path_received_ = false;
  bool scan_received_ = false;
  bool imu_received_ = false;                    
  
  CRSMState crsm_state_ = CRSMState::ST_NORMAL;  
  double last_cmd_velocity_ = 0.0;               
  double imu_accel_magnitude_ = 0.0;             

  double lookahead_distance_ = 0.0;
  double min_lookahead_ = 0.4;         
  double max_lookahead_ = 1.2;         
  double lookahead_speed_gain_ = 0.3;  
  double lookahead_error_gain_ = 0.0;
  double target_speed_ = 4.0;          
  double max_speed_ = 6.0;             
  double wheelbase_ = 0.33;
  double max_steer_angle_ = 0.6981;  
  double max_steer_rate_ = 4.0;  
  double lateral_error_gain_ = 1.5;  
  double heading_error_gain_ = 0.8;  
  double curvature_feedforward_gain_ = 1.2;  
  double smoothing_factor_ = 0.6;  
  double steering_sign_ = 1.0;  
  double direct_correction_gain_ = 0.0;
  double direct_correction_limit_ = 0.0;
  
  double pid_kp_ = 0.7;    
  double pid_ki_ = 0.03;   
  double pid_kd_ = 0.2;    
  double pid_integral_limit_ = 0.0;
  double lateral_error_integral_ = 0.0;
  double prev_lateral_error_ = 0.0;
  
  double corner_curvature_threshold_ = 0.5;  
  double corner_speed_factor_ = 0.65;        
  double corner_steer_amplify_ = 1.5;        
  double min_corner_speed_factor_ = 0.3;     
  
  double corner_approach_distance_ = 1.5;    
  double out_in_out_offset_ = 0.35;          
  bool enable_out_in_out_ = false;           
  
  bool is_overtaking_ = false;               
  double overtake_start_time_ = 0.0;         
  double overtake_max_duration_ = 3.0;       
  double overtake_lateral_offset_ = 0.55;    
  double return_to_line_distance_ = 2.0;     
  double min_overtake_gap_ = 1.0;            
  double overtake_speed_boost_ = 1.4;        
  double post_overtake_speed_factor_ = 1.2;  
  double post_overtake_duration_ = 2.0;      
  double overtake_steer_max_ = 0.28;         
  double overtake_steer_decay_ = 0.5;        
  double overtake_wall_caution_dist_ = 0.5;  
  rclcpp::Time overtake_start_time_ros_;     
  rclcpp::Time overtake_end_time_ros_;       
  bool is_post_overtake_ = false;            
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr overtake_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_collision_marker_pub_;
  
  struct WallPoint {
    double x, y;
    double d_left, d_right;
    double psi;
  };
  std::vector<WallPoint> wall_data_;
  bool wall_data_loaded_ = false;
  bool use_path_interpolation_ = true;
  
  double a1_threshold_ = 0.18;             
  double a2_threshold_ = 0.50;             
  double a2_urgent_threshold_ = 0.30;      
  double a1_side_factor_ = 0.9;            
  double a2_max_steer_ratio_ = 1.0;        
  double a2_urgent_steer_ratio_ = 1.0;     
  double reverse_speed_ = 1.5;             
  double reverse_duration_ = 1.2;          
  double reverse_pause_duration_ = 0.5;    
  double a1_steer_gain_ = 0.8;             
  double a2_steer_gain_ = 1.0;             
  double a2_urgent_steer_gain_ = 1.5;      
  bool is_reversing_ = false;              
  bool is_pausing_after_reverse_ = false;  
  bool is_planning_after_pause_ = false;   
  bool is_in_a1_zone_ = false;             
  bool just_finished_reverse_ = false;     
  int reverse_cooldown_counter_ = 0;       
  static constexpr int REVERSE_COOLDOWN_CYCLES = 50;  
  rclcpp::Time reverse_start_time_;        
  rclcpp::Time pause_start_time_;          
  rclcpp::Time planning_start_time_;       
  double last_steering_before_reverse_ = 0.0;  
  double planning_duration_ = 0.3;         
  
  int reverse_steering_mode_ = 0;          
  
  double wall_repulsion_gain_ = 0.8;       
  double wall_repulsion_threshold_ = 0.20; 
  double wall_repulsion_max_steer_ = 0.3;  
  
  bool is_following_opponent_ = false;     
  double follow_distance_threshold_ = 5.0; 
  double follow_min_distance_ = 0.8;       
  double follow_speed_factor_ = 0.4;       
  double follow_min_speed_ratio_ = 0.15;   
  double speed_smooth_factor_ = 0.1;       
  double last_adjusted_speed_ = 0.0;       
  
  double acc_kp_ = 0.5;                    
  double acc_kd_ = 0.2;                    
  double target_follow_gap_ = 3.5;         
  double estimated_opponent_velocity_ = 0.0;  
  double prev_opponent_distance_ = 10.0;   
  rclcpp::Time prev_opponent_time_;        
  
  double vehicle_width_ = 0.35;            
  double vehicle_length_ = 0.5;            
  
  double narrow_road_threshold_ = 1.2;     
  double min_visibility_angle_ = 0.6;      
  double min_overtake_clearance_ = 0.0;    
  
  double last_obstacle_left_dist_ = 10.0;   
  double last_obstacle_right_dist_ = 10.0;  
  double last_obstacle_front_dist_ = 10.0;  
  double last_obstacle_angle_ = 0.0;        
  
  static constexpr double LIDAR_BOX_SAFETY_FRONT = 0.18;    
  static constexpr double LIDAR_BOX_SAFETY_WIDTH = 0.7;     
  
  static constexpr double LIDAR_BOX_FOLLOW_FRONT = 5.0;     
  static constexpr double LIDAR_BOX_FOLLOW_WIDTH = 1.0;     
  
  static constexpr double LIDAR_BOX_OVERTAKE_FRONT = 4.0;   
  static constexpr double LIDAR_BOX_OVERTAKE_WIDTH = 2.0;   
  
  bool opponent_in_safety_box_ = false;      
  bool opponent_in_follow_box_ = false;      
  bool overtake_path_clear_ = false;         
  
  bool check_lidar_box_occupancy(double front_dist, double lateral_width, double angle_range);
  void update_virtual_boxes();               
  
  
  bool enable_collision_avoidance_ = true;  
  
  std::string odom_topic_{"/odom"};
  std::string path_topic_{"/global_raceline"};
  std::string drive_topic_{"/drive"};
  std::string scan_topic_{"/scan"};
  
  double prev_steering_angle_ = 0.0;
  rclcpp::Time prev_time_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void mode_callback(const std_msgs::msg::String::SharedPtr msg);  
  void control_loop();
  
  bool is_obstacle_stop_mode() const;  
  void execute_obstacle_stop();        
  
  int find_target_point_index(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Path& path, double adaptive_lookahead);
  double compute_adaptive_lookahead(double speed);
  double compute_lateral_error(double current_x, double current_y, double current_yaw, const nav_msgs::msg::Path& path, int closest_idx);
  double compute_heading_error(double current_yaw, const nav_msgs::msg::Path& path, int closest_idx);
  double compute_path_curvature(const nav_msgs::msg::Path& path, int idx);
  geometry_msgs::msg::PoseStamped interpolate_path_point(const nav_msgs::msg::Path& path, int idx, double fraction);
  double compute_pid_control(double error, double dt);
  double limit_steering_rate(double desired_steering, double dt);
  double smooth_steering(double new_steering, double prev_steering);
  int find_closest_point_along_path(double current_x, double current_y, double current_yaw, const nav_msgs::msg::Path& path, int start_idx);
  bool check_a1_zone();                     
  bool check_a2_zone();                     
  double compute_reverse_steering();        
  void update_obstacle_distances();         
  double find_gap_center_angle(double lookahead_angle);  
  void publish_lookahead_marker(double x, double y, double z);  
  double compute_opponent_following_speed(double base_speed);  
  double smooth_speed_change(double target_speed, double current_speed);  
  double compute_overtake_speed(double base_speed);  
  void check_and_start_overtake();  
  void update_overtake_state();  
  
  bool load_wall_data(const std::string& csv_file);  
  size_t find_closest_wall_point(double x, double y);  
  double get_wall_distance_left(double x, double y);  
  double get_wall_distance_right(double x, double y); 
  bool detect_upcoming_corner(int current_idx, double& corner_direction);  
  double compute_out_in_out_offset(int current_idx, double corner_direction);  
  bool can_overtake_safely(double opponent_dist, double opponent_angle);  
  double compute_overtake_steering(double base_steering);  
  bool should_return_to_line();  
  double compute_return_to_line_steering();  
  void publish_overtake_path(double offset_direction);  
  void publish_wall_collision_indicator(bool is_colliding, double x, double y);  
  bool validate_overtake_path(double overtake_direction);  
  std::vector<geometry_msgs::msg::Point> generate_overtake_trajectory(double overtake_direction);  
  
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);  
  bool check_imu_collision();                  
  bool check_stall_condition();                
  double compute_inverted_reverse_steering();  
  void update_crsm_state();                    
  void publish_crsm_state();                   
  void publish_safety_zone_markers();          
  
  double compute_acc_following_speed(double base_speed);  
  void update_opponent_velocity_estimate();    
  
  mutable size_t last_wall_search_idx_ = 0;
  
  mutable int last_closest_idx_overtake_ = 0;
  
  double planned_overtake_direction_ = 0.0;  
  bool has_valid_overtake_plan_ = false;     
  
  double opponent_d_position_ = 0.0;         
  double opponent_s_position_ = 0.0;         
  double opponent_width_with_margin_ = 0.0;  
};
#endif  
