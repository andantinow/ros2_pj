#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <deque>
#include <cmath>
#include <memory>
#include <mutex>
#include <algorithm>

// Define M_PI if not available (for cross-platform compatibility)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Dual EKF Node for robust state estimation
 * 
 * This node implements a dual Extended Kalman Filter architecture:
 * 1. Local EKF (ekf_odom): Fuses wheel odometry + IMU for short-term, 
 *    drift-prone but smooth velocity estimation in odom frame
 * 2. Global EKF (ekf_map): Fuses local estimate + global pose corrections
 *    (from AMCL, scan matching, etc.) for drift-free map-frame localization
 * 
 * The dual EKF approach provides:
 * - Continuous, smooth velocity estimates from local EKF
 * - Drift-free global position from global EKF
 * - Robustness to temporary loss of global observations
 */
class DualEkfNode : public rclcpp::Node {
public:
    DualEkfNode() : Node("dual_ekf_node") {
        // Declare parameters
        declareParameters();
        
        // Initialize EKF states
        initializeLocalEkf();
        initializeGlobalEkf();
        
        // Setup subscribers
        setupSubscribers();
        
        // Setup publishers
        setupPublishers();
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Create timer for periodic update
        double update_rate = get_parameter("update_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
            std::bind(&DualEkfNode::updateCallback, this));
        
        RCLCPP_INFO(get_logger(), "DualEkfNode initialized at %.1f Hz", update_rate);
    }

private:
    // State dimensions
    static constexpr int LOCAL_STATE_DIM = 6;   // [vx, vy, yaw_rate, ax, ay, yaw_accel]
    static constexpr int GLOBAL_STATE_DIM = 6;  // [x, y, yaw, vx, vy, yaw_rate]
    
    using LocalState = Eigen::Matrix<double, LOCAL_STATE_DIM, 1>;
    using LocalCov = Eigen::Matrix<double, LOCAL_STATE_DIM, LOCAL_STATE_DIM>;
    using GlobalState = Eigen::Matrix<double, GLOBAL_STATE_DIM, 1>;
    using GlobalCov = Eigen::Matrix<double, GLOBAL_STATE_DIM, GLOBAL_STATE_DIM>;
    
    // EKF states
    LocalState local_state_;
    LocalCov local_cov_;
    GlobalState global_state_;
    GlobalCov global_cov_;
    
    // Process noise covariances
    LocalCov local_Q_;
    GlobalCov global_Q_;
    
    // Measurement noise (configured via parameters)
    double odom_vel_var_;
    double imu_yaw_rate_var_;
    double imu_accel_var_;
    double global_pose_var_;
    double global_yaw_var_;
    
    // Buffer sizes (configurable)
    size_t odom_buffer_size_ = 50;
    size_t imu_buffer_size_ = 200;
    
    // Time tracking
    rclcpp::Time last_local_update_;
    rclcpp::Time last_global_update_;
    bool local_initialized_ = false;
    bool global_initialized_ = false;
    
    // Data buffers
    std::mutex data_mutex_;
    std::deque<nav_msgs::msg::Odometry> odom_buffer_;
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_global_pose_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr amcl_pose_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Frame IDs
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    
    // Outlier rejection thresholds (Mahalanobis distance)
    double mahalanobis_threshold_ = 5.0;  // Chi-squared 95% for 3 DOF
    
    // Covariance bounds
    double min_covariance_ = 0.0001;
    double max_covariance_ = 100.0;
    
    /**
     * @brief Declare all ROS parameters
     */
    void declareParameters() {
        // Topics
        declare_parameter<std::string>("odom_topic", "/odom");
        declare_parameter<std::string>("imu_topic", "/sensors/imu/raw");
        declare_parameter<std::string>("global_pose_topic", "/amcl_pose");
        declare_parameter<std::string>("global_pose_stamped_topic", "");  // Optional: for PoseStamped format
        declare_parameter<std::string>("local_odom_output", "/dual_ekf/local_odom");
        declare_parameter<std::string>("global_odom_output", "/dual_ekf/global_odom");
        declare_parameter<std::string>("global_pose_output", "/dual_ekf/pose");
        
        // Frame IDs
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_link_frame", "base_link");
        
        // Update rate
        declare_parameter<double>("update_rate", 50.0);
        
        // Measurement noise
        declare_parameter<double>("odom_vel_variance", 0.01);
        declare_parameter<double>("imu_yaw_rate_variance", 0.001);
        declare_parameter<double>("imu_accel_variance", 0.1);
        declare_parameter<double>("global_pose_variance", 0.1);
        declare_parameter<double>("global_yaw_variance", 0.05);
        
        // Process noise
        declare_parameter<double>("local_process_noise_vel", 0.1);
        declare_parameter<double>("local_process_noise_accel", 0.5);
        declare_parameter<double>("global_process_noise_pos", 0.01);
        declare_parameter<double>("global_process_noise_vel", 0.1);
        
        // Buffer sizes
        declare_parameter<int>("odom_buffer_size", 50);
        declare_parameter<int>("imu_buffer_size", 200);
        
        // TF broadcasting
        declare_parameter<bool>("publish_tf", true);
        
        // Outlier rejection
        declare_parameter<double>("mahalanobis_threshold", 5.0);
        
        // Covariance bounds
        declare_parameter<double>("min_covariance", 0.0001);
        declare_parameter<double>("max_covariance", 100.0);
        
        // Get frame IDs
        map_frame_ = get_parameter("map_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_link_frame_ = get_parameter("base_link_frame").as_string();
        
        // Get measurement noise
        odom_vel_var_ = get_parameter("odom_vel_variance").as_double();
        imu_yaw_rate_var_ = get_parameter("imu_yaw_rate_variance").as_double();
        imu_accel_var_ = get_parameter("imu_accel_variance").as_double();
        global_pose_var_ = get_parameter("global_pose_variance").as_double();
        global_yaw_var_ = get_parameter("global_yaw_variance").as_double();
        
        // Get buffer sizes
        odom_buffer_size_ = static_cast<size_t>(get_parameter("odom_buffer_size").as_int());
        imu_buffer_size_ = static_cast<size_t>(get_parameter("imu_buffer_size").as_int());
        
        // Get outlier rejection and covariance bounds
        mahalanobis_threshold_ = get_parameter("mahalanobis_threshold").as_double();
        min_covariance_ = get_parameter("min_covariance").as_double();
        max_covariance_ = get_parameter("max_covariance").as_double();
    }
    
    /**
     * @brief Initialize local EKF state and covariance
     */
    void initializeLocalEkf() {
        local_state_.setZero();
        local_cov_.setIdentity();
        local_cov_ *= 1.0;  // Initial uncertainty
        
        // Process noise matrix
        local_Q_.setZero();
        double vel_noise = get_parameter("local_process_noise_vel").as_double();
        double accel_noise = get_parameter("local_process_noise_accel").as_double();
        local_Q_(0, 0) = vel_noise;    // vx
        local_Q_(1, 1) = vel_noise;    // vy
        local_Q_(2, 2) = vel_noise;    // yaw_rate
        local_Q_(3, 3) = accel_noise;  // ax
        local_Q_(4, 4) = accel_noise;  // ay
        local_Q_(5, 5) = accel_noise;  // yaw_accel
    }
    
    /**
     * @brief Initialize global EKF state and covariance
     */
    void initializeGlobalEkf() {
        global_state_.setZero();
        global_cov_.setIdentity();
        global_cov_ *= 10.0;  // Higher initial uncertainty
        
        // Process noise matrix
        global_Q_.setZero();
        double pos_noise = get_parameter("global_process_noise_pos").as_double();
        double vel_noise = get_parameter("global_process_noise_vel").as_double();
        global_Q_(0, 0) = pos_noise;  // x
        global_Q_(1, 1) = pos_noise;  // y
        global_Q_(2, 2) = pos_noise;  // yaw
        global_Q_(3, 3) = vel_noise;  // vx
        global_Q_(4, 4) = vel_noise;  // vy
        global_Q_(5, 5) = vel_noise;  // yaw_rate
    }
    
    /**
     * @brief Setup ROS subscribers
     */
    void setupSubscribers() {
        std::string odom_topic = get_parameter("odom_topic").as_string();
        std::string imu_topic = get_parameter("imu_topic").as_string();
        std::string global_pose_topic = get_parameter("global_pose_topic").as_string();
        std::string pose_stamped_topic = get_parameter("global_pose_stamped_topic").as_string();
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 20,
            std::bind(&DualEkfNode::odomCallback, this, std::placeholders::_1));
        
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 50,
            std::bind(&DualEkfNode::imuCallback, this, std::placeholders::_1));
        
        // Subscribe to PoseWithCovarianceStamped format if topic is configured
        // Note: Both PoseStamped and PoseWithCovarianceStamped can be configured,
        // and if both receive data, the most recent update will be used.
        // In practice, typically only one source is active.
        if (!global_pose_topic.empty()) {
            global_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                global_pose_topic, 10,
                std::bind(&DualEkfNode::globalPoseCallback, this, std::placeholders::_1));
        }
        
        // Subscribe to PoseStamped format if topic is configured
        // This is useful when the global pose source (e.g., gym_bridge) uses PoseStamped
        if (!pose_stamped_topic.empty()) {
            amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                pose_stamped_topic, 10,
                std::bind(&DualEkfNode::amclPoseCallback, this, std::placeholders::_1));
        }
        
        RCLCPP_INFO(get_logger(), "Subscribing to odom: %s, imu: %s", odom_topic.c_str(), imu_topic.c_str());
        if (!global_pose_topic.empty()) {
            RCLCPP_INFO(get_logger(), "  Global pose (PoseWithCovarianceStamped): %s", global_pose_topic.c_str());
        }
        if (!pose_stamped_topic.empty()) {
            RCLCPP_INFO(get_logger(), "  Global pose (PoseStamped): %s", pose_stamped_topic.c_str());
        }
        if (global_pose_topic.empty() && pose_stamped_topic.empty()) {
            RCLCPP_WARN(get_logger(), "  No global pose topic configured - EKF will rely on odometry dead-reckoning");
        }
    }
    
    /**
     * @brief Setup ROS publishers
     */
    void setupPublishers() {
        std::string local_odom = get_parameter("local_odom_output").as_string();
        std::string global_odom = get_parameter("global_odom_output").as_string();
        std::string global_pose = get_parameter("global_pose_output").as_string();
        
        local_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(local_odom, 10);
        global_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(global_odom, 10);
        global_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(global_pose, 10);
        
        RCLCPP_INFO(get_logger(), "Publishing local odom: %s, global odom: %s",
                    local_odom.c_str(), global_odom.c_str());
    }
    
    /**
     * @brief Odometry callback - buffer for local EKF
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_buffer_.push_back(*msg);
        if (odom_buffer_.size() > odom_buffer_size_) {
            odom_buffer_.pop_front();
        }
    }
    
    /**
     * @brief IMU callback - buffer for local EKF
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_buffer_.push_back(*msg);
        if (imu_buffer_.size() > imu_buffer_size_) {
            imu_buffer_.pop_front();
        }
    }
    
    /**
     * @brief Global pose callback (from AMCL or other localization)
     */
    void globalPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_global_pose_ = msg;
    }
    
    /**
     * @brief AMCL PoseStamped callback - convert to PoseWithCovarianceStamped
     */
    void amclPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        auto pose_cov = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        pose_cov->header = msg->header;
        pose_cov->pose.pose = msg->pose;
        
        // Set default covariance
        std::fill(pose_cov->pose.covariance.begin(), pose_cov->pose.covariance.end(), 0.0);
        pose_cov->pose.covariance[0] = global_pose_var_;   // x
        pose_cov->pose.covariance[7] = global_pose_var_;   // y
        pose_cov->pose.covariance[35] = global_yaw_var_;   // yaw
        
        latest_global_pose_ = pose_cov;
    }
    
    /**
     * @brief Main update callback - runs both EKFs
     */
    void updateCallback() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        rclcpp::Time now = this->now();
        
        // Update local EKF if we have data
        if (!odom_buffer_.empty() || !imu_buffer_.empty()) {
            updateLocalEkf(now);
        }
        
        // Update global EKF with local estimate and global corrections
        updateGlobalEkf(now);
        
        // Publish results
        publishResults(now);
    }
    
    /**
     * @brief Update local EKF with wheel odometry and IMU
     */
    void updateLocalEkf(const rclcpp::Time& now) {
        if (!local_initialized_) {
            last_local_update_ = now;
            local_initialized_ = true;
            return;
        }
        
        double dt = (now - last_local_update_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            last_local_update_ = now;
            return;
        }
        
        // === Prediction Step ===
        // State transition: velocity evolves based on acceleration
        // x_{k+1} = F * x_k
        Eigen::Matrix<double, LOCAL_STATE_DIM, LOCAL_STATE_DIM> F;
        F.setIdentity();
        F(0, 3) = dt;  // vx += ax * dt
        F(1, 4) = dt;  // vy += ay * dt
        F(2, 5) = dt;  // yaw_rate += yaw_accel * dt
        
        local_state_ = F * local_state_;
        local_cov_ = F * local_cov_ * F.transpose() + local_Q_ * dt;
        
        // Bound covariance after prediction
        boundCovariance<LOCAL_STATE_DIM>(local_cov_);
        
        // === Correction Step with Odometry ===
        if (!odom_buffer_.empty()) {
            auto& odom = odom_buffer_.back();
            double meas_vx = odom.twist.twist.linear.x;
            double meas_vy = odom.twist.twist.linear.y;
            
            // Measurement model: H = [I_3, 0_3] for velocity
            Eigen::Matrix<double, 2, LOCAL_STATE_DIM> H_odom;
            H_odom.setZero();
            H_odom(0, 0) = 1.0;  // vx
            H_odom(1, 1) = 1.0;  // vy
            
            Eigen::Matrix<double, 2, 2> R_odom;
            R_odom.setIdentity();
            R_odom *= odom_vel_var_;
            
            Eigen::Vector2d z_odom;
            z_odom << meas_vx, meas_vy;
            
            Eigen::Vector2d y_odom = z_odom - H_odom * local_state_;
            Eigen::Matrix<double, 2, 2> S_odom = H_odom * local_cov_ * H_odom.transpose() + R_odom;
            
            // Outlier rejection using Mahalanobis distance
            double mahal_dist_odom = mahalanobisDistance<2>(y_odom, S_odom);
            if (mahal_dist_odom > mahalanobis_threshold_ * mahalanobis_threshold_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Odom measurement rejected: Mahalanobis distance %.2f exceeds threshold",
                    std::sqrt(mahal_dist_odom));
            } else {
                // Use LLT decomposition for numerical stability with error checking
                Eigen::LLT<Eigen::Matrix2d> llt_odom(S_odom);
                if (llt_odom.info() == Eigen::Success) {
                    Eigen::Matrix<double, LOCAL_STATE_DIM, 2> K_odom = local_cov_ * H_odom.transpose() * llt_odom.solve(Eigen::Matrix2d::Identity());
                    
                    local_state_ = local_state_ + K_odom * y_odom;
                    
                    // Joseph form update for numerical stability: P = (I-KH)*P*(I-KH)^T + K*R*K^T
                    LocalCov I_KH = LocalCov::Identity() - K_odom * H_odom;
                    local_cov_ = I_KH * local_cov_ * I_KH.transpose() + K_odom * R_odom * K_odom.transpose();
                    
                    // Bound covariance after update
                    boundCovariance<LOCAL_STATE_DIM>(local_cov_);
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "Odom EKF update skipped: S matrix not positive definite");
                }
            }
        }
        
        // === Correction Step with IMU ===
        if (!imu_buffer_.empty()) {
            auto& imu = imu_buffer_.back();
            double meas_yaw_rate = imu.angular_velocity.z;
            double meas_ax = imu.linear_acceleration.x;
            double meas_ay = imu.linear_acceleration.y;
            
            // Measurement model for IMU
            Eigen::Matrix<double, 3, LOCAL_STATE_DIM> H_imu;
            H_imu.setZero();
            H_imu(0, 2) = 1.0;  // yaw_rate
            H_imu(1, 3) = 1.0;  // ax
            H_imu(2, 4) = 1.0;  // ay
            
            Eigen::Matrix<double, 3, 3> R_imu;
            R_imu.setZero();
            R_imu(0, 0) = imu_yaw_rate_var_;
            R_imu(1, 1) = imu_accel_var_;
            R_imu(2, 2) = imu_accel_var_;
            
            Eigen::Vector3d z_imu;
            z_imu << meas_yaw_rate, meas_ax, meas_ay;
            
            Eigen::Vector3d y_imu = z_imu - H_imu * local_state_;
            Eigen::Matrix3d S_imu = H_imu * local_cov_ * H_imu.transpose() + R_imu;
            
            // Outlier rejection using Mahalanobis distance
            double mahal_dist_imu = mahalanobisDistance<3>(y_imu, S_imu);
            if (mahal_dist_imu > mahalanobis_threshold_ * mahalanobis_threshold_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "IMU measurement rejected: Mahalanobis distance %.2f exceeds threshold",
                    std::sqrt(mahal_dist_imu));
            } else {
                // Use LLT decomposition for numerical stability with error checking
                Eigen::LLT<Eigen::Matrix3d> llt_imu(S_imu);
                if (llt_imu.info() == Eigen::Success) {
                    Eigen::Matrix<double, LOCAL_STATE_DIM, 3> K_imu = local_cov_ * H_imu.transpose() * llt_imu.solve(Eigen::Matrix3d::Identity());
                    
                    local_state_ = local_state_ + K_imu * y_imu;
                    
                    // Joseph form update for numerical stability: P = (I-KH)*P*(I-KH)^T + K*R*K^T
                    LocalCov I_KH = LocalCov::Identity() - K_imu * H_imu;
                    local_cov_ = I_KH * local_cov_ * I_KH.transpose() + K_imu * R_imu * K_imu.transpose();
                    
                    // Bound covariance after update
                    boundCovariance<LOCAL_STATE_DIM>(local_cov_);
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "IMU EKF update skipped: S matrix not positive definite");
                }
            }
        }
        
        last_local_update_ = now;
    }
    
    /**
     * @brief Update global EKF with local velocity and global pose corrections
     */
    void updateGlobalEkf(const rclcpp::Time& now) {
        if (!global_initialized_) {
            // Initialize global state from first global pose if available
            if (latest_global_pose_) {
                global_state_(0) = latest_global_pose_->pose.pose.position.x;
                global_state_(1) = latest_global_pose_->pose.pose.position.y;
                global_state_(2) = yawFromQuaternion(latest_global_pose_->pose.pose.orientation);
                global_state_(3) = local_state_(0);  // vx from local EKF
                global_state_(4) = local_state_(1);  // vy from local EKF
                global_state_(5) = local_state_(2);  // yaw_rate from local EKF
            }
            last_global_update_ = now;
            global_initialized_ = true;
            return;
        }
        
        double dt = (now - last_global_update_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            last_global_update_ = now;
            return;
        }
        
        // === Prediction Step ===
        // Use local EKF velocity for prediction in global frame
        double yaw = global_state_(2);
        double vx_local = local_state_(0);
        double vy_local = local_state_(1);
        double yaw_rate = local_state_(2);
        
        // Transform local velocity to global frame
        double vx_global = vx_local * std::cos(yaw) - vy_local * std::sin(yaw);
        double vy_global = vx_local * std::sin(yaw) + vy_local * std::cos(yaw);
        
        // State prediction
        global_state_(0) += vx_global * dt;  // x
        global_state_(1) += vy_global * dt;  // y
        global_state_(2) += yaw_rate * dt;   // yaw
        global_state_(3) = vx_local;
        global_state_(4) = vy_local;
        global_state_(5) = yaw_rate;
        
        // Normalize yaw to [-pi, pi]
        global_state_(2) = normalizeAngle(global_state_(2));
        
        // State transition Jacobian
        Eigen::Matrix<double, GLOBAL_STATE_DIM, GLOBAL_STATE_DIM> F;
        F.setIdentity();
        F(0, 2) = -vx_local * std::sin(yaw) * dt - vy_local * std::cos(yaw) * dt;
        F(0, 3) = std::cos(yaw) * dt;
        F(0, 4) = -std::sin(yaw) * dt;
        F(1, 2) = vx_local * std::cos(yaw) * dt - vy_local * std::sin(yaw) * dt;
        F(1, 3) = std::sin(yaw) * dt;
        F(1, 4) = std::cos(yaw) * dt;
        F(2, 5) = dt;
        
        global_cov_ = F * global_cov_ * F.transpose() + global_Q_ * dt;
        
        // Bound covariance after prediction
        boundCovariance<GLOBAL_STATE_DIM>(global_cov_);
        
        // === Correction Step with Global Pose ===
        if (latest_global_pose_) {
            double meas_x = latest_global_pose_->pose.pose.position.x;
            double meas_y = latest_global_pose_->pose.pose.position.y;
            double meas_yaw = yawFromQuaternion(latest_global_pose_->pose.pose.orientation);
            
            // Measurement model: H = [I_3, 0_3] for pose
            Eigen::Matrix<double, 3, GLOBAL_STATE_DIM> H;
            H.setZero();
            H(0, 0) = 1.0;  // x
            H(1, 1) = 1.0;  // y
            H(2, 2) = 1.0;  // yaw
            
            // Use covariance from message if available
            Eigen::Matrix3d R;
            R.setZero();
            R(0, 0) = latest_global_pose_->pose.covariance[0];
            R(1, 1) = latest_global_pose_->pose.covariance[7];
            R(2, 2) = latest_global_pose_->pose.covariance[35];
            
            // Ensure minimum covariance
            R(0, 0) = std::max(R(0, 0), 0.001);
            R(1, 1) = std::max(R(1, 1), 0.001);
            R(2, 2) = std::max(R(2, 2), 0.0001);
            
            Eigen::Vector3d z;
            z << meas_x, meas_y, meas_yaw;
            
            Eigen::Vector3d y = z - H * global_state_;
            // Normalize yaw innovation
            y(2) = normalizeAngle(y(2));
            
            Eigen::Matrix3d S = H * global_cov_ * H.transpose() + R;
            
            // Outlier rejection using Mahalanobis distance
            double mahal_dist_global = mahalanobisDistance<3>(y, S);
            if (mahal_dist_global > mahalanobis_threshold_ * mahalanobis_threshold_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Global pose measurement rejected: Mahalanobis distance %.2f exceeds threshold",
                    std::sqrt(mahal_dist_global));
            } else {
                // Use LLT decomposition for numerical stability with error checking
                Eigen::LLT<Eigen::Matrix3d> llt_global(S);
                if (llt_global.info() == Eigen::Success) {
                    Eigen::Matrix<double, GLOBAL_STATE_DIM, 3> K = global_cov_ * H.transpose() * llt_global.solve(Eigen::Matrix3d::Identity());
                    
                    global_state_ = global_state_ + K * y;
                    global_state_(2) = normalizeAngle(global_state_(2));
                    
                    // Joseph form update for numerical stability: P = (I-KH)*P*(I-KH)^T + K*R*K^T
                    GlobalCov I_KH = GlobalCov::Identity() - K * H;
                    global_cov_ = I_KH * global_cov_ * I_KH.transpose() + K * R * K.transpose();
                    
                    // Bound covariance after update
                    boundCovariance<GLOBAL_STATE_DIM>(global_cov_);
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Global EKF update skipped: S matrix not positive definite");
                }
            }
            
            // Clear the latest pose after using it
            latest_global_pose_.reset();
        }
        
        last_global_update_ = now;
    }
    
    /**
     * @brief Publish EKF results
     */
    void publishResults(const rclcpp::Time& now) {
        // Publish local odometry (velocity in odom frame)
        nav_msgs::msg::Odometry local_odom;
        local_odom.header.stamp = now;
        local_odom.header.frame_id = odom_frame_;
        local_odom.child_frame_id = base_link_frame_;
        local_odom.twist.twist.linear.x = local_state_(0);
        local_odom.twist.twist.linear.y = local_state_(1);
        local_odom.twist.twist.angular.z = local_state_(2);
        
        // Set twist covariance
        local_odom.twist.covariance[0] = local_cov_(0, 0);   // vx
        local_odom.twist.covariance[7] = local_cov_(1, 1);   // vy
        local_odom.twist.covariance[35] = local_cov_(2, 2);  // yaw_rate
        
        local_odom_pub_->publish(local_odom);
        
        // Publish global odometry (full pose in map frame)
        nav_msgs::msg::Odometry global_odom;
        global_odom.header.stamp = now;
        global_odom.header.frame_id = map_frame_;
        global_odom.child_frame_id = base_link_frame_;
        
        global_odom.pose.pose.position.x = global_state_(0);
        global_odom.pose.pose.position.y = global_state_(1);
        global_odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, global_state_(2));
        global_odom.pose.pose.orientation.x = q.x();
        global_odom.pose.pose.orientation.y = q.y();
        global_odom.pose.pose.orientation.z = q.z();
        global_odom.pose.pose.orientation.w = q.w();
        
        global_odom.twist.twist.linear.x = global_state_(3);
        global_odom.twist.twist.linear.y = global_state_(4);
        global_odom.twist.twist.angular.z = global_state_(5);
        
        // Set covariances
        global_odom.pose.covariance[0] = global_cov_(0, 0);
        global_odom.pose.covariance[7] = global_cov_(1, 1);
        global_odom.pose.covariance[35] = global_cov_(2, 2);
        global_odom.twist.covariance[0] = global_cov_(3, 3);
        global_odom.twist.covariance[7] = global_cov_(4, 4);
        global_odom.twist.covariance[35] = global_cov_(5, 5);
        
        global_odom_pub_->publish(global_odom);
        
        // Publish pose
        geometry_msgs::msg::PoseStamped global_pose;
        global_pose.header = global_odom.header;
        global_pose.pose = global_odom.pose.pose;
        global_pose_pub_->publish(global_pose);
        
        // Publish TF if enabled
        if (get_parameter("publish_tf").as_bool()) {
            publishTf(now);
        }
    }
    
    /**
     * @brief Publish map -> odom transform
     * 
     * This computes the correction transform between the map frame and odom frame.
     * The transform represents: map = T_map_odom * odom
     * 
     * In a dual EKF setup:
     * - odom frame accumulates drift from wheel odometry
     * - map frame is the global reference from localization
     * - The map->odom transform corrects for accumulated drift
     */
    void publishTf(const rclcpp::Time& now) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        
        // Get the latest odom position (robot pose in odom frame)
        double odom_x = 0.0, odom_y = 0.0, odom_yaw = 0.0;
        if (!odom_buffer_.empty()) {
            const auto& odom = odom_buffer_.back();
            odom_x = odom.pose.pose.position.x;
            odom_y = odom.pose.pose.position.y;
            odom_yaw = yawFromQuaternion(odom.pose.pose.orientation);
        }
        
        // Global state gives robot pose in map frame
        double map_x = global_state_(0);
        double map_y = global_state_(1);
        double map_yaw = global_state_(2);
        
        // Compute map->odom transform
        // If robot is at (odom_x, odom_y, odom_yaw) in odom frame
        // and at (map_x, map_y, map_yaw) in map frame
        // then map->odom transform T satisfies: map_pose = T * odom_pose
        // 
        // For the transform, we need: T_map_odom = T_map_robot * T_robot_odom^(-1)
        double dyaw = normalizeAngle(map_yaw - odom_yaw);
        double cos_dyaw = std::cos(dyaw);
        double sin_dyaw = std::sin(dyaw);
        
        // Transform odom origin to map frame
        double dx = map_x - (odom_x * cos_dyaw - odom_y * sin_dyaw);
        double dy = map_y - (odom_x * sin_dyaw + odom_y * cos_dyaw);
        
        transform.transform.translation.x = dx;
        transform.transform.translation.y = dy;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, dyaw);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    /**
     * @brief Extract yaw from quaternion
     */
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    /**
     * @brief Normalize angle to [-pi, pi] using efficient modulo operation
     */
    static double normalizeAngle(double angle) {
        // Use fmod for efficiency with large angles
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }
    
    /**
     * @brief Bound covariance matrix to prevent numerical issues
     * Ensures all diagonal elements are within [min_cov, max_cov]
     */
    template<int N>
    void boundCovariance(Eigen::Matrix<double, N, N>& cov) {
        for (int i = 0; i < N; ++i) {
            cov(i, i) = std::clamp(cov(i, i), min_covariance_, max_covariance_);
        }
        
        // Ensure symmetry
        cov = (cov + cov.transpose()) / 2.0;
        
        // Ensure positive semi-definite (simple check)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> solver(cov);
        if (solver.info() == Eigen::Success) {
            Eigen::Matrix<double, N, 1> eigenvalues = solver.eigenvalues();
            bool needs_fix = false;
            for (int i = 0; i < N; ++i) {
                if (eigenvalues(i) < min_covariance_) {
                    eigenvalues(i) = min_covariance_;
                    needs_fix = true;
                }
            }
            if (needs_fix) {
                cov = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
            }
        }
    }
    
    /**
     * @brief Calculate Mahalanobis distance for outlier rejection
     * d² = y' * S^(-1) * y
     */
    template<int N>
    double mahalanobisDistance(const Eigen::Matrix<double, N, 1>& innovation,
                                const Eigen::Matrix<double, N, N>& innovation_cov) {
        Eigen::LLT<Eigen::Matrix<double, N, N>> llt(innovation_cov);
        if (llt.info() != Eigen::Success) {
            return std::numeric_limits<double>::max();  // Invalid, reject
        }
        Eigen::Matrix<double, N, 1> v = llt.matrixL().solve(innovation);
        return v.squaredNorm();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualEkfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
