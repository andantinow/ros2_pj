/**
 * @file frenet_overtake_planner.cpp
 * @brief Frenet-frame based overtaking path planner with visualization
 * 
 * This node implements a sampling-based planner in the Frenet coordinate system
 * for generating overtaking trajectories. It samples multiple lateral offset
 * candidates and selects the optimal path based on a cost function considering:
 *   - Distance from centerline (prefer center)
 *   - Obstacle proximity (avoid collisions)
 *   - Smoothness (prefer gradual lane changes)
 * 
 * Coordinate System:
 *   s: Longitudinal distance along track centerline
 *   d: Lateral deviation from centerline (+left, -right)
 * 
 * Visualization:
 *   - Green (thick): Selected optimal path
 *   - Gray (thin): Valid candidate paths
 *   - Red (thin): Collision paths (rejected)
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace planning_pkg
{

/**
 * @brief Structure representing a path in Frenet coordinates
 */
struct FrenetPath
{
    std::vector<double> s;     // Longitudinal positions
    std::vector<double> d;     // Lateral positions
    std::vector<double> x;     // Cartesian X coordinates
    std::vector<double> y;     // Cartesian Y coordinates
    double cost;               // Path cost (lower is better)
    bool collision;            // Collision flag
    double d_target;           // Target lateral offset

    FrenetPath() : cost(std::numeric_limits<double>::max()), collision(false), d_target(0.0) {}
};

/**
 * @brief Simple obstacle representation
 */
struct Obstacle
{
    double x;
    double y;
    double radius;
};

class FrenetOvertakePlannerNode : public rclcpp::Node
{
public:
    FrenetOvertakePlannerNode()
    : Node("frenet_overtake_planner")
    {
        // Parameter declarations
        this->declare_parameter<double>("max_road_width", 2.0);
        this->declare_parameter<double>("d_road_w", 0.5);
        this->declare_parameter<double>("planning_horizon", 3.0);
        this->declare_parameter<double>("planning_dt", 0.2);
        this->declare_parameter<double>("planning_speed", 5.0);
        this->declare_parameter<double>("collision_radius", 0.5);
        this->declare_parameter<double>("weight_centerline", 0.5);
        this->declare_parameter<double>("weight_obstacle", 1.0);
        this->declare_parameter<double>("weight_smoothness", 0.3);
        this->declare_parameter<double>("planning_rate", 10.0);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("min_planning_speed", 1.0);
        this->declare_parameter<double>("sigmoid_steepness", 6.0);

        // Publishers
        vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planner/candidate_paths", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planner/selected_path", 10);
        cost_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/planner/path_cost", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&FrenetOvertakePlannerNode::odomCallback, this, std::placeholders::_1));

        // Planning timer
        double rate = this->get_parameter("planning_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&FrenetOvertakePlannerNode::planLoop, this));

        // Initialize demo obstacle
        demo_obstacle_.x = 5.0;
        demo_obstacle_.y = 0.5;
        demo_obstacle_.radius = 0.3;

        RCLCPP_INFO(this->get_logger(), 
            "Frenet Overtake Planner initialized with visualization");
    }

private:
    double current_speed_ = 5.0;
    double current_s_ = 0.0;
    double current_d_ = 0.0;
    Obstacle demo_obstacle_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cost_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double min_planning_speed = this->get_parameter("min_planning_speed").as_double();
        current_speed_ = std::max(min_planning_speed, msg->twist.twist.linear.x);
    }

    /**
     * @brief Convert Frenet coordinates to Cartesian
     * 
     * NOTE: This is a simplified transformation assuming a straight track along X-axis.
     * For production use with curved tracks, replace with:
     * 1. Spline interpolation along centerline waypoints
     * 2. Projection onto centerline normal at each s position
     * 
     * The transformation should be:
     *   (x, y) = centerline(s) + d * normal(s)
     * where normal(s) is the unit vector perpendicular to the centerline at s.
     */
    void frenetToCartesian(FrenetPath& fp)
    {
        fp.x.clear();
        fp.y.clear();
        for (size_t i = 0; i < fp.s.size(); ++i) {
            // Simplified transformation for straight track (demo mode)
            // Production: Use centerline spline + normal projection
            fp.x.push_back(fp.s[i]);
            fp.y.push_back(fp.d[i]);
        }
    }

    /**
     * @brief Generate a smooth lateral transition using sigmoid function
     */
    double sigmoidTransition(double t, double d_target, double transition_time)
    {
        if (transition_time <= 0.0) {
            return d_target;
        }
        // Sigmoid: d(t) = d_target * (1 / (1 + exp(-k*(t - t_mid))))
        double sigmoid_steepness = this->get_parameter("sigmoid_steepness").as_double();
        double k = sigmoid_steepness / transition_time;
        double t_mid = transition_time * 0.5;
        return d_target / (1.0 + std::exp(-k * (t - t_mid)));
    }

    /**
     * @brief Generate a single candidate path
     */
    FrenetPath generatePath(double d_target)
    {
        FrenetPath fp;
        fp.d_target = d_target;
        fp.collision = false;

        double speed = this->get_parameter("planning_speed").as_double();
        double horizon = this->get_parameter("planning_horizon").as_double();
        double dt = this->get_parameter("planning_dt").as_double();

        // Generate path points
        for (double t = 0.0; t < horizon; t += dt) {
            double s_next = current_s_ + speed * t;
            double d_next = sigmoidTransition(t, d_target, horizon * 0.5);

            fp.s.push_back(s_next);
            fp.d.push_back(d_next);
        }

        frenetToCartesian(fp);
        return fp;
    }

    /**
     * @brief Check for collision and calculate path cost
     */
    void evaluatePath(FrenetPath& fp, const Obstacle& obs)
    {
        double collision_radius = this->get_parameter("collision_radius").as_double();
        double w_center = this->get_parameter("weight_centerline").as_double();
        double w_obs = this->get_parameter("weight_obstacle").as_double();
        double w_smooth = this->get_parameter("weight_smoothness").as_double();

        // 1. Centerline deviation cost
        double centerline_cost = std::abs(fp.d_target);

        // 2. Obstacle proximity cost & collision check
        double min_dist_to_obs = std::numeric_limits<double>::max();
        for (size_t i = 0; i < fp.x.size(); ++i) {
            double dist = std::hypot(fp.x[i] - obs.x, fp.y[i] - obs.y);
            min_dist_to_obs = std::min(min_dist_to_obs, dist);

            if (dist < collision_radius + obs.radius) {
                fp.collision = true;
            }
        }

        double obstacle_cost = (min_dist_to_obs > 0.1) ? (1.0 / min_dist_to_obs) : 1000.0;

        // 3. Smoothness cost (penalize large lateral changes)
        double smoothness_cost = std::abs(fp.d_target - current_d_);

        // Calculate total cost
        if (fp.collision) {
            fp.cost = std::numeric_limits<double>::max();
        } else {
            fp.cost = w_center * centerline_cost 
                    + w_obs * obstacle_cost 
                    + w_smooth * smoothness_cost;
        }
    }

    void planLoop()
    {
        std::vector<FrenetPath> all_paths;
        FrenetPath best_path;
        double min_cost = std::numeric_limits<double>::max();

        // 1. Path Sampling: Generate paths across the road width
        double max_width = this->get_parameter("max_road_width").as_double();
        double d_step = this->get_parameter("d_road_w").as_double();

        for (double d_target = -max_width; d_target <= max_width; d_target += d_step) {
            FrenetPath fp = generatePath(d_target);
            evaluatePath(fp, demo_obstacle_);
            all_paths.push_back(fp);

            if (!fp.collision && fp.cost < min_cost) {
                min_cost = fp.cost;
                best_path = fp;
            }
        }

        // 2. Visualization (Key user requirement)
        publishVisualization(all_paths, best_path);

        // 3. Publish selected path
        if (min_cost < std::numeric_limits<double>::max()) {
            publishPath(best_path);
            publishCost(min_cost);
        }

        // Update current lateral offset for smoothness calculation
        if (!best_path.d.empty()) {
            current_d_ = best_path.d_target;
        }
    }

    /**
     * @brief Publish candidate paths with color coding for debugging
     */
    void publishVisualization(const std::vector<FrenetPath>& paths, const FrenetPath& best)
    {
        visualization_msgs::msg::MarkerArray markers;
        std::string frame_id = this->get_parameter("frame_id").as_string();
        int id = 0;

        // Publish all candidate paths
        for (const auto& path : paths) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = this->now();
            marker.ns = "frenet_candidates";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;

            // Color coding:
            // - Green (thick): Selected path
            // - Red: Collision path
            // - Gray: Valid candidates
            bool is_best = (std::abs(path.d_target - best.d_target) < 0.01 && !path.collision);

            if (is_best) {
                marker.scale.x = 0.08;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                marker.ns = "best_path";
            } else if (path.collision) {
                marker.scale.x = 0.02;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.5f;
            } else {
                marker.scale.x = 0.02;
                marker.color.r = 0.7f;
                marker.color.g = 0.7f;
                marker.color.b = 0.7f;
                marker.color.a = 0.3f;
            }

            // Add points to the marker
            for (size_t i = 0; i < path.x.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = path.x[i];
                p.y = path.y[i];
                p.z = 0.0;
                marker.points.push_back(p);
            }

            marker.lifetime = rclcpp::Duration::from_seconds(0.15);
            markers.markers.push_back(marker);
        }

        // Publish obstacle marker
        visualization_msgs::msg::Marker obs_marker;
        obs_marker.header.frame_id = frame_id;
        obs_marker.header.stamp = this->now();
        obs_marker.ns = "obstacle";
        obs_marker.id = 1000;
        obs_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        obs_marker.action = visualization_msgs::msg::Marker::ADD;
        obs_marker.pose.position.x = demo_obstacle_.x;
        obs_marker.pose.position.y = demo_obstacle_.y;
        obs_marker.pose.position.z = 0.25;
        obs_marker.pose.orientation.w = 1.0;
        obs_marker.scale.x = demo_obstacle_.radius * 2;
        obs_marker.scale.y = demo_obstacle_.radius * 2;
        obs_marker.scale.z = 0.5;
        obs_marker.color.r = 1.0f;
        obs_marker.color.g = 0.3f;
        obs_marker.color.b = 0.0f;
        obs_marker.color.a = 0.8f;
        obs_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
        markers.markers.push_back(obs_marker);

        vis_pub_->publish(markers);
    }

    void publishPath(const FrenetPath& path)
    {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = this->get_parameter("frame_id").as_string();
        msg.header.stamp = this->now();

        for (size_t i = 0; i < path.x.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = path.x[i];
            pose.pose.position.y = path.y[i];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        path_pub_->publish(msg);
    }

    void publishCost(double cost)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = cost;
        cost_pub_->publish(msg);
    }
};

}  // namespace planning_pkg

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planning_pkg::FrenetOvertakePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
