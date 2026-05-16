#include "mult_path_fuser/multPathFuser_node.hpp"

// For _1
using namespace std::placeholders;

mult_path_fuser::mult_path_fuser(const rclcpp::NodeOptions& options) : Node("mult_path_fuser", options) {
    // Parameters
    float x = this->declare_parameter<float>("foo", -10.0);

    target_frame = this->declare_parameter<std::string>("target_frame", "rear_axle");

    this->path_gps_sub = this->create_subscription<nav_msgs::msg::Path>("/path_gps", 5);

    this->path_vision_sub = this->create_subscription<nav_msgs::msg::Path>("/path_vision", 5);

    this->combined_path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // tf
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

// Store the latest GPS path
void mult_path_fuser::path_gps_sub(const nav_msgs::msg::Path::SharedPtr msg) {
    // store the lastest gps_path
    // note this only works because the GPS path is published at a lower rate than the other path!
    std::lock_guard<std::mutex> lock(gps_path_mtx);
    path_gps = *msg;
}
// Helper to compute 2D distance between poses
double pose_distance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}
// Fuse paths: vision preferred, append extra GPS points if vision is shorter
void mult_path_fuser::path_vision_sub(const nav_msgs::msg::Path::SharedPtr msg) {
    transform_path(*msg, target_frame);
    {
        std::lock_guard<std::mutex> vision_lock(vision_path_mtx);
        path_vision = *msg;  // Start with vision path
    }
    transform_path(path_vision, target_frame);

    // Transform GPS path into vision frame
    nav_msgs::msg::Path gps_transformed;
    {
        std::lock_guard<std::mutex> gps_lock(gps_path_mtx);
        gps_transformed = path_gps;
    }
    transform_path(gps_transformed, target_frame);

    // Start fused path with vision
    nav_msgs::msg::Path fused_path = path_vision;

    // Distance threshold in meters
    const double dist_thresh = 0.5;

    if (!gps_transformed.poses.empty()) {
        // Start from last vision point
        geometry_msgs::msg::PoseStamped last_vision_pose = fused_path.poses.back();

        for (const auto& gps_pose : gps_transformed.poses) {
            if (pose_distance(last_vision_pose, gps_pose) > dist_thresh) {
                fused_path.poses.push_back(gps_pose);
                last_vision_pose = gps_pose;  // Update last point
            }
        }
    }

    // Update vision path with fused version for next iteration
    path_vision = msg;

    combined_path_pub->publish(fused_path);
}