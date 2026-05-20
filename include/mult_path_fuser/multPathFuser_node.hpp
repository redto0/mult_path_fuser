#pragma once

#include <tf2/LinearMath/Transform.h>    // (optional) low‑level transform if needed
#include <tf2/time.h>                    // for tf2::TimePointZero
#include <tf2_ros/buffer.h>              // for tf2_ros::Buffer
#include <tf2_ros/transform_listener.h>  // for tf2_ros::TransformListener

#include <geometry_msgs/msg/pose_stamped.hpp>       // for geometry_msgs::msg::PoseStamped
#include <memory>                                   // for std::shared_ptr, std::unique_ptr
#include <mutex>                                    // for std::mutex
#include <nav_msgs/msg/path.hpp>                    // for nav_msgs::msg::Path
#include <rclcpp/rclcpp.hpp>                        // for rclcpp::Node, rclcpp::Publisher, etc.
#include <std_msgs/msg/string.hpp>                  // for std_msgs::msg::String
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // for tf2::doTransform on geometry_msgs

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class mult_path_fuser : public rclcpp::Node {
private:
    // pubs
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr combined_path_pub;

    // subs
    // rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_vision_sub;

    // rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_gps_sub;

    /// Transforms a path into a given target frame.
    void transform_path(nav_msgs::msg::Path& path, std::string target_frame) {
        auto trans = this->tf_buffer->lookupTransform(target_frame, path.header.frame_id, tf2::TimePointZero);

        for (auto& pose : path.poses) {
            tf2::doTransform(pose, pose, trans);
            pose.header.frame_id = target_frame;
        }
        path.header.frame_id = target_frame;
    }

    /// Transforms a spline into a given target frame.
    void transform_path(std::vector<geometry_msgs::msg::PoseStamped>& path, std::string target_frame) {
        if (path.empty()) {
            return;
        }

        auto trans = this->tf_buffer->lookupTransform(target_frame, path[0].header.frame_id, tf2::TimePointZero);

        for (auto& pose : path) {
            tf2::doTransform(pose, pose, trans);
            pose.header.frame_id = target_frame;
        }
    }

    /// Path lock
    std::mutex gps_path_mtx;
    /// Odom lock
    std::mutex vision_path_mtx;
    /// Objects lock
    std::mutex odom_mtx;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

public:
    mult_path_fuser(const rclcpp::NodeOptions& options);

    nav_msgs::msg::Path path_vision;
    nav_msgs::msg::Path path_gps;

    std::string target_frame;

    /// subscriber callback
    void path_vision_sub(const nav_msgs::msg::Path::SharedPtr msg);
    void path_gps_sub(const nav_msgs::msg::Path::SharedPtr msg);
    // void combined_path_pub()
};
