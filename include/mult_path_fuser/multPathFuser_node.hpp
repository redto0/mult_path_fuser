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
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_preferred_subscription;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_secondary_subscription;

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

    /// Transforms a path that may be stale into target_frame as of target_time, bridging the age
    /// gap through fixed_frame (odom) so the result reflects how far the vehicle has actually
    /// travelled since the path was stamped, rather than just the latest known transform. Used
    /// for both paths symmetrically since either may arrive async/stale and which one is
    /// "preferred" vs "secondary" is a fusion-policy choice, not a freshness guarantee.
    void transform_path(nav_msgs::msg::Path& path, const std::string& target_frame, const rclcpp::Time& target_time,
                         const std::string& fixed_frame) {
        if (path.poses.empty()) {
            path.header.frame_id = target_frame;
            return;
        }

        auto trans = this->tf_buffer->lookupTransform(target_frame, target_time, path.header.frame_id,
                                                       rclcpp::Time(path.header.stamp), fixed_frame);

        for (auto& pose : path.poses) {
            tf2::doTransform(pose, pose, trans);
            pose.header.frame_id = target_frame;
            pose.header.stamp = target_time;
        }
        path.header.frame_id = target_frame;
        path.header.stamp = target_time;
    }

    /// Path lock
    std::mutex secondary_path_mtx;
    /// Odom lock
    std::mutex preferred_path_mtx;
    /// Objects lock
    std::mutex odom_mtx;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    /// Re-fuses and publishes using the latest of both paths. Called from both subscriber
    /// callbacks so a fresh arrival of either path triggers a re-fuse, not just the preferred one.
    void fuse_and_publish();

public:
    mult_path_fuser(const rclcpp::NodeOptions& options);

    nav_msgs::msg::Path path_preferred;
    nav_msgs::msg::Path path_secondary;

    std::string target_frame;
    /// Fixed frame used to bridge the age of a stale secondary (e.g. GPS) path to the present.
    std::string odom_frame;

    /// subscriber callback
    void path_preferred_sub(const nav_msgs::msg::Path::SharedPtr msg);
    void path_secondary_sub(const nav_msgs::msg::Path::SharedPtr msg);
    // void combined_path_pub()
};
