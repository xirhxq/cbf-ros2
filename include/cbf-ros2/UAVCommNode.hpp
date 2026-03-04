/**
 * @file UAVCommNode.hpp
 * @brief ROS2 node for UAV communication (velocity commands and pose subscription)
 */

#ifndef CBF_ROS2_UAV_COMM_NODE_HPP
#define CBF_ROS2_UAV_COMM_NODE_HPP

#include <memory>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cbf-ros2/Utils.h"

class UAVCommNode : public rclcpp::Node {
public:
    UAVCommNode(const std::string &id)
        : Node("uav_comm_" + id), id_(id), yaw_(0.0) {
        vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("uav_" + id_ + "/cmd_vel", 10);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "uav_" + id_ + "/pose/groundtruth", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_pose_ = *msg;
                const auto& q = msg->pose.orientation;
                yaw_ = quaternionToYaw(q.w, q.x, q.y, q.z);
            });
    }

    void spin() {
        rclcpp::spin(shared_from_this());
    }

    void publish_velocity(const Eigen::Vector3d &velocity, double yaw_rate = 0.0) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = velocity.x();
        cmd.linear.y = velocity.y();
        cmd.linear.z = velocity.z();
        cmd.angular.z = yaw_rate;
        vel_cmd_pub_->publish(cmd);
    }

    Eigen::Vector3d get_last_pose() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return Eigen::Vector3d(
            last_pose_.pose.position.x,
            last_pose_.pose.position.y,
            last_pose_.pose.position.z);
    }

    double get_yaw() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return yaw_;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    std::string id_;
    double yaw_;

    mutable std::mutex data_mutex_;
    geometry_msgs::msg::PoseStamped last_pose_;
};

#endif // CBF_ROS2_UAV_COMM_NODE_HPP
