/**
 * @file UAVCommNode.hpp
 * @brief ROS2 node for UAV communication (velocity commands and pose/IMU subscription)
 */

#ifndef CBF_ROS2_UAV_COMM_NODE_HPP
#define CBF_ROS2_UAV_COMM_NODE_HPP

#include <memory>
#include <mutex>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cbf-ros2/Utils.h"
#include "cbf-ros2/Config.hpp"

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

        // Subscribe to IMU for rotation matrix
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "uav_" + id_ + "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                const auto& q = msg->orientation;
                double roll, pitch, yaw;
                quaternionToEuler(q.w, q.x, q.y, q.z, roll, pitch, yaw);
                eulerToDcm(roll, pitch, yaw, R_e2b_);
            });

        // Initialize rotation matrix to identity
        std::memset(R_e2b_, 0, sizeof(R_e2b_));
        R_e2b_[0][0] = R_e2b_[1][1] = R_e2b_[2][2] = 1.0;
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

    /**
     * @brief Publish velocity command, converting from earth frame to body frame
     * @param velocity Earth-frame velocity
     * @param yaw_rate Yaw rate command
     */
    void publish_velocity_earth(const Eigen::Vector3d &velocity, double yaw_rate = 0.0) {
        geometry_msgs::msg::Twist cmd;

        // Velocity limits from Config.hpp
        const double MAX_HORIZONTAL_VEL_EARTH = cbf_ros2::config::velocity::MAX_HORIZONTAL_EARTH_MPS;
        const double MAX_VERTICAL_VEL_EARTH = cbf_ros2::config::velocity::MAX_VERTICAL_EARTH_MPS;
        const double MAX_HORIZONTAL_VEL_BODY = cbf_ros2::config::velocity::MAX_HORIZONTAL_BODY_MPS;
        const double MAX_VERTICAL_VEL_BODY = cbf_ros2::config::velocity::MAX_VERTICAL_BODY_MPS;

        // Step 1: Limit velocity in earth frame before transformation
        double vx = velocity.x(), vy = velocity.y(), vz = velocity.z();
        double horizontal_speed = std::sqrt(vx * vx + vy * vy);
        if (horizontal_speed > MAX_HORIZONTAL_VEL_EARTH) {
            double scale = MAX_HORIZONTAL_VEL_EARTH / horizontal_speed;
            vx *= scale;
            vy *= scale;
        }
        if (std::abs(vz) > MAX_VERTICAL_VEL_EARTH) {
            vz = std::copysign(MAX_VERTICAL_VEL_EARTH, vz);
        }

        // Step 2: Full earth-to-body transformation
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            earthToBody(vx, vy, vz, R_e2b_);
        }

        // Step 3: Limit velocity in body frame after transformation
        horizontal_speed = std::sqrt(vx * vx + vy * vy);
        if (horizontal_speed > MAX_HORIZONTAL_VEL_BODY) {
            double scale = MAX_HORIZONTAL_VEL_BODY / horizontal_speed;
            vx *= scale;
            vy *= scale;
        }
        if (std::abs(vz) > MAX_VERTICAL_VEL_BODY) {
            vz = std::copysign(MAX_VERTICAL_VEL_BODY, vz);
        }

        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.linear.z = vz;
        cmd.angular.z = yaw_rate;
        vel_cmd_pub_->publish(cmd);

        // Store body frame velocity for debugging
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_body_vel_ = Eigen::Vector3d(vx, vy, vz);
    }

    Eigen::Vector3d get_last_body_vel() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return last_body_vel_;
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::string id_;
    double yaw_;

    mutable std::mutex data_mutex_;
    geometry_msgs::msg::PoseStamped last_pose_;
    double R_e2b_[3][3];  // Rotation matrix: ENU (earth) to Body frame
    Eigen::Vector3d last_body_vel_;  // Last body frame velocity command
};

#endif // CBF_ROS2_UAV_COMM_NODE_HPP
