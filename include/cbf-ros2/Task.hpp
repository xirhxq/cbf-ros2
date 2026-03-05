/**
 * @file Task.hpp
 * @brief Task class for UAV state machine and control
 */

#ifndef CBF_ROS2_TASK_HPP
#define CBF_ROS2_TASK_HPP

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "cbf-ros2/UAVCommNode.hpp"
#include "cbf-ros2/Utils.h"

class Task {
public:
    Task(const json& settings, double target_yaw_deg)
        : id_(settings["id"]),
          uav_comm_(std::make_shared<UAVCommNode>(id_)),
          stop_flag_(false),
          target_yaw_(target_yaw_deg * M_PI / 180.0),
          yaw_rate_cmd_(0.0),
          yaw_rate_from_cbf_(0.0),
          current_state_(State::INIT) {

        auto arr = settings["prepare_point"].get<std::vector<double>>();
        prepare_point_ = Eigen::Vector3d(arr.data());
        search_height_ = prepare_point_.z();

        spin_thread_ = std::thread([this]() {
            uav_comm_->spin();
        });
    }

    ~Task() {
        stop();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    void runOnce() {
        if (!stop_flag_) {
            update();
        }
    }

    void stop() {
        stop_flag_ = true;
    }

    bool isInPerform() const {
        return current_state_ == State::PERFORM;
    }

    bool isInLand() const {
        return current_state_ == State::LAND;
    }

    bool isInBack() const {
        return current_state_ == State::BACK;
    }

    void endPerform() {
        if (!isInPerform()) return;
        transition_to(State::BACK);
    }

    // Set CBF control commands (called from main loop)
    void setCBFControl(const Eigen::Vector2d &velocity, double yaw_rate) {
        velocity_2d_cmd_ = velocity;
        yaw_rate_from_cbf_ = yaw_rate;
    }

    Eigen::Vector3d getPosition() const {
        return uav_comm_->get_last_pose();
    }

    double getYaw() const {
        return uav_comm_->get_yaw();
    }

    std::string getId() const { return id_; }

private:
    enum class State {
        INIT,
        TAKEOFF,
        PREPARE,
        PERFORM,
        BACK,
        LAND
    };

    void update() {
        Eigen::Vector3d current_pose = uav_comm_->get_last_pose();
        double current_yaw = uav_comm_->get_yaw();

        switch (current_state_) {
            case State::INIT:
                if (current_pose.norm() > 1e-3) {
                spawn_point_ = current_pose;
                takeoff_point_ = spawn_point_ + Eigen::Vector3d(0.0, 0.0, 20.0);
                transition_to(State::TAKEOFF);
            }
            yaw_rate_cmd_ = 0.0;
            break;
            case State::TAKEOFF:
                control_to_point(current_pose, takeoff_point_);
                yaw_rate_cmd_ = 0.0;
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::PREPARE);
                }
                break;
            case State::PREPARE:
                control_to_point(current_pose, prepare_point_);
                control_yaw_to_target(current_yaw);
                if (is_at_point(current_pose, prepare_point_) && is_at_yaw(current_yaw)) {
                    transition_to(State::PERFORM);
                }
                break;
            case State::PERFORM:
                velocity2dControl(current_pose, velocity_2d_cmd_);
                yaw_rate_cmd_ = yaw_rate_from_cbf_;
                break;
            case State::BACK:
                control_to_point(current_pose, takeoff_point_);
                yaw_rate_cmd_ = 0.0;
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::LAND);
                }
                break;
            case State::LAND:
                control_to_point(current_pose, spawn_point_);
                yaw_rate_cmd_ = 0.0;
                break;
        }

        // Convert world frame velocity to body frame before publishing
        // MBZIRC cmd_vel expects body frame velocities
        uav_comm_->publish_velocity_earth(velocity_cmd_, yaw_rate_cmd_);

        std::lock_guard<std::mutex> lock(log_mutex_);
        double yaw_deg = current_yaw * 180.0 / M_PI;
        printf(
            "#%s | Pos: (%.1f, %.1f, %.1f) | Yaw: %.1f° | State: %s | Vel: (%.2f, %.2f, %.2f) | YawRate: %.2f\n",
            id_.c_str(), current_pose.x(), current_pose.y(), current_pose.z(),
            yaw_deg, state_to_string(current_state_).c_str(),
            velocity_cmd_.x(), velocity_cmd_.y(), velocity_cmd_.z(),
            yaw_rate_cmd_
        );
    }

    void control_to_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose) {
        Eigen::Vector3d delta = target_pose - current_pose;

        double kp = 1.0;       // Increased from 0.2 for faster response
        double max_speed = 25.0; // Increased from 5.0 to match simulation speed limit
        Eigen::Vector3d vel = kp * delta;

        if (vel.norm() > max_speed) {
            vel = vel.normalized() * max_speed;
        }

        velocity_cmd_ = vel;
    }

    void velocity2dControl(const Eigen::Vector3d &current_pose, const Eigen::Vector2d &velocity) {
        double kp = 1.0;  // Increased from 0.2 for faster altitude response
        velocity_cmd_ = Eigen::Vector3d(velocity.x(), velocity.y(), kp * (search_height_ - current_pose.z()));
    }

    void control_yaw_to_target(double current_yaw) {
        double yaw_error = normalizeAngle(target_yaw_ - current_yaw);

        double kp_yaw = 1.0;
        double max_yaw_rate = 1.0;

        yaw_rate_cmd_ = kp_yaw * yaw_error;

        if (std::abs(yaw_rate_cmd_) > max_yaw_rate) {
            yaw_rate_cmd_ = std::copysign(max_yaw_rate, yaw_rate_cmd_);
        }
    }

    bool is_at_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose, double tolerance = 0.5) {
        return (current_pose - target_pose).norm() < tolerance;
    }

    bool is_at_yaw(double current_yaw, double tolerance = 0.1) {
        double yaw_error = normalizeAngle(target_yaw_ - current_yaw);
        return std::abs(yaw_error) < tolerance;
    }

    void transition_to(State new_state) {
        current_state_ = new_state;
        std::lock_guard<std::mutex> lock(log_mutex_);
        std::cout << "UAV #" << id_ << " transitioned to: " << state_to_string(current_state_) << std::endl;
    }

    std::string state_to_string(State state) {
        switch (state) {
            case State::INIT: return "INIT";
            case State::TAKEOFF: return "TAKEOFF";
            case State::PREPARE: return BLUE + "PREPARE" + RESET;
            case State::PERFORM: return GREEN + "PERFORM" + RESET;
            case State::BACK: return "BACK";
            case State::LAND: return "LAND";
            default: return "UNKNOWN";
        }
    }

    std::string id_;
    std::shared_ptr<UAVCommNode> uav_comm_;
    std::atomic<bool> stop_flag_;
    std::thread spin_thread_;
    State current_state_;

    double search_height_;
    double target_yaw_;  // Target yaw in radians
    double yaw_rate_cmd_;
    double yaw_rate_from_cbf_;

    Eigen::Vector3d spawn_point_;
    Eigen::Vector3d takeoff_point_;
    Eigen::Vector3d prepare_point_;

    Eigen::Vector3d velocity_cmd_ = Eigen::Vector3d::Zero();
    Eigen::Vector2d velocity_2d_cmd_ = Eigen::Vector2d::Zero();

    std::mutex log_mutex_;
};

#endif // CBF_ROS2_TASK_HPP
