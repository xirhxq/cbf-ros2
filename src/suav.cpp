#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>
#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ros_ign_interfaces/msg/dataframe.hpp"

using namespace std::chrono_literals;

class UAVCommNode : public rclcpp::Node {
public:
    UAVCommNode(const std::string &id)
        : Node("uav_comm_" + id), id_(id) {
        vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("uav_" + id_ + "/cmd_vel", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "uav_" + id_ + "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_imu_ = *msg;
            });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "uav_" + id_ + "/pose/groundtruth", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_pose_ = *msg;
            });
    }

    void spin() {
        rclcpp::spin(shared_from_this());
    }

    void publish_velocity(const Eigen::Vector3d &velocity) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = velocity.x();
        cmd.linear.y = velocity.y();
        cmd.linear.z = velocity.z();
        vel_cmd_pub_->publish(cmd);
    }

    Eigen::Vector3d get_last_pose() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return Eigen::Vector3d(
            last_pose_.pose.position.x,
            last_pose_.pose.position.y,
            last_pose_.pose.position.z);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    std::string id_;

    mutable std::mutex data_mutex_;
    sensor_msgs::msg::Imu last_imu_;
    geometry_msgs::msg::PoseStamped last_pose_;
};

class Task {
public:
    Task(const std::string &id)
        : id_(id), uav_comm_(std::make_shared<UAVCommNode>(id)), stop_flag_(false) {

        search_height_ = 20.0;

        spawn_point_ = Eigen::Vector3d(-1500.0, 0.0, 0.0);
        takeoff_point_ = Eigen::Vector3d(-1500.0, 0.0, search_height_);
        prepare_point_ = Eigen::Vector3d(-1400.0, 0.0, search_height_);

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

    void run() {
        while (!stop_flag_) {
            std::this_thread::sleep_for(100ms);
            update();
        }
    }

    void stop() {
        stop_flag_ = true;
    }

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

        switch (current_state_) {
            case State::INIT:
                if (current_pose.norm() > 1e-3) {
                    spawn_point_ = current_pose;
                    takeoff_point_ = spawn_point_ + Eigen::Vector3d(0.0, 0.0, 20.0);
                    transition_to(State::TAKEOFF);
                }
                break;
            case State::TAKEOFF:
                control_to_point(current_pose, takeoff_point_);
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::PREPARE);
                }
                break;
            case State::PREPARE:
                control_to_point(current_pose, prepare_point_);
                if (is_at_point(current_pose, prepare_point_)) {
                    transition_to(State::PERFORM);
                }
                break;
            case State::PERFORM:
                velocity2dControl(current_pose, velocity_2d_cmd_);
                break;
            case State::BACK:
                control_to_point(current_pose, takeoff_point_);
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::LAND);
                }
                break;
            case State::LAND:
                control_to_point(current_pose, spawn_point_);
                break;
        }

        uav_comm_->publish_velocity(velocity_cmd_);

        std::lock_guard<std::mutex> lock(log_mutex_);
        printf(
            "#%s | Position: %.2f, %.2f, %.2f | State: %s | Control: (%.2f, %.2f, %.2f)\n",
            id_.c_str(), current_pose.x(), current_pose.y(), current_pose.z(),
            state_to_string(current_state_).c_str(),
            velocity_cmd_.x(), velocity_cmd_.y(), velocity_cmd_.z()
        );
    }

    void control_to_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose) {
        Eigen::Vector3d delta = target_pose - current_pose;
        
        double kp = 0.2;
        double max_speed = 5.0;
        Eigen::Vector3d vel = kp * delta;

        if (vel.norm() > max_speed) {
            vel = vel.normalized() * max_speed;
        }
        
        velocity_cmd_ = vel;
    }

    void velocity2dControl(const Eigen::Vector3d &current_pose, const Eigen::Vector2d &velocity) {
        double kp = 0.2;
        velocity_cmd_ = Eigen::Vector3d(velocity.x(), velocity.y(), kp * (search_height_ - current_pose.z()));
    }

    void setVelocity2dCmd(const Eigen::Vector2d &velocity) {
        velocity_2d_cmd_ = velocity;
    }

    bool is_at_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose, double tolerance = 0.5) {
        return (current_pose - target_pose).norm() < tolerance;
    }

    void transition_to(State new_state) {
        current_state_ = new_state;
        std::lock_guard<std::mutex> lock(log_mutex_);
        std::cout << "Transitioned to state: " << state_to_string(current_state_) << std::endl;
    }

    std::string state_to_string(State state) {
        switch (state) {
            case State::INIT: return "INIT";
            case State::TAKEOFF: return "TAKEOFF";
            case State::PREPARE: return "PREPARE";
            case State::PERFORM: return "PERFORM";
            case State::BACK: return "BACK";
            case State::LAND: return "LAND";
            default: return "UNKNOWN";
        }
    }

    std::string id_;
    std::shared_ptr<UAVCommNode> uav_comm_;
    std::atomic<bool> stop_flag_;
    std::thread spin_thread_;
    State current_state_ = State::INIT;

    double search_height_;

    Eigen::Vector3d spawn_point_;
    Eigen::Vector3d takeoff_point_;
    Eigen::Vector3d prepare_point_;
    
    Eigen::Vector3d velocity_cmd_;
    Eigen::Vector2d velocity_2d_cmd_;

    std::mutex log_mutex_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide an ID as an argument.");
        return 1;
    }
    std::string id(argv[1]);

    Task task(id);

    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
    });

    task.run();

    return 0;
}