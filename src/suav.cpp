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

    void publish_velocity(const geometry_msgs::msg::Twist &cmd) {
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
        takeoff_point_ = Eigen::Vector3d(-1500.0, 0.0, 20.0);
        prepare_point_ = Eigen::Vector3d(-1400.0, 0.0, 20.0);

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
        TAKEOFF,
        PREPARE,
        PERFORM,
        BACK,
        LAND
    };

    void update() {
        Eigen::Vector3d current_pose = uav_comm_->get_last_pose();

        switch (current_state_) {
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
                // Add specific task logic here
                break;
            case State::BACK:
                control_to_point(current_pose, takeoff_point_);
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::LAND);
                }
                break;
            case State::LAND:
                control_to_height(current_pose.z(), 0.0);
                break;
        }

        // Log position, state, and control input
        std::lock_guard<std::mutex> lock(log_mutex_);
        std::cout << "Current Position: " << current_pose.transpose()
                  << " | State: " << state_to_string(current_state_)
                  << " | Control: (" << current_cmd_.linear.x << ", "
                  << current_cmd_.linear.y << ", " << current_cmd_.linear.z << ")" << std::endl;
    }

    void control_to_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose) {
        geometry_msgs::msg::Twist cmd;
        Eigen::Vector3d delta = target_pose - current_pose;

        double max_speed = 1.0;
        cmd.linear.x = std::copysign(std::min(std::abs(delta.x()), max_speed), delta.x());
        cmd.linear.y = std::copysign(std::min(std::abs(delta.y()), max_speed), delta.y());
        cmd.linear.z = std::copysign(std::min(std::abs(delta.z()), max_speed), delta.z());

        current_cmd_ = cmd;
        uav_comm_->publish_velocity(cmd);
    }

    void control_to_height(double current_z, double target_height) {
        geometry_msgs::msg::Twist cmd;
        double dz = target_height - current_z;
        double max_speed = 1.0;

        cmd.linear.z = std::copysign(std::min(std::abs(dz), max_speed), dz);
        current_cmd_ = cmd;
        uav_comm_->publish_velocity(cmd);
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
    State current_state_ = State::TAKEOFF;

    Eigen::Vector3d takeoff_point_;
    Eigen::Vector3d prepare_point_;
    geometry_msgs::msg::Twist current_cmd_;

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