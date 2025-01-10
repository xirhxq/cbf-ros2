#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
                last_imu_ = *msg;
            });

        nav_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "uav_" + id_ + "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                last_odometry_ = *msg;
            });
    }

    void spin() {
        rclcpp::spin(shared_from_this());
    }

    void publish_velocity(const geometry_msgs::msg::Twist &cmd) {
        vel_cmd_pub_->publish(cmd);
    }

    const geometry_msgs::msg::Pose &get_last_pose() const {
        return last_pose_;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_sub_;

    std::string id_;

    sensor_msgs::msg::Imu last_imu_;
    geometry_msgs::msg::Pose last_pose_;
    nav_msgs::msg::Odometry last_odometry_;
};

class Task {
public:
    Task(const std::string &id)
        : id_(id), uav_comm_(std::make_shared<UAVCommNode>(id)), stop_flag_(false) {
        takeoff_point_.position.x = -1500.0;
        takeoff_point_.position.y = 0.0;
        takeoff_point_.position.z = 20.0;

        prepare_point_.position.x = -1400.0;
        prepare_point_.position.y = 0.0;
        prepare_point_.position.z = 20.0;

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
        const auto &current_pose = uav_comm_->get_last_pose();

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
                // 执行特定任务逻辑
                break;
            case State::BACK:
                control_to_point(current_pose, takeoff_point_);
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::LAND);
                }
                break;
            case State::LAND:
                control_to_height(current_pose, 0.0);
                break;
        }
    }

    void control_to_point(const geometry_msgs::msg::Pose &current_pose,
                          const geometry_msgs::msg::Pose &target_pose) {
        geometry_msgs::msg::Twist cmd;
        double dx = target_pose.position.x - current_pose.position.x;
        double dy = target_pose.position.y - current_pose.position.y;
        double dz = target_pose.position.z - current_pose.position.z;

        double max_speed = 1.0;

        cmd.linear.x = std::copysign(std::min(std::abs(dx), max_speed), dx);
        cmd.linear.y = std::copysign(std::min(std::abs(dy), max_speed), dy);
        cmd.linear.z = std::copysign(std::min(std::abs(dz), max_speed), dz);

        uav_comm_->publish_velocity(cmd);
    }

    void control_to_height(const geometry_msgs::msg::Pose &current_pose, double target_height) {
        geometry_msgs::msg::Twist cmd;
        double dz = target_height - current_pose.position.z;
        double max_speed = 1.0;

        cmd.linear.z = std::copysign(std::min(std::abs(dz), max_speed), dz);
        uav_comm_->publish_velocity(cmd);
    }

    bool is_at_point(const geometry_msgs::msg::Pose &current_pose,
                     const geometry_msgs::msg::Pose &target_pose,
                     double tolerance = 0.5) {
        double dx = target_pose.position.x - current_pose.position.x;
        double dy = target_pose.position.y - current_pose.position.y;
        double dz = target_pose.position.z - current_pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) < tolerance;
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

    geometry_msgs::msg::Pose takeoff_point_;
    geometry_msgs::msg::Pose prepare_point_;

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
    task.run();

    rclcpp::shutdown();
    return 0;
}
