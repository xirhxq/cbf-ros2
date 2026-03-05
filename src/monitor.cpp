/**
 * @file monitor.cpp
 * @brief Monitor node for visualizing UAV positions in terminal
 *
 * Usage: ros2 run cbf-ros2 monitor [uav_num]
 * Example: ros2 run cbf-ros2 monitor 14
 */

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <iomanip>
#include <deque>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cbf-ros2/Utils.h"

using namespace std::chrono_literals;

class Monitor : public rclcpp::Node {
public:
    Monitor(int uav_num) : Node("Monitor"), clock_(0.0), last_clock_(0.0), uav_num_(uav_num) {
        // Limit to max supported
        if (uav_num_ > 50) uav_num_ = 50;
        if (uav_num_ < 1) uav_num_ = 1;

        // Initialize positions
        for (int i = 0; i <= uav_num_; i++) {
            uav_pos_[i] = {0.0, 0.0, -1.0, 0.0, 0.0, 0.0};  // z < 0 means not spawned
            uav_pos_prev_[i] = {0.0, 0.0, -1.0, 0.0, 0.0, 0.0};
        }

        RCLCPP_INFO(this->get_logger(), "Monitor initialized for %d UAVs", uav_num_);

        // Subscribe to clock
        clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,
            [this](const rosgraph_msgs::msg::Clock &msg) {
                this->clock_ = msg.clock.sec + msg.clock.nanosec / 1e9;
            }
        );

        // Subscribe to UAV groundtruth poses (from pose_bridge)
        for (int i = 1; i <= uav_num_; i++) {
            std::string id = std::to_string(i);
            std::string topic = "uav_" + id + "/pose/groundtruth";
            uav_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, 10,
                [i, this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    uav_pos_[i].x = msg->pose.position.x;
                    uav_pos_[i].y = msg->pose.position.y;
                    uav_pos_[i].z = msg->pose.position.z;
                }
            );
        }

        // Timer for display update
        timer_ = this->create_wall_timer(100ms, std::bind(&Monitor::timer_callback, this));
    }

private:
    struct Position {
        double x, y, z;
        double vx, vy, vz;  // Instant velocities computed from position derivative
        std::deque<double> vx_history, vy_history, vz_history;  // History for averaging
        double vx_avg, vy_avg, vz_avg;  // Averaged velocities (1 second window)
    };

    void timer_callback() {
        // Clear screen
        std::cout << "\033c" << std::flush;

        const int HISTORY_SIZE = 10;  // 10 samples at 100ms = 1 second window

        // Compute velocities from position derivative
        double dt = clock_ - last_clock_;
        if (dt > 0.001 && dt < 1.0) {  // Valid dt range
            for (int i = 1; i <= uav_num_; i++) {
                if (uav_pos_[i].z >= 0 && uav_pos_prev_[i].z >= 0) {
                    uav_pos_[i].vx = (uav_pos_[i].x - uav_pos_prev_[i].x) / dt;
                    uav_pos_[i].vy = (uav_pos_[i].y - uav_pos_prev_[i].y) / dt;
                    uav_pos_[i].vz = (uav_pos_[i].z - uav_pos_prev_[i].z) / dt;
                } else {
                    uav_pos_[i].vx = 0.0;
                    uav_pos_[i].vy = 0.0;
                    uav_pos_[i].vz = 0.0;
                }

                // Update velocity history
                uav_pos_[i].vx_history.push_back(uav_pos_[i].vx);
                uav_pos_[i].vy_history.push_back(uav_pos_[i].vy);
                uav_pos_[i].vz_history.push_back(uav_pos_[i].vz);

                if (uav_pos_[i].vx_history.size() > HISTORY_SIZE) {
                    uav_pos_[i].vx_history.pop_front();
                    uav_pos_[i].vy_history.pop_front();
                    uav_pos_[i].vz_history.pop_front();
                }

                // Compute averaged velocities
                if (!uav_pos_[i].vx_history.empty()) {
                    uav_pos_[i].vx_avg = std::accumulate(uav_pos_[i].vx_history.begin(),
                                                          uav_pos_[i].vx_history.end(), 0.0)
                                          / uav_pos_[i].vx_history.size();
                    uav_pos_[i].vy_avg = std::accumulate(uav_pos_[i].vy_history.begin(),
                                                          uav_pos_[i].vy_history.end(), 0.0)
                                          / uav_pos_[i].vy_history.size();
                    uav_pos_[i].vz_avg = std::accumulate(uav_pos_[i].vz_history.begin(),
                                                          uav_pos_[i].vz_history.end(), 0.0)
                                          / uav_pos_[i].vz_history.size();
                }
            }
        }

        // Store current positions for next iteration
        for (int i = 1; i <= uav_num_; i++) {
            uav_pos_prev_[i] = uav_pos_[i];
        }
        last_clock_ = clock_;

        // Print header
        std::cout << BOLDWHITE << "========================================" << RESET << std::endl;
        std::cout << BOLDWHITE << "       CBF Multi-UAV Monitor" << RESET << std::endl;
        std::cout << BOLDWHITE << "========================================" << RESET << std::endl;
        std::cout << "Sim Time: " << GREEN << std::fixed << std::setprecision(1) << clock_ << "s" << RESET << std::endl;
        std::cout << std::endl;

        // Print ASCII map
        output_map();

        // Print UAV details
        std::cout << std::endl;
        std::cout << BOLDWHITE << "UAV Positions and Velocities:" << RESET << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        for (int i = 1; i <= uav_num_; i++) {
            if (uav_pos_[i].z < 0) {
                // UAV not spawned yet
                std::cout << YELLOW << "#" << std::setw(2) << i << RESET << ": "
                          << RED << "NOT SPAWNED" << RESET << std::endl;
            } else {
                std::cout << GREEN << "#" << std::setw(2) << i << RESET << ": "
                          << "(" << std::setw(8) << std::fixed << std::setprecision(1) << uav_pos_[i].x << ", "
                          << std::setw(8) << uav_pos_[i].y << ", "
                          << std::setw(6) << uav_pos_[i].z << ")"
                          << CYAN << " vx=" << std::setw(5) << std::setprecision(1) << uav_pos_[i].vx_avg
                          << " vy=" << std::setw(5) << uav_pos_[i].vy_avg
                          << " vz=" << std::setw(5) << uav_pos_[i].vz_avg << RESET
                          << std::endl;
            }
        }
    }

    void output_map() {
        const int l = 25, w = 81;
        std::vector<std::vector<char>> mp(l, std::vector<char>(w, ' '));

        // Map coordinate bounds (based on MBZIRC coast world)
        const double max_x = 3162.28;
        const double max_y = 3162.28;

        // Convert world coordinates to map indices
        // x: left to right is positive -> column (w), no flip
        // y: bottom to top is positive -> row (l), flip needed
        auto to_map_row = [max_y, l](double y) -> int {
            int res = static_cast<int>((y + max_y / 2) / max_y * (l - 1));
            if (res < 0) res = 0;
            if (res >= l) res = l - 1;
            return l - 1 - res;  // Flip so y+ is up
        };

        auto to_map_col = [max_x, w](double y) -> int {
            int res = static_cast<int>((y + max_x / 2) / max_x * (w - 1));
            if (res < 0) res = 0;
            if (res >= w) res = w - 1;
            return w - 1 - res;  // Flip so y+ is left
        };

        // Add grid lines
        for (int i = 1; i < uav_num_; i++) {
            int col = static_cast<int>(1.0 * i / uav_num_ * w);
            for (int k = 0; k < l; k++) {
                mp[k][col] = ':';
            }
        }

        // Add UAV positions to map
        for (int i = 1; i <= uav_num_; i++) {
            if (uav_pos_[i].z < 0) continue;  // Skip unspawned UAVs

            int mx = to_map_row(uav_pos_[i].x);  // x -> row
            int my = to_map_col(uav_pos_[i].y);  // y -> column

            // Use hex digits for IDs > 9
            char c;
            if (i >= 36) {
                c = 'Z';
            } else if (i >= 10) {
                c = 'a' + i - 10;
            } else {
                c = '0' + i;
            }
            mp[mx][my] = c;
        }

        // Print map border
        std::cout << "+";
        for (int j = 0; j < w; j++) std::cout << "-";
        std::cout << "+" << std::endl;

        // Print map content
        for (int i = 0; i < l; i++) {
            std::cout << "|";
            for (int j = 0; j < w; j++) {
                char c = mp[i][j];
                if (c >= '0' && c <= '9') {
                    std::cout << BOLDGREEN << c << RESET;
                } else if (c >= 'a' && c <= 'z') {
                    std::cout << BOLDCYAN << c << RESET;
                } else if (c == ':') {
                    std::cout << BLACK << c << RESET;
                } else {
                    std::cout << c;
                }
            }
            std::cout << "|" << std::endl;
        }

        // Print map border
        std::cout << "+";
        for (int j = 0; j < w; j++) std::cout << "-";
        std::cout << "+" << std::endl;

        // Print legend
        std::cout << "Legend: " << GREEN << "0-9" << RESET << " = UAV 1-10, "
                  << CYAN << "a-z" << RESET << " = UAV 11-36" << std::endl;
    }

    // Members
    double clock_;
    double last_clock_;
    int uav_num_;
    Position uav_pos_[51];       // Support up to 50 UAVs
    Position uav_pos_prev_[51];  // Previous positions for velocity calculation

    // Subscriptions
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr uav_sub_[51];
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int uav_num = 6;  // default
    if (argc > 1) {
        try {
            uav_num = std::stoi(argv[1]);
        } catch (...) {
            std::cerr << "Invalid argument: " << argv[1] << ", using default 6" << std::endl;
            uav_num = 6;
        }
    }

    rclcpp::spin(std::make_shared<Monitor>(uav_num));
    rclcpp::shutdown();
    return 0;
}
