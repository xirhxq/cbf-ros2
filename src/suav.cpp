/**
 * @file suav.cpp
 * @brief Multi-UAV control with CBF integration for MBZIRC simulation
 *
 * This node controls multiple UAVs using Control Barrier Functions (CBF)
 * for cooperative search missions.
 */

#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>
#include <csignal>
#include <filesystem>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cbf-core.h"
#include "cbf-ros2/Utils.h"
#include "cbf-ros2/SwarmController.hpp"
#include "cbf-ros2/UAVCommNode.hpp"
#include "cbf-ros2/Task.hpp"

using namespace std::chrono_literals;

// === Global Variables ===

double g_sim_time = 0.0;
std::mutex g_time_mutex;

// === Main Function ===

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
    });

    std::cout << "========================================" << std::endl;
    std::cout << "  CBF Multi-UAV Control for MBZIRC" << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. Load config
    std::string config_path;
    try {
        std::string pkg_name = "cbf-ros2";
        std::string share_path = ament_index_cpp::get_package_share_directory(pkg_name);
        config_path = share_path + "/config/config.json";
    } catch (const std::exception& e) {
        config_path = "../config/config.json";
    }

    std::cout << "\n[Step 1] Loading config from: " << config_path << std::endl;

    if (!std::filesystem::exists(config_path)) {
        std::cerr << RED << "Config file not found: " << config_path << RESET << std::endl;
        return 1;
    }

    json cbf_config;
    try {
        cbf_config = json::parse(std::ifstream(config_path));
    } catch (const std::exception& e) {
        std::cerr << RED << "Error loading config: " << e.what() << RESET << std::endl;
        return 1;
    }

    // 2. Set output path for data logging
    std::string cbf_data_path = std::filesystem::current_path().string() + "/src/cbf-ros2/cbf/data";
    cbf_config["output_path"] = cbf_data_path;

    int num_robots = cbf_config["num"].get<int>();
    double target_yaw_deg = cbf_config["initial"]["yawDeg"].get<double>();
    double time_step = cbf_config["execute"]["time-step"].get<double>();

    std::cout << "  - Number of robots: " << num_robots << std::endl;
    std::cout << "  - Target yaw: " << target_yaw_deg << " deg" << std::endl;
    std::cout << "  - CBF time step: " << time_step << "s" << std::endl;

    // 3. Initialize SwarmController
    std::cout << "\n[Step 2] Initializing SwarmController..." << std::endl;
    SwarmController swarm_ctrl(cbf_config);
    swarm_ctrl.initialize();

    if (!swarm_ctrl.isInitialized()) {
        std::cerr << RED << "Failed to initialize SwarmController!" << RESET << std::endl;
        return 1;
    }

    // 4. Initialize data logging
    std::cout << "\n[Step 3] Initializing data log..." << std::endl;
    swarm_ctrl.initLog();

    // 5. Create clock subscriber node
    auto clock_node = rclcpp::Node::make_shared("cbf_clock_node");
    auto clock_sub = clock_node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10,
        [](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(g_time_mutex);
            g_sim_time = msg->clock.sec + msg->clock.nanosec / 1e9;
        });

    // 6. Create Tasks (UAV nodes) from config
    std::cout << "\n[Step 4] Creating UAV tasks..." << std::endl;

    // Get initial positions from config
    std::vector<Eigen::Vector3d> initial_positions;
    if (cbf_config["initial"]["position"]["method"] == "specified") {
        for (const auto& pos : cbf_config["initial"]["position"]["positions"]) {
            initial_positions.emplace_back(pos[0].get<double>(), pos[1].get<double>(), 50.0);
        }
    }

    // Create task settings
    nlohmann::json task_settings = json::array();
    for (size_t i = 0; i < static_cast<size_t>(num_robots); ++i) {
        double z = (i < initial_positions.size()) ? initial_positions[i].z() : 50.0;
        task_settings.push_back({
            {"id", std::to_string(i + 1)},
            {"prepare_point", {initial_positions[i].x(), initial_positions[i].y(), z}}
        });
    }

    std::vector<std::unique_ptr<Task>> tasks;
    for (auto& settings : task_settings) {
        tasks.emplace_back(std::make_unique<Task>(settings, target_yaw_deg));
        std::cout << "  - Created task for UAV #" << tasks.back()->getId() << std::endl;
    }

    // 7. Main control loop
    std::cout << "\n[Step 5] Starting main control loop..." << std::endl;
    std::cout << "  - Waiting for UAVs to spawn..." << std::endl;

    double last_cbf_time = 0.0;
    double perform_start_time = 0.0;
    double perform_duration = cbf_config["execute"]["time-total"].get<double>();

    while (rclcpp::ok()) {
        // Spin clock node
        rclcpp::spin_some(clock_node);

        // Get current simulation time
        double current_time;
        {
            std::lock_guard<std::mutex> lock(g_time_mutex);
            current_time = g_sim_time;
        }

        // Check if all UAVs are in PERFORM state
        bool all_in_perform = true;
        for (auto& task : tasks) {
            if (!task->isInPerform()) {
                all_in_perform = false;
                break;
            }
        }

        // Run CBF at fixed rate (based on simulation time)
        double dt = current_time - last_cbf_time;
        if (all_in_perform && dt >= time_step) {
            last_cbf_time = current_time;

            // Record start time when all UAVs enter PERFORM
            if (perform_start_time == 0.0) {
                perform_start_time = current_time;
                std::cout << GREEN << "\n*** All UAVs in PERFORM state, starting CBF control ***" << RESET << std::endl;
            }

            // Check if we should end PERFORM
            if (current_time - perform_start_time >= perform_duration) {
                std::cout << YELLOW << "\n*** Perform duration reached, ending mission ***" << RESET << std::endl;
                for (auto& task : tasks) {
                    task->endPerform();
                }
            }

            // Get all UAV positions
            std::vector<Eigen::Vector3d> positions(tasks.size());
            for (size_t i = 0; i < tasks.size(); ++i) {
                positions[i] = tasks[i]->getPosition();
                swarm_ctrl.updateYawFromSim(i, tasks[i]->getYaw());
            }

            // Collect velocities for injection
            std::vector<std::pair<double, double>> velocities;
            for (size_t i = 0; i < positions.size(); ++i) {
                Eigen::Vector3d ctrl = swarm_ctrl.getControl(i);
                velocities.emplace_back(ctrl.x(), ctrl.y());
            }
            swarm_ctrl.injectVelocities(velocities);

            // Run CBF optimization
            swarm_ctrl.step(positions, dt);

            // Log data
            swarm_ctrl.logStep();

            // Get control commands and send to UAVs
            for (size_t i = 0; i < tasks.size(); ++i) {
                Eigen::Vector3d ctrl = swarm_ctrl.getControl(i);
                tasks[i]->setCBFControl(Eigen::Vector2d(ctrl.x(), ctrl.y()), ctrl.z());
            }
        }

        // Run task state machines
        for (auto& task : tasks) {
            task->runOnce();
        }

        // Check if all UAVs are in LAND state - if so, exit
        bool all_in_land = true;
        for (auto& task : tasks) {
            if (!task->isInLand()) {
                all_in_land = false;
                break;
            }
        }
        if (all_in_land) {
            std::cout << GREEN << "\n*** All UAVs have landed, mission complete! ***" << RESET << std::endl;
            break;
        }

        std::this_thread::sleep_for(50ms);
    }

    // 8. Save log before exit
    std::cout << "\n[Step 6] Saving data log..." << std::endl;
    swarm_ctrl.endLog();

    rclcpp::shutdown();
    return 0;
}
