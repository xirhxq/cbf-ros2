/**
 * @file test_cbf_integration.cpp
 * @brief 独立测试程序，验证 CBF 算法集成无需 ROS2 仿真环境
 *
 * 编译后可直接运行：
 *   ros2 run cbf-ros2 test_cbf_integration
 *
 * 或者在 Docker 外单独编译：
 *   g++ -std=c++17 -I../cbf/include -I../cbf/external test_cbf_integration.cpp -o test_cbf
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <filesystem>
#include <iomanip>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cbf-core.h"
#include "cbf-ros2/Utils.h"

// Get initial positions from config
std::vector<Eigen::Vector3d> getInitialPositions(const json& config) {
    std::vector<Eigen::Vector3d> positions;
    if (config["initial"]["position"]["method"] == "specified") {
        for (const auto& pos : config["initial"]["position"]["positions"]) {
            positions.emplace_back(pos[0].get<double>(), pos[1].get<double>(), 50.0);
        }
    }
    return positions;
}

// SwarmController - Same implementation as in suav.cpp
class SwarmController {
public:
    SwarmController(const json& config)
        : config_(config), swarm_initialized_(false) {}

    void initialize() {
        // Create Swarm instance
        swarm_ = std::make_unique<Swarm>(config_);

        // Initialize CBF for each robot (distributed mode)
        for (auto &robot : swarm_->robots) {
            robot->presetCBF();
        }

        // Initialize data exchange
        swarm_->exchangeData();
        swarm_initialized_ = true;

        std::cout << GREEN << "[SwarmController] Initialized with "
                  << swarm_->robots.size() << " robots" << RESET << std::endl;
    }

    void step(const std::vector<Eigen::Vector3d>& uav_positions, double dt) {
        if (!swarm_initialized_) return;

        // 1. Sync positions
        for (size_t i = 0; i < swarm_->robots.size() && i < uav_positions.size(); ++i) {
            Point pos2d(uav_positions[i].x(), uav_positions[i].y());
            swarm_->robots[i]->model->setPosition2D(pos2d);
        }

        // 2. Data exchange
        swarm_->exchangeData();

        // 3. Update grid world and CBF
        for (auto &robot : swarm_->robots) {
            robot->updateGridWorld();
            robot->postsetCBF();
        }
        swarm_->updateGridWorld();  // Update Swarm-level grid world (for logging)

        // 4. Distributed optimization
        for (auto &robot : swarm_->robots) {
            robot->optimise();
        }

        // 5. Update runtime
        for (auto &robot : swarm_->robots) {
            robot->runtime += dt;
        }
    }

    Eigen::Vector2d getControl(int robot_index) {
        if (!swarm_initialized_ || robot_index >= static_cast<int>(swarm_->robots.size())) {
            return Eigen::Vector2d::Zero();
        }

        auto control = swarm_->robots[robot_index]->model->getControlInput();
        return Eigen::Vector2d(control(0), control(1));
    }

    // === Logging interface ===
    void initLog() {
        if (swarm_initialized_) {
            swarm_->initLog();
            swarm_->logParams();
            std::cout << GREEN << "[SwarmController] Log initialized, data will be saved to: "
                      << swarm_->filename << RESET << std::endl;
        }
    }

    void logStep() {
        if (swarm_initialized_) {
            swarm_->logOnce();
        }
    }

    void endLog() {
        if (swarm_initialized_) {
            swarm_->endLog();
            std::cout << GREEN << "[SwarmController] Log saved to: " << swarm_->filename << RESET << std::endl;
        }
    }

    // === External velocity injection interface ===
    void injectVelocities(const std::vector<std::pair<double, double>>& velocities) {
        if (swarm_initialized_) {
            swarm_->injectExternalVelocities(velocities);
        }
    }

    bool isInitialized() const { return swarm_initialized_; }

private:
    json config_;
    std::unique_ptr<Swarm> swarm_;
    bool swarm_initialized_;
};

int main(int argc, char **argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  CBF Integration Test (No Simulation)" << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. 加载配置文件
    std::string config_path;

    // 优先使用命令行参数
    if (argc > 1) {
        config_path = argv[1];
    } else {
        // 尝试从 ROS2 包安装路径获取
        try {
            std::string pkg_name = "cbf-ros2";
            std::string share_path = ament_index_cpp::get_package_share_directory(pkg_name);
            config_path = share_path + "/config/config.json";
        } catch (const std::exception& e) {
            // 如果 ament 索引失败，尝试相对路径（用于开发时直接运行）
            config_path = "../config/config.json";
        }
    }

    std::cout << "\n[Step 1] Loading config from: " << config_path << std::endl;

    // 检查文件是否存在
    if (!std::filesystem::exists(config_path)) {
        std::cerr << RED << "Config file not found: " << config_path << RESET << std::endl;
        std::cerr << "Usage: ros2 run cbf-ros2 test_cbf_integration [config_path]" << std::endl;
        return 1;
    }

    json cbf_config;
    try {
        cbf_config = json::parse(std::ifstream(config_path));
    } catch (const std::exception& e) {
        std::cerr << RED << "Error loading config: " << e.what() << RESET << std::endl;
        return 1;
    }

    // 2. Set output path (save to cbf submodule's data folder)
    // When running from cbf_ws, cbf submodule is at src/cbf-ros2/cbf
    std::string cbf_data_path = std::filesystem::current_path().string() + "/src/cbf-ros2/cbf/data";
    cbf_config["output_path"] = cbf_data_path;

    // 3. Display config info
    int num_robots = cbf_config["num"].get<int>();
    std::cout << "  - Number of robots: " << num_robots << std::endl;
    std::cout << "  - Execution mode: " << cbf_config["execute"]["execution-mode"] << std::endl;
    std::cout << "  - Position covariance: "
              << (cbf_config["position_covariance"]["enable"].get<bool>() ? "ON" : "OFF") << std::endl;

    // 3. Get initial positions
    std::cout << "\n[Step 2] Getting initial positions from config..." << std::endl;
    auto uav_positions = getInitialPositions(cbf_config);

    for (size_t i = 0; i < uav_positions.size(); ++i) {
        std::cout << "  - UAV " << (i + 1) << ": ("
                  << uav_positions[i].x() << ", " << uav_positions[i].y() << ", " << uav_positions[i].z() << ")" << std::endl;
    }

    // 4. Initialize SwarmController
    std::cout << "\n[Step 3] Initializing SwarmController..." << std::endl;
    SwarmController swarm_ctrl(cbf_config);
    swarm_ctrl.initialize();

    if (!swarm_ctrl.isInitialized()) {
        std::cerr << RED << "Failed to initialize SwarmController!" << RESET << std::endl;
        return 1;
    }

    // 5. Initialize log
    std::cout << "\n[Step 4] Initializing data log..." << std::endl;
    std::cout << "  - Output path: " << cbf_config["output_path"].get<std::string>() << std::endl;
    swarm_ctrl.initLog();

    // 6. Read time step and total time from config
    double dt = cbf_config["execute"]["time-step"].get<double>();
    double total_time = cbf_config["execute"]["time-total"].get<double>();
    int num_steps = static_cast<int>(total_time / dt);

    std::cout << "  - Time step (dt): " << dt << "s" << std::endl;
    std::cout << "  - Total time: " << total_time << "s" << std::endl;
    std::cout << "  - Number of steps: " << num_steps << std::endl;

    // 7. Run CBF optimization
    std::cout << "\n[Step 5] Running CBF optimization steps..." << std::endl;

    for (int step = 0; step < num_steps; ++step) {
        // Display progress
        if (step % 10 == 0 || step == num_steps - 1) {
            std::cout << "\r  Progress: " << (step + 1) << "/" << num_steps
                      << " (" << std::fixed << std::setprecision(1)
                      << (100.0 * (step + 1) / num_steps) << "%)"
                      << std::flush;
        }

        // Collect current velocities (simulating external velocity injection)
        std::vector<std::pair<double, double>> velocities;
        for (size_t i = 0; i < uav_positions.size(); ++i) {
            Eigen::Vector2d ctrl = swarm_ctrl.getControl(i);
            velocities.emplace_back(ctrl.x(), ctrl.y());
        }
        swarm_ctrl.injectVelocities(velocities);

        // Run CBF optimization
        swarm_ctrl.step(uav_positions, dt);

        // Log data
        swarm_ctrl.logStep();

        // Simple position update simulation
        for (size_t i = 0; i < uav_positions.size(); ++i) {
            Eigen::Vector2d ctrl = swarm_ctrl.getControl(i);
            uav_positions[i].x() += ctrl.x() * dt;
            uav_positions[i].y() += ctrl.y() * dt;
        }
    }
    std::cout << std::endl;

    // 8. Save log
    std::cout << "\n[Step 6] Saving data log..." << std::endl;
    swarm_ctrl.endLog();

    // 9. Final positions
    std::cout << "\n[Step 7] Final positions after " << num_steps << " steps:" << std::endl;
    for (size_t i = 0; i < uav_positions.size(); ++i) {
        std::cout << "  UAV " << (i + 1) << ": ("
                  << std::fixed << std::setprecision(2)
                  << uav_positions[i].x() << ", "
                  << uav_positions[i].y() << ", "
                  << uav_positions[i].z() << ")" << std::endl;
    }

    std::cout << "\n" << GREEN << "========================================" << std::endl;
    std::cout << "  CBF Integration Test PASSED!" << std::endl;
    std::cout << "========================================" << RESET << std::endl;

    return 0;
}
