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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cbf-core.h"
#include "cbf-ros2/Utils.h"

// 模拟 UAV 位置（与 suav.cpp 中相同数量的 UAV）
const int NUM_UAVS = 6;

// 模拟初始位置（与 spawn.launch.py 中一致）
std::vector<Eigen::Vector3d> getInitialPositions() {
    return {
        {-1490.0, 0.0, 50.0},  // UAV 1
        {-1492.0, 0.0, 50.0},  // UAV 2
        {-1494.0, 0.0, 50.0},  // UAV 3
        {-1496.0, 0.0, 50.0},  // UAV 4
        {-1498.0, 0.0, 50.0},  // UAV 5
        {-1500.0, 0.0, 50.0},  // UAV 6
    };
}

// SwarmController - 与 suav.cpp 中相同的实现
class SwarmController {
public:
    SwarmController(const json& config)
        : config_(config), swarm_initialized_(false) {}

    void initialize() {
        // 创建 Swarm 实例
        swarm_ = std::make_unique<Swarm>(config_);

        // 为每个机器人初始化 CBF（分布式模式）
        for (auto &robot : swarm_->robots) {
            robot->presetCBF();
        }

        // 初始化数据交换
        swarm_->exchangeData();
        swarm_initialized_ = true;

        std::cout << GREEN << "[SwarmController] Initialized with "
                  << swarm_->robots.size() << " robots" << RESET << std::endl;
    }

    void step(const std::vector<Eigen::Vector3d>& uav_positions, double dt) {
        if (!swarm_initialized_) return;

        // 1. 同步位置
        for (size_t i = 0; i < swarm_->robots.size() && i < uav_positions.size(); ++i) {
            Point pos2d(uav_positions[i].x(), uav_positions[i].y());
            swarm_->robots[i]->model->setPosition2D(pos2d);
        }

        // 2. 数据交换
        swarm_->exchangeData();

        // 3. 更新网格世界和 CBF
        for (auto &robot : swarm_->robots) {
            robot->updateGridWorld();
            robot->postsetCBF();
        }

        // 4. 分布式优化
        for (auto &robot : swarm_->robots) {
            robot->optimise();
        }

        // 5. 更新运行时间
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

    // 2. 覆盖配置用于测试
    cbf_config["num"] = NUM_UAVS;
    cbf_config["execute"]["execution-mode"] = "distributed";

    // 关闭位置协方差计算（测试中 UAV 距离基站太远，无法获得足够锚点）
    cbf_config["position_covariance"]["enable"] = false;

    std::cout << "  - Number of robots: " << cbf_config["num"] << std::endl;
    std::cout << "  - Execution mode: " << cbf_config["execute"]["execution-mode"] << std::endl;
    std::cout << "  - Position covariance: "
              << (cbf_config["position_covariance"]["enable"].get<bool>() ? "ON" : "OFF") << std::endl;

    // 3. 设置初始位置
    std::cout << "\n[Step 2] Setting initial positions..." << std::endl;
    auto uav_positions = getInitialPositions();

    json positions_json = json::array();
    for (const auto &pos : uav_positions) {
        positions_json.push_back({pos.x(), pos.y()});
        std::cout << "  - UAV " << positions_json.size() << ": ("
                  << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
    }
    cbf_config["initial"]["position"]["method"] = "specified";
    cbf_config["initial"]["position"]["positions"] = positions_json;

    // 4. 初始化 SwarmController
    std::cout << "\n[Step 3] Initializing SwarmController..." << std::endl;
    SwarmController swarm_ctrl(cbf_config);
    swarm_ctrl.initialize();

    if (!swarm_ctrl.isInitialized()) {
        std::cerr << RED << "Failed to initialize SwarmController!" << RESET << std::endl;
        return 1;
    }

    // 5. 运行几步优化
    std::cout << "\n[Step 4] Running CBF optimization steps..." << std::endl;
    double dt = 0.1;  // 100ms
    int num_steps = 10;

    for (int step = 0; step < num_steps; ++step) {
        std::cout << "\n--- Step " << (step + 1) << " ---" << std::endl;

        // 运行 CBF 优化
        swarm_ctrl.step(uav_positions, dt);

        // 获取并显示控制输出
        std::cout << "Control outputs (vx, vy):" << std::endl;
        for (int i = 0; i < NUM_UAVS; ++i) {
            Eigen::Vector2d ctrl = swarm_ctrl.getControl(i);
            std::cout << "  UAV " << (i + 1) << ": ("
                      << ctrl.x() << ", " << ctrl.y() << ")" << std::endl;

            // 简单模拟位置更新（用于测试）
            uav_positions[i].x() += ctrl.x() * dt;
            uav_positions[i].y() += ctrl.y() * dt;
        }
    }

    // 6. 最终位置
    std::cout << "\n[Step 5] Final positions after " << num_steps << " steps:" << std::endl;
    for (int i = 0; i < NUM_UAVS; ++i) {
        std::cout << "  UAV " << (i + 1) << ": ("
                  << uav_positions[i].x() << ", "
                  << uav_positions[i].y() << ", "
                  << uav_positions[i].z() << ")" << std::endl;
    }

    std::cout << "\n" << GREEN << "========================================" << std::endl;
    std::cout << "  CBF Integration Test PASSED!" << std::endl;
    std::cout << "========================================" << RESET << std::endl;

    return 0;
}
