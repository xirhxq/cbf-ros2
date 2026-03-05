/**
 * @file SwarmController.hpp
 * @brief Wrapper class for CBF Swarm optimization
 *
 * This class wraps the CBF library's Swarm functionality and * providing a clean interface for CBF integration.
 */

#ifndef CBF_ROS2_SWARM_CONTROLLER_HPP
#define CBF_ROS2_SWARM_CONTROLLER_HPP

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "cbf-core.h"

class SwarmController {
public:
    SwarmController(const json& config)
        : config_(config), swarm_initialized_(false) {}

    void initialize() {
        swarm_ = std::make_unique<Swarm>(config_);

        for (auto &robot : swarm_->robots) {
            robot->presetCBF();
        }

        swarm_->exchangeData();
        swarm_initialized_ = true;

        // Initialize yaw from config
        double initial_yaw_deg = config_["initial"]["yawDeg"].get<double>();
        double initial_yaw_rad = initial_yaw_deg * M_PI / 180.0;
        uav_yaws_.resize(swarm_->robots.size(), initial_yaw_rad);

        // Set initial yaw in models
        for (size_t i = 0; i < swarm_->robots.size(); ++i) {
            swarm_->robots[i]->model->setStateVariable("yawRad", uav_yaws_[i]);
        }

        std::cout << "\033[32m[SwarmController] Initialized with "
                  << swarm_->robots.size() << " robots\033[0m" << std::endl;
    }

    void step(const std::vector<Eigen::Vector3d>& uav_positions, double dt) {
        if (!swarm_initialized_) return;

        // 1. Sync positions and yaw
        for (size_t i = 0; i < swarm_->robots.size() && i < uav_positions.size(); ++i) {
            Point pos2d(uav_positions[i].x(), uav_positions[i].y());
            swarm_->robots[i]->model->setPosition2D(pos2d);
            swarm_->robots[i]->model->setStateVariable("yawRad", uav_yaws_[i]);
        }

        // 2. Data Exchange
        swarm_->exchangeData();

        // 3. Update grid World and CBF
        for (auto &robot : swarm_->robots) {
            robot->updateGridWorld();
            robot->postsetCBF();
        }
        swarm_->updateGridWorld();

        // 4. Distributed Optimization
        for (auto &robot : swarm_->robots) {
            robot->optimise();
        }

        // 5. Update Runtime
        for (auto &robot : swarm_->robots) {
            robot->runtime += dt;
        }
    }

    Eigen::Vector3d getControl(int robot_index) {
        if (!swarm_initialized_ || robot_index >= static_cast<int>(swarm_->robots.size())) {
            return Eigen::Vector3d::Zero();
        }

        auto control = swarm_->robots[robot_index]->model->getControlInput();
        return Eigen::Vector3d(control(0), control(1), control(2));
    }

    void initLog() {
        if (swarm_initialized_) {
            swarm_->initLog();
            swarm_->logParams();
            std::cout << "\033[32m[SwarmController] Log initialized, data will be saved to: "
                      << swarm_->filename << "\033[0m" << std::endl;
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
            std::cout << "\033[32m[SwarmController] Log saved to: " << swarm_->filename << "\033[0m" << std::endl;
        }
    }

    void injectVelocities(const std::vector<std::pair<double, double>>& velocities) {
        if (swarm_initialized_) {
            swarm_->injectExternalVelocities(velocities);
        }
    }

    void updateYawFromSim(int robot_index, double sim_yaw) {
        if (robot_index >= 0 && robot_index < static_cast<int>(uav_yaws_.size())) {
            uav_yaws_[robot_index] = sim_yaw;
        }
    }

    void resetRuntime() {
        if (swarm_initialized_) {
            for (auto& robot : swarm_->robots) {
                robot->runtime = 0.0;
            }
        }
    }

    void updateYawFromControl(int robot_index, double yaw_rate, double dt) {
        if (robot_index >= 0 && robot_index < static_cast<int>(uav_yaws_.size())) {
            uav_yaws_[robot_index] += yaw_rate * dt;
            uav_yaws_[robot_index] = normalizeAngle(uav_yaws_[robot_index]);
        }
    }

    double getYaw(int robot_index) const {
        if (robot_index >= 0 && robot_index < static_cast<int>(uav_yaws_.size())) {
            return uav_yaws_[robot_index];
            }
        return 0.0;
    }

    bool isInitialized() const { return swarm_initialized_; }

private:
    json config_;
    std::unique_ptr<Swarm> swarm_;
    bool swarm_initialized_;
    std::vector<double> uav_yaws_;
};

#endif // CBF_ROS2_SWARM_CONTROLLER_HPP
