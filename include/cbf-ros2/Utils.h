#ifndef CBF_ROS2_UTILS_H
#define CBF_ROS2_UTILS_H

#include <string>
#include <cmath>

// === ANSI Color Codes ===

const std::string RESET = "\033[0m";
const std::string BLACK = "\033[30m";
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";
const std::string WHITE = "\033[37m";
const std::string BOLDBLACK = "\033[1m\033[30m";
const std::string BOLDRED = "\033[1m\033[31m";
const std::string BOLDGREEN = "\033[1m\033[32m";
const std::string BOLDYELLOW = "\033[1m\033[33m";
const std::string BOLDBLUE = "\033[1m\033[34m";
const std::string BOLDMAGENTA = "\033[1m\033[35m";
const std::string BOLDCYAN = "\033[1m\033[36m";
const std::string BOLDWHITE = "\033[1m\033[37m";

// === Utility Functions ===

/**
 * @brief Normalize angle to [-pi, pi]
 * @param angle Input angle in radians
 * @return Normalized angle in radians
 */
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

/**
 * @brief Extract yaw angle from quaternion
 * @param w, x, y, z Quaternion components
 * @return Yaw angle in radians
 */
inline double quaternionToYaw(double w, double x, double y, double z) {
    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

#endif // CBF_ROS2_UTILS_H