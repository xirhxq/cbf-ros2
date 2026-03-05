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
    // Use the same formula as quaternionToEuler for consistency with reference code
    return std::atan2(2.0 * (z * w + x * y), -1.0 + 2.0 * (w * w + x * x));
}

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param w, x, y, z Quaternion components (w, x, y, z order)
 * @param roll, pitch, yaw Output Euler angles in radians
 */
inline void quaternionToEuler(double w, double x, double y, double z,
                              double& roll, double& pitch, double& yaw) {
    // Reference: MyMathFun::quaternion_2_euler
    // quat[0]=w, quat[1]=x, quat[2]=y, quat[3]=z
    roll  = std::atan2(2.0 * (z * y + w * x), 1.0 - 2.0 * (x * x + y * y));
    pitch = std::asin(2.0 * (y * w - z * x));
    yaw   = std::atan2(2.0 * (z * w + x * y), -1.0 + 2.0 * (w * w + x * x));
}

/**
 * @brief Compute rotation matrix from Euler angles (ENU to Body frame)
 * @param phi, theta, psi Euler angles (roll, pitch, yaw) in radians
 * @param R Output 3x3 rotation matrix (row-major)
 */
inline void eulerToDcm(double phi, double theta, double psi, double R[3][3]) {
    // Reference: MyMathFun::Euler_2_Dcm
    double sinPhi = std::sin(phi), cosPhi = std::cos(phi);
    double sinThe = std::sin(theta), cosThe = std::cos(theta);
    double sinPsi = std::sin(psi), cosPsi = std::cos(psi);

    R[0][0] = cosThe * cosPsi;
    R[0][1] = cosThe * sinPsi;
    R[0][2] = -sinThe;

    R[1][0] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
    R[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
    R[1][2] = sinPhi * cosThe;

    R[2][0] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;
    R[2][1] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;
    R[2][2] = cosPhi * cosThe;
}

/**
 * @brief Transform velocity from earth frame to body frame
 * @param vx, vy, vz Velocity in earth frame
 * @param R Rotation matrix from eulerToDcm
 * @return Velocity in body frame (x-forward, y-left, z-up)
 */
inline void earthToBody(double& vx, double& vy, double& vz, const double R[3][3]) {
    double xx = vx, yy = vy, zz = vz;
    vx = R[0][0] * xx + R[0][1] * yy + R[0][2] * zz;
    vy = R[1][0] * xx + R[1][1] * yy + R[1][2] * zz;
    vz = R[2][0] * xx + R[2][1] * yy + R[2][2] * zz;
}

#endif // CBF_ROS2_UTILS_H