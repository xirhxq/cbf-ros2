/**
 * @file Config.hpp
 * @brief Centralized configuration constants for cbf-ros2
 */

#ifndef CBF_ROS2_CONFIG_HPP
#define CBF_ROS2_CONFIG_HPP

namespace cbf_ros2 {
namespace config {

namespace altitude {
    constexpr double INITIAL_HEIGHT_M = 50.0;
    constexpr double SEARCH_HEIGHT_M = 200.0;
    constexpr double TAKEOFF_HEIGHT_M = 20.0;
}

namespace velocity {
    constexpr double MAX_HORIZONTAL_EARTH_MPS = 25.0;
    constexpr double MAX_VERTICAL_EARTH_MPS = 15.0;
    constexpr double MAX_HORIZONTAL_BODY_MPS = 25.0;
    constexpr double MAX_VERTICAL_BODY_MPS = 15.0;
    constexpr double MAX_SPEED_MPS = 25.0;
    constexpr double MAX_YAW_RATE_RADPS = 1.0;
}

namespace gains {
    constexpr double KP_POSITION = 0.8;
    constexpr double KP_ALTITUDE = 1.0;
    constexpr double KP_YAW = 1.0;
}

namespace tolerance {
    constexpr double POSITION_M = 0.5;
    constexpr double YAW_RAD = 0.1;
}

} // namespace config
} // namespace cbf_ros2

#endif // CBF_ROS2_CONFIG_HPP
