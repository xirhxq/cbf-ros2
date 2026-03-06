# CBF-ROS2: Control Barrier Functions for Multi-UAV Search

A ROS2 package implementing **Control Barrier Functions (CBF)** for safe multi-UAV search missions in the MBZIRC simulation environment.

## Overview

This project wraps the [cbf](cbf/) core library into a ROS2 ecosystem, enabling CBF-based control for multiple UAVs operating in the MBZIRC Ignition Gazebo simulation. CBFs provide formal safety guarantees by encoding constraints directly into the control synthesis process.

### Key Features

- **Multi-UAV Support**: Coordinate up to 14+ UAVs simultaneously
- **Safety-Critical Control**: CBF-based velocity commands with collision avoidance
- **State Machine Architecture**: Robust mission execution (INIT → TAKEOFF → PREPARE → PERFORM → BACK → LAND)
- **MBZIRC Integration**: Full compatibility with MBZIRC simulation environment
- **Flexible Solver Backend**: Support for HiGHS (open-source) and Gurobi (commercial) optimizers
- **Centralized Configuration**: All control parameters in `Config.hpp` with unit-suffixed names

## Quick Start

### Prerequisites

- Docker (with GPU support)
- X11 display server

### Build

```bash
# 1. Build Docker image (first time only)
cd docker && ./build_docker.sh

# 2. Enter Docker environment
./docker/run_docker.sh

# 3. Build the workspace (inside Docker)
colcon build
```

### Run Options

#### Option 1: Pure CBF Algorithm (No Simulation)

Test CBF algorithms without MBZIRC simulation:

```bash
# Inside Docker container
. install/setup.bash
ros2 run cbf-ros2 cbf_pure
```

Runs a swarm simulation using configuration from `cbf/config/config.json`. See [cbf/README.md](cbf/README.md) for CBF algorithm details.

#### Option 2: Full MBZIRC Simulation

Launch the complete simulation with Ignition Gazebo:

```bash
# Inside Docker container
tmuxinator start . -p src/cbf-ros2/tmuxinator.cbf-ros2.yml
```

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Container                          │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                   cbf_ws Workspace                    │    │
│  │  ┌─────────────┐    ┌─────────────────────────────┐  │    │
│  │  │   cbf-ros2  │    │      MBZIRC Packages        │  │    │
│  │  │  (this pkg) │◄──►│  mbzirc_ros, mbzirc_ign    │  │    │
│  │  └─────────────┘    └─────────────────────────────┘  │    │
│  │         │                        │                    │    │
│  │         ▼                        ▼                    │    │
│  │  ┌─────────────────────────────────────────────┐     │    │
│  │  │           Ignition Gazebo (coast.sdf)        │     │    │
│  │  │              UAV x 14 Simulation             │     │    │
│  │  └─────────────────────────────────────────────┘     │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### UAV State Machine

```
┌──────┐   spawn    ┌──────────┐   20m alt  ┌─────────┐   ready   ┌─────────┐
│ INIT │──────────►│ TAKEOFF  │──────────►│ PREPARE │──────────►│ PERFORM │
└──────┘           └──────────┘           └─────────┘           └─────────┘
                                                                  │
                                                                  │ mission
                                                                  │ complete
                                                                  ▼
┌──────┐   landed    ┌──────────┐   descend  ┌─────────┐   return  ┌─────┐
│ LAND │◄───────────┤  BACK    │◄──────────┤         │◄─────────┤     │
└──────┘            └──────────┘            └─────────┘           └─────┘
```

### ROS2 Node Structure

Each UAV runs an independent `UAVCommNode`:

```
UAVCommNode (uav_comm_{id})
│
├── Publishers
│   └── ~/cmd_vel (geometry_msgs/Twist) ──► Velocity commands
│
├── Subscriptions
│   ├── ~/imu/data (sensor_msgs/Imu) ◄── IMU readings
│   └── ~/pose/groundtruth (geometry_msgs/PoseStamped) ◄── Position feedback
│
└── Control Loop (100 Hz)
    └── State machine updates → CBF optimization → Velocity publish
```

## Project Structure

```
cbf-ros2/
├── cbf/                        # Core CBF library (git submodule)
│   ├── src/                   # Optimization algorithms
│   ├── include/               # C++ headers
│   ├── external/Eigen/        # Linear algebra
│   └── config/                # Mission parameters
├── src/                        # ROS2 nodes
│   ├── suav.cpp               # Multi-UAV controller
│   ├── cbf_pure.cpp           # Standalone CBF task
│   └── pose_bridge.cpp        # Ignition-ROS bridge
├── include/cbf-ros2/           # Project headers
│   ├── Config.hpp             # Centralized configuration constants
│   ├── Task.hpp               # UAV state machine
│   ├── UAVCommNode.hpp        # ROS2 communication node
│   ├── SwarmController.hpp    # CBF Swarm wrapper
│   └── Utils.h                # Utility functions
├── scripts/                    # Utility scripts
│   ├── patch_uav_speed.sh     # Patch velocity/acceleration limits
│   ├── patch_geofence_height.sh # Patch altitude limits
│   └── patch_uav_thrust.sh    # Patch motor thrust constant
├── launch/                     # Launch configurations
│   ├── spawn.launch.py        # Multi-UAV spawner
│   └── groundtruth_pose.launch.py
├── docker/                     # Container setup
│   ├── Dockerfile             # MBZIRC-based image
│   ├── build_docker.sh        # Image builder
│   └── run_docker.sh          # Container launcher
└── tmuxinator.cbf-ros2.yml    # Session orchestration
```

## Configuration

### Centralized Configuration (Config.hpp)

All non-CBF algorithm parameters are centralized in `include/cbf-ros2/Config.hpp`:

```cpp
namespace cbf_ros2::config {

namespace altitude {
    constexpr double INITIAL_HEIGHT_M = 50.0;    // Initial position height
    constexpr double SEARCH_HEIGHT_M = 200.0;    // Search mission height
    constexpr double TAKEOFF_HEIGHT_M = 20.0;    // Height above spawn for takeoff
}

namespace velocity {
    constexpr double MAX_HORIZONTAL_EARTH_MPS = 15.0;  // Earth frame horizontal limit
    constexpr double MAX_VERTICAL_EARTH_MPS = 5.0;     // Earth frame vertical limit
    constexpr double MAX_HORIZONTAL_BODY_MPS = 15.0;   // Body frame horizontal limit
    constexpr double MAX_VERTICAL_BODY_MPS = 5.0;      // Body frame vertical limit
    constexpr double MAX_SPEED_MPS = 10.0;             // Control point max speed
    constexpr double MAX_YAW_RATE_RADPS = 1.0;         // Max yaw angular rate
}

namespace gains {
    constexpr double KP_POSITION = 0.4;   // Position control gain
    constexpr double KP_ALTITUDE = 1.0;   // Altitude control gain
    constexpr double KP_YAW = 1.0;        // Yaw control gain
}

namespace tolerance {
    constexpr double POSITION_M = 0.5;    // Position reached tolerance
    constexpr double YAW_RAD = 0.1;       // Yaw reached tolerance
}

}
```

### UAV Parameters (MBZIRC Simulation)

Edit `src/suav.cpp` to modify UAV settings for MBZIRC simulation:

```cpp
nlohmann::json settings = {
    {"id", "1"},
    {"prepare_point", {-1450.0, 30.0, 50.0}}  // x, y, z (meters)
};
```

### Number of UAVs

Modify `launch/spawn.launch.py`:

```python
DeclareLaunchArgument("numbers", default_value=TextSubstitution(text="6"))
```

## Patch Scripts

MBZIRC simulation parameters can be modified using patch scripts in `scripts/`:

### patch_uav_speed.sh

Modifies UAV velocity and acceleration limits:

```bash
./scripts/patch_uav_speed.sh [speed_mps] [horizontal_accel_mpss] [vertical_accel_mpss]
# Defaults: 25 10 4
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed_mps` | 25 | Maximum linear velocity (m/s) |
| `horizontal_accel_mpss` | 10 | Maximum horizontal acceleration (m/s²) |
| `vertical_accel_mpss` | 4 | Maximum vertical acceleration (m/s²) |

### patch_geofence_height.sh

Modifies geofence altitude limits:

```bash
./scripts/patch_geofence_height.sh [max_height_m] [min_height_m]
# Defaults: 200 -100
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_height_m` | 200 | Maximum flight altitude (m) |
| `min_height_m` | -100 | Minimum flight altitude (m) |

### patch_uav_thrust.sh

Modifies UAV motor thrust constant:

```bash
./scripts/patch_uav_thrust.sh [multiplier]
# Default: 1.5
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `multiplier` | 1.5 | Motor constant multiplier (1.5 = 50% more thrust) |

**Note:** Thrust = motorConstant × motor_speed²

## Development

### Executables

| Executable | Description |
|------------|-------------|
| `cbf_pure` | Standalone CBF algorithm (no simulation) |
| `suav` | Multi-UAV controller for MBZIRC |
| `pose_bridge` | Ignition-ROS pose bridge |
| `Swarm` | Core swarm simulation (from cbf submodule) |

### Building

```bash
# Standard build
colcon build

# Release mode
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clean rebuild
rm -rf build install log && colcon build
```

### Debugging

```bash
# Monitor UAV pose
ros2 topic echo /uav_1/pose/groundtruth

# Check velocity commands
ros2 topic echo /uav_1/cmd_vel

# List all topics
ros2 topic list
```

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| ROS2 | Humble | Robot middleware |
| Ignition Gazebo | Fortress | Simulation |
| MBZIRC | Latest | Competition framework |
| Eigen3 | 3.x | Linear algebra |
| HiGHS | Latest | QP solver (default) |
| Gurobi | 12.x | QP solver (optional) |

## References

- **CBF Theory**: [Control Barrier Functions: Theory and Applications](https://arxiv.org/abs/1512.07873)
- **MBZIRC**: [Maritime RobotX Challenge](https://github.com/osrf/mbzirc)
- **ROS2**: [Documentation](https://docs.ros.org/en/humble/)

## License

Apache License 2.0

## Author

**xirhxq** - [xirhxq@gmail.com](mailto:xirhxq@gmail.com)
