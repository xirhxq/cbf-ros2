cmake_minimum_required(VERSION 3.8)
project(cbf-ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_subdirectory(cbf)

install(TARGETS Swarm
  DESTINATION lib/${PROJECT_NAME}
)

set(bridges pose_bridge)

foreach(BRIDGE ${bridges})
  add_executable(${BRIDGE} src/${BRIDGE}.cpp)
  ament_target_dependencies(${BRIDGE} rclcpp ignition-transport11 ignition-msgs8 nav_msgs)
  install(TARGETS ${BRIDGE}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

