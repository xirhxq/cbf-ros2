cmake_minimum_required(VERSION 3.8)
project(cbf-ros2)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

add_subdirectory(cbf)

install(TARGETS Swarm
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY cbf/config
  DESTINATION share/${PROJECT_NAME}
)

set(bridges pose_bridge)

foreach(BRIDGE ${bridges})
  add_executable(${BRIDGE} src/${BRIDGE}.cpp)
  ament_target_dependencies(${BRIDGE} rclcpp ignition-transport11 ignition-msgs8 nav_msgs)
  install(TARGETS ${BRIDGE}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

set(tasks suav cbf_pure)

foreach(TASK ${tasks})
  add_executable(${TASK} src/${TASK}.cpp)
  ament_target_dependencies(${TASK} rclcpp ros_ign_interfaces sensor_msgs nav_msgs std_msgs geometry_msgs)
  target_link_libraries(${TASK} cbf_deps)
  install(TARGETS ${TASK}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach(TASK ${tasks})


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

