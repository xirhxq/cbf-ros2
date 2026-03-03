cmake_minimum_required(VERSION 3.8)
project(cbf-ros2)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

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

# 独立测试程序 - 不需要 ROS2 仿真环境
add_executable(test_cbf_integration src/test_cbf_integration.cpp)
ament_target_dependencies(test_cbf_integration ament_index_cpp)
set_target_properties(test_cbf_integration PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_link_libraries(test_cbf_integration cbf_deps)
install(TARGETS test_cbf_integration
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

