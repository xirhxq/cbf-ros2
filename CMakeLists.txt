cmake_minimum_required(VERSION 3.8)
project(cbf-ros2)

find_package(ament_cmake REQUIRED)

add_subdirectory(cbf)

install(TARGETS Swarm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

