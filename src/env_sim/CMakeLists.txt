cmake_minimum_required(VERSION 3.8)
project(env_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

# Add executable and link libraries
add_executable(env_sim_node
  src/env_sim_node.cpp
  src/road_visualization.cpp
  src/road.cpp
  src/drone_controller.cpp
  src/car.cpp 
  src/velocity_visualization.cpp
  src/raceline_visualization.cpp
)

# Link required libraries
ament_target_dependencies(
  env_sim_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
)

# Install executable and other files
install(TARGETS env_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
