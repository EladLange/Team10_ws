cmake_minimum_required(VERSION 3.8)
project(vo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# Include directories for headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/vo)

# Add executable and link libraries
add_executable(vo_simulation_node
  src/vo_simulation_node.cpp
  src/road_visualization.cpp
  src/road.cpp
  src/drone_controller.cpp
  src/car.cpp  
  src/velocity_visualization.cpp
  src/vo_visualization.cpp
  src/velocity_obstacle.cpp
  src/raceline_visualization.cpp
)

add_executable(velocity_obstacle
  src/velocity_obstacle_main.cpp
  src/velocity_obstacle.cpp
)


# Link required libraries
ament_target_dependencies(
  vo_simulation_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
)

ament_target_dependencies(
  velocity_obstacle
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

# Install executable and other files
install(TARGETS vo_simulation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS velocity_obstacle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
