cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
add_executable(car_visualizer_node src/car_visualizer_node.cpp)

ament_target_dependencies(pure_pursuit_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(car_visualizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
)


install(TARGETS
  pure_pursuit_node
  car_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

