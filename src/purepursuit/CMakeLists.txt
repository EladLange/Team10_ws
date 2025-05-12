cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)

ament_target_dependencies(pure_pursuit_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
