cmake_minimum_required(VERSION 3.5)
project(purepursuitonline_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(purepursuitonline_lib
  src/pure_pursuit.cpp
)

target_include_directories(purepursuitonline_lib PUBLIC
  include
)

add_executable(purepursuitonline_node
  src/purepursuitonline_node.cpp
)

target_link_libraries(purepursuitonline_node
  purepursuitonline_lib
)

ament_target_dependencies(purepursuitonline_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  purepursuitonline_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
