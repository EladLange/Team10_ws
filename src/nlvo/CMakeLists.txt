cmake_minimum_required(VERSION 3.8)
project(nlvo)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(common REQUIRED)


add_library(nlvo_lib
  src/nlvo.cpp
)

target_include_directories(nlvo_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(nlvo_lib 
rclcpp
geometry_msgs
visualization_msgs
tf2_ros
common
)

install(TARGETS nlvo_lib
  EXPORT export_nlvo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(export_nlvo)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()