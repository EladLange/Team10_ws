cmake_minimum_required(VERSION 3.8)
project(pscav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(common REQUIRED)


add_library(pscav_lib
  src/pscav.cpp
)

target_include_directories(pscav_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(pscav_lib
rclcpp
geometry_msgs
visualization_msgs
tf2_ros
common
)

install(TARGETS pscav_lib
  EXPORT export_pscav
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(export_pscav)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()