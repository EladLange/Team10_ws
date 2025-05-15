cmake_minimum_required(VERSION 3.8)
project(car_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(car_controller)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)  
find_package(ros_gz_bridge REQUIRED) 

include_directories(include)


add_executable(ego_controller src/ego_controller.cpp)
ament_target_dependencies(ego_controller rclcpp std_msgs geometry_msgs)

add_executable(ego_pose_pub src/ego_odom_pub.cpp)
ament_target_dependencies(ego_pose_pub rclcpp std_msgs geometry_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY meshes urdf launch RVIZ
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ego_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ego_pose_pub
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
