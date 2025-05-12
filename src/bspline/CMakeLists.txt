cmake_minimum_required(VERSION 3.8)
project(bspline)

# Enable extra compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Find required ROS 2 packages ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# === Define the executable (ROS 2 node) ===
add_executable(bspline_node src/bspline_node.cpp)

# === Link dependencies to the executable ===
ament_target_dependencies(bspline_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

# === Install the executable so ROS 2 can run it ===
install(TARGETS
  bspline_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Enable linting and testing tools (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Finalize the package ===
ament_package()
