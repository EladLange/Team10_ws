cmake_minimum_required(VERSION 3.8)
project(nao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleState.msg"
  "msg/ObstacleArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(include)

# Create the nao_planner library
add_library(nao_planner
  src/nao_planner.cpp
)

# Link dependencies to the library
ament_target_dependencies(nao_planner
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# Make sure the library can find the generated message headers
rosidl_target_interfaces(nao_planner
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Create the nao_node executable
add_executable(nao_node src/nao_node.cpp)

# Link dependencies to the executable
ament_target_dependencies(nao_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# Link the nao_planner library to the executable
target_link_libraries(nao_node
  nao_planner
)

# Make sure the executable can find the generated message headers
rosidl_target_interfaces(nao_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install targets
install(TARGETS
  nao_planner
  nao_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
