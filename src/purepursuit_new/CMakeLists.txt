# Minimum required CMake version
cmake_minimum_required(VERSION 3.8)

# Name of the project
project(purepursuit_new)

# Use C++17 standard
add_compile_options(-std=c++17)

# =====================================================
# Step 1: Find required ROS2 packages
# =====================================================
# These packages provide core ROS2 functionality and message definitions
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                         # Core ROS2 C++ API
find_package(geometry_msgs REQUIRED)                 # For Pose, Point, etc.
find_package(nav_msgs REQUIRED)                      # For Path messages
find_package(tf2 REQUIRED)                           # For transform calculations
find_package(tf2_ros REQUIRED)                       # For broadcasting transforms
find_package(tf2_geometry_msgs REQUIRED)             # For tf2 and geometry_msgs conversion
find_package(visualization_msgs REQUIRED)            # For RViz markers

# =====================================================
# Step 2: Build the vehicle models library
# =====================================================
# This library contains the base and bicycle model implementations
add_library(vehicle_models
  src/bicycle_model.cpp
)

# Include directories for the model library
target_include_directories(vehicle_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>   # During build
  $<INSTALL_INTERFACE:include>                             # During install
)

# Link ROS2 dependencies for the model library
ament_target_dependencies(vehicle_models
  rclcpp
)

# =====================================================
# Step 3: Build the main node executable
# =====================================================
# This builds the main pure_pursuit_node binary
add_executable(pure_pursuit_node
  src/pure_pursuit_node.cpp
)

# Include directories for the main node
target_include_directories(pure_pursuit_node PUBLIC
  include
)

# Link the vehicle model library
target_link_libraries(pure_pursuit_node
  vehicle_models
)

# Declare ROS2 dependencies for the node
ament_target_dependencies(pure_pursuit_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# =====================================================
# Step 4: Installation rules (optional but useful)
# =====================================================
# Install the executable and library
install(TARGETS
  pure_pursuit_node
  vehicle_models
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# =====================================================
# Step 5: Finalize the package
# =====================================================
# This macro declares that this is a ROS2 package
ament_package()
