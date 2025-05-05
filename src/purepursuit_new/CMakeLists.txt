cmake_minimum_required(VERSION 3.8)
project(purepursuit_new)

# נדרש כדי להשתמש ב־C++17
add_compile_options(-std=c++17)

# חבילות ROS2 נדרשות
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ספריית המודלים - כולל הממשק והמודל האופניים
add_library(vehicle_models
  src/bicycle_model.cpp
)

target_include_directories(vehicle_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vehicle_models
  rclcpp
)

# הקובץ הראשי של הצומת
add_executable(pure_pursuit_node
  src/pure_pursuit_node.cpp
)

target_include_directories(pure_pursuit_node PUBLIC
  include
)

target_link_libraries(pure_pursuit_node
  vehicle_models
)

ament_target_dependencies(pure_pursuit_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# התקנה (אם רוצים לעשות install)
install(TARGETS
  pure_pursuit_node
  vehicle_models
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
