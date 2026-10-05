cmake_minimum_required(VERSION 3.8)
project(depth_costmap_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# ---- depth_fusion_node ----
add_executable(depth_fusion_node depth_costmap_fusion/fusion_node.cpp)

ament_target_dependencies(depth_fusion_node
  rclcpp
  message_filters
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(depth_fusion_node
  ${OpenCV_LIBRARIES}
)

# ---- odom_tf_broadcaster ----
add_executable(odom_tf_broadcaster depth_costmap_fusion/odom_tf_broadcaster.cpp)

ament_target_dependencies(odom_tf_broadcaster
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  depth_fusion_node
  odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
