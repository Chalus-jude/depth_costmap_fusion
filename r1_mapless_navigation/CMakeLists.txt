cmake_minimum_required(VERSION 3.8)
project(r1_mapless_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(fusion_node src/fusion_node.cpp)
ament_target_dependencies(fusion_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  message_filters cv_bridge)
target_include_directories(fusion_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fusion_node ${OpenCV_LIBS})



install(TARGETS fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
