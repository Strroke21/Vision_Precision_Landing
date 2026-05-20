cmake_minimum_required(VERSION 3.8)

project(aruco_mavsdk)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(
  aruco_node
  src/PrecLander.cpp
)

ament_target_dependencies(
  aruco_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(
  aruco_node
  ${OpenCV_LIBS}
  mavsdk
  pthread
)

install(
  TARGETS aruco_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
