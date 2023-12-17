cmake_minimum_required(VERSION 3.5)
project(robomagellan)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(gps_to_cart
  src/convert_gps.cpp
  src/gps_to_cart.cpp
)
ament_target_dependencies(gps_to_cart
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(gps_to_cart proj)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS
    gps_to_cart
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
