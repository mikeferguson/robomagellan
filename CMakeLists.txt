cmake_minimum_required(VERSION 3.5)
project(robomagellan)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLLA.srv"
  "srv/ToLLA.srv"
  DEPENDENCIES geographic_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

add_executable(gps_to_cart
  src/convert_gps.cpp
  src/gps_to_cart.cpp
)
ament_target_dependencies(gps_to_cart
  geographic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(gps_to_cart proj "${cpp_typesupport_target}")

add_executable(rolling_ground_filter src/rolling_ground_filter.cpp)
ament_target_dependencies(rolling_ground_filter
  angles
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

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
    rolling_ground_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
