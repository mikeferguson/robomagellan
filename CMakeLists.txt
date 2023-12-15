cmake_minimum_required(VERSION 3.5)
project(robomagellan)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(gps
  src/gps.cpp
  src/main.cpp
)
ament_target_dependencies(gps
  rclcpp
)
target_link_libraries(gps proj)

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
    gps
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
