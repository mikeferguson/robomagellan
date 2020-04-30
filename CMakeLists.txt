cmake_minimum_required(VERSION 2.8.3)
project(robomagellan)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    dynamic_reconfigure
    geographic_msgs
    geometry_msgs
    nav_core
    robot_localization
    roscpp
    tf2_geometry_msgs
    tf2_ros
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/TrajectoryRollout.cfg
)

add_action_files(
  DIRECTORY action
  FILES FollowWaypoints.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geographic_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib_msgs
    roscpp
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(waypoint_nav
  src/trajectory_rollout.cpp
  src/trajectory_rollout_ros.cpp
  src/waypoint_transformer.cpp
)
target_link_libraries(waypoint_nav
  ${catkin_LIBRARIES}
)
add_dependencies(waypoint_nav ${PROJECT_NAME}_gencfg)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(path_tests test/path_tests.cpp)
  target_link_libraries(path_tests
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(trajectory_rollout_tests test/trajectory_rollout_tests.cpp)
  target_link_libraries(trajectory_rollout_tests
    ${catkin_LIBRARIES}
    waypoint_nav
  )
endif()

install(TARGETS
  waypoint_nav
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
