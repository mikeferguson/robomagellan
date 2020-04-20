cmake_minimum_required(VERSION 2.8.3)
project(robomagellan)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    robot_localization
    roscpp
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(waypoint_nav
  src/waypoint_transformer.cpp
)
target_link_libraries(waypoint_nav
  ${catkin_LIBRARIES}
)

add_executable(waypoint_nav_node
  src/waypoint_nav_node.cpp
)
target_link_libraries(waypoint_nav_node
  waypoint_nav
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(path_tests
    test/path_tests.cpp
  )
  target_link_libraries(path_tests
    ${catkin_LIBRARIES}
  )
endif()
