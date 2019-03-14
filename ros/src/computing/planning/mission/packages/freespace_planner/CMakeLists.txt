cmake_minimum_required(VERSION 2.8.3)
project(freespace_planner)


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  visualization_msgs
  autoware_msgs
  astar_search
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  visualization_msgs
  autoware_msgs
  astar_search
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_navi
  src/astar_navi/astar_navi.cpp
  src/astar_navi/astar_navi_node.cpp
)

target_link_libraries(astar_navi
  ${catkin_LIBRARIES}
)

add_dependencies(astar_navi
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS astar_navi
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(astar_navi_test
    test/astar_navi/test_astar_navi.test
    test/astar_navi/src/test_astar_navi.cpp
    src/astar_navi/astar_navi.cpp
  )
  add_dependencies(astar_navi_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(astar_navi_test
    ${catkin_LIBRARIES}
    astar_search
  )
endif()
