cmake_minimum_required(VERSION 3.1.3)
project(ur5_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(ur5_pick_place src/ur5_pick_place.cpp)
target_link_libraries(ur5_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
