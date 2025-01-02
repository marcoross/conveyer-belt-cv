cmake_minimum_required(VERSION 3.0.2)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  nav_msgs
  sensor_msgs
  std_msgs
)

# This must come AFTER the catkin find_package so it doesn’t get clobbered
find_package(gazebo REQUIRED)

############################################
## Declare catkin_package for ROS usage   ##
############################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    gazebo_ros 
    nav_msgs 
    sensor_msgs 
    std_msgs
  # If you do NOT actually have an "include" folder or custom headers, 
  # don’t list them. In your case you can omit INCLUDE_DIRS entirely 
  # unless you have some .h files.
  #
  # If you don’t actually have external libraries to depend on, 
  # remove "DEPENDS system_lib" etc.
)

############################################
## Include and link paths                 ##
############################################
# Make sure we include the Gazebo headers and any catkin headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# This ensures we can link to Gazebo libraries
link_directories(${GAZEBO_LIBRARY_DIRS})

# Some plugins need extra compile definitions from Gazebo (e.g., -D_GLIBCXX_USE_C99)
add_definitions(${GAZEBO_CXX_FLAGS})

# (Optional) Set C++ standard if needed:
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

############################################
## Build the plugin                       ##
############################################
add_library(conveyor_belt_plugin SHARED src/conveyor_belt_plugin.cpp)
target_link_libraries(conveyor_belt_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
