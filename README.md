# conveyer-belt-cv
Group project for Robotics AI


## Environment
```
sudo apt-get update
sudo apt-get install ros-noetic-rospy ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-rosgraph-msgs
```




## Elisa edits

sudo apt-get update
sudo apt-get install ros-noetic-rospy ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-rosgraph-msgs ros-noetic-moveit

cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/universal_robot.git

rosdep update


mkdir -p ~/catkin_ws/src/fhtw/conveyer-belt-cv/simulation/include -> somehow need for catkin_make
catkin_make

source $HOME/catkin_ws/devel/setup.bash
source devel/setup.bash

