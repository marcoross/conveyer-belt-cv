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

sudo apt-get install ros-noetic-tf-conversions

cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/universal_robot.git

rosdep update


mkdir -p ~/catkin_ws/src/fhtw/conveyer-belt-cv/simulation/include -> somehow need for catkin_make
catkin_make

source $HOME/catkin_ws/devel/setup.bash
source devel/setup.bash


export DISPLAY=host.docker.internal:0

pip install pandas


### Elisa Edits 2


sudo apt-get update
sudo apt-get install ros-noetic-rospy ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-rosgraph-msgs ros-noetic-moveit

sudo apt-get install ros-noetic-tf-conversions

I now use the following Github Repo:
https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation

git clone https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/

catkin build
source devel/setup.bash