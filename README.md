# conveyer-belt-cv

**Group project for Robotics AI. Team Members:**
- Christina Petschnig
- Elisabeth Bankl
- Sara Spreitzhofer
- Bodowin Bittner
- Marco Ross


## Environment

After building the ROS docker container, run the following commands to install required packages
```
sudo apt-get update
sudo apt-get install ros-noetic-rospy ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-rosgraph-msgs
pip install ultralytics
```


## Build and Execute

To build the application, cd to `~/catkin_ws` and run the following commands:
```
catkin_make
source devel/setup.bash
```
To start the application, cd to `~/catkin_ws/srd/fhtw` and run:
```
roslaunch conveyer-belt-cv-package/launch/project.launch 
```