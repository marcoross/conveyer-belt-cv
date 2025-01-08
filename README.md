# conveyer-belt-cv

**Group project for Robotics AI. Team Members:**
- Christina Petschnig
- Elisabeth Bankl
- Sara Spreitzhofer
- Bodowin Bittner
- Marco Ross


## Environment

After building the ROS docker container, run the following commands to install required packages
```bash
sudo apt-get update
sudo apt-get install ros-noetic-rospy ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-rosgraph-msgs
pip install ultralytics
```


## Build and Execute

To build the application, cd to `~/catkin_ws` and run the following commands:
```bash
catkin_make
source devel/setup.bash
```
To start the application, cd to `~/catkin_ws/src/fhtw` and run:
```bash
roslaunch conveyer-belt-cv-package/launch/project.launch 
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

*statt* Elisa Edits ausführen


sudo apt-get update
sudo apt-get install ros-noetic-rospy ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-rosgraph-msgs ros-noetic-moveit

sudo apt-get install ros-noetic-tf-conversions

I now use the following Github Repo:
https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation

git clone https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/

catkin_make
source devel/setup.bash


roslaunch moveit_setup_assistant setup_assistant.launch



Pick and place script from: https://github.com/moveit/moveit_tutorials/blob/master/doc/pick_place/src/pick_place_tutorial.cpp


to use moveit with the simulation, call
roslaunch simulation simulation.launch 
(as usual) and then
roslaunch ur5_moveit_config demo.launch