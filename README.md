# Simulated Swarm

## About the Project

UCF Senior Design Project Sponsored by Lockheed Martin

Summary: An autonomous swarm of aerial drones(hector quadrotor) that uses object dection(YOLOv5) and pathfinding(explore_lite) to indentify our target.

## Introduction

1. Install Ubuntu 20.04 LTS 

3. Install ROS-Noetic
```bash
sudo apt-get update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install git
sudo apt-get install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/gazebo_models_worlds_collection/models" >> ~/.bashrc
```
__Open New Terminal__
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
Reference: http://wiki.ros.org/noetic/Installation/Ubuntu

3. Create Simulation Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

4. Install ROS packages
```bash
cd ~/catkin_ws/src
sudo git clone https://github.com/HuyPhamG/simulatedswarm.git
sudo apt install ros-noetic-multirobot-map-merge ros-noetic-explore-lite
sudo apt-get install ros-noetic-openslam-gmapping
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

###Running scripts:
Run world/navigation/map_merging
```bash
./world
```
Run drone_take_off/pathfinding/object_dectection
```bash
./search
```

