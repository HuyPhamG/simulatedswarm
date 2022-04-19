# simulatedswarm

## About the Project

UCF Senior Design Project Sponsored by Lockheed Martin

Summary: An autonomous swarm of aerial drones(hector quadrotor) that uses object dection(YOLOv5) and pathfinding(explore_lite) to indentify our target.

## Introduction

1. Install Ubuntu 20.04 LTS 

3. Install ROS-Noetic and its dependencies
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



CLOSE AND RE-OPEN TERMINAL

```bash
sudo rosdep init
rosdep update
```

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
git clone https://github.com/chaolmu/gazebo_models_worlds_collection.git
git clone https://github.com/patrick1bauer/autonomous_search_with_ai.git
git clone https://github.com/Tossy0423/darknet_ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

