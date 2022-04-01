install guide:
 sudo apt-get install ros-kinetic-dwa-local-planner
 sudo apt install ros-${ROS_DISTRO}-multirobot-map-merge ros-${ROS_DISTRO}-explore-lite
 sudo apt-get install ros-kinetic-openslam-gmapping

run guide:
 roslaunch hector_quadrotor_gazebo camera_launch.launch
 rosrun rviz rviz
  - open frontier_mapping config
 rosrun teleopkeyboard
  - fly up a little
 roslaunch quadrotor_navigation quadrotor_move_base.launch
 roslaunch explore_lite explore.launch
