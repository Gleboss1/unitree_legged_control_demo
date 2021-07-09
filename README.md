# Description

Here is a ROS package for Unitree robot A1. This package intended for demostration in Gazebo simulator low-lewel joints control (control the torque and position).

# Build and quick start
 + Install original [unitree_ros](https://github.com/unitreerobotics/unitree_ros) project
 + Install and build this package
 + Run the Gazebo simulator
 ```
 roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
 ```
 + Run low-lewel joints controler
 ```
 rosrun unitree_legged_control_demo demo_controler 
 ```
 # Demonstartion
 
 
 
