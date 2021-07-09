# Description

Here is a ROS package for Unitree A1 robot. This package intended for demonstration in Gazebo simulator low-lewel joints control (control the torque and position).

# Build and quick start
 + Install original [unitree_ros](https://github.com/unitreerobotics/unitree_ros) project
 + Install and build this package
 + Run the Gazebo simulator
 ```
 roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
 ```
 + Run demo joints controler
 ```
 rosrun unitree_legged_control_demo demo_controler 
 ```
 # Demonstartion
 [Video](https://youtu.be/XWjoJwxsewI)
 
 ![demo](https://github.com/Gleboss1/unitree_legged_control_demo/blob/main/demo.gif)
 
 
