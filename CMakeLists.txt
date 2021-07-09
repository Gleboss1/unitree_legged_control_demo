cmake_minimum_required(VERSION 3.0.2)
project(unitree_legged_control_demo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Declare ROS messages and services
add_message_files(FILES MotorCmd.msg MotorState.msg)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################

catkin_package()

## demo_controler node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(demo_controler src/demo_controler.cpp)
target_link_libraries(demo_controler ${catkin_LIBRARIES})
#add_dependencies(talker generate_messages_cpp)




