cmake_minimum_required(VERSION 2.8.3)
project(ros_control_client_py)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  catkin
  control_msgs
  controller_manager
  controller_manager_msgs
  rospy
  sensor_msgs
  trajectory_msgs
)

find_package(
  openrave
)

catkin_python_setup()
