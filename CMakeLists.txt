cmake_minimum_required(VERSION 3.0.2)
project(visual_virtual_tether)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/MallardMixer.cfg
  cfg/VirtualTether.cfg
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)