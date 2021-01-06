cmake_minimum_required(VERSION 3.0.2)
project(robotic_arm)

find_package(
   catkin REQUIRED
   # Specify dependencies
   COMPONENTS
   std_msgs
   rospy
   message_generation
   dynamic_reconfigure
)

add_message_files(
   FILES
   RobotState.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/RoboticArm.cfg
)

catkin_package(
   # Specify package dependencies
   CATKIN_DEPENDS
   std_msgs
   rospy
   message_runtime
   dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS})