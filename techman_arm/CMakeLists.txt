cmake_minimum_required(VERSION 3.0.2)
project(techman_arm)

find_package(
   catkin REQUIRED
   # Specify dependencies
   COMPONENTS
   std_msgs
   sensor_msgs
   rospy
   message_generation
   dynamic_reconfigure
   actionlib_msgs
)

add_message_files(
   FILES
   RobotState.msg
)

add_service_files(
   FILES
   ExitListen.srv
   GetMode.srv
   SetTCP.srv
)

add_action_files(
    DIRECTORY
    action
    FILES
    MoveJoints.action
    MoveTCP.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/RoboticArm.cfg
)

catkin_package(
   # Specify package dependencies
   CATKIN_DEPENDS
   std_msgs
   sensor_msgs
   rospy
   message_runtime
   dynamic_reconfigure
   actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})