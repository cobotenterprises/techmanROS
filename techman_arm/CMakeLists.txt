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
   roscpp
   moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)

add_service_files(
   FILES
   ExitListen.srv
   FindIKCandidates.srv
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
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_executable(find_ik_candidates src/find_ik_candidates.cpp)
target_link_libraries(find_ik_candidates ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS find_ik_candidates DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
