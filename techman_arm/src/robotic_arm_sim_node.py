#!/usr/bin/env python3

import os
import sys
import rospy
import actionlib
import time
import dateutil.parser
import numpy as np
import datetime
import threading

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

import tf_conversions
import tf2_ros
import geometry_msgs.msg

from geometry_msgs.msg import Pose as PoseMsg
from sensor_msgs.msg import JointState as JointStateMsg

from techman_arm.msg import RobotState as RobotStateMsg
from techman_arm.msg import MoveJointsAction, MoveJointsFeedback, MoveJointsResult
from techman_arm.msg import MoveTCPAction, MoveTCPFeedback, MoveTCPResult

from techman_arm_node import TechmanArmNode

class TechmanArmSimNode(TechmanArmNode):
   ''' Simple ROS node to interface with a Techman robotic arm. '''


   def __init__(self, node_name, node_name_pretty):
      super().__init__(node_name, node_name_pretty, None)
      self._items = None

      # Set up subscriber
      rospy.Subscriber('/move_group/fake_controller_joint_states', JointStateMsg, self._on_joint_state)

      # Setup MoveIt
      self._moveit_group = moveit_commander.MoveGroupCommander('manipulator')
      self._moveit_group.set_max_acceleration_scaling_factor(0.5)
      self._moveit_group.set_max_velocity_scaling_factor(0.5)

      # Start simulated TM server
      self._simulate_tmserver()


   async def _move_joints_async(self, goal):      
      # We can get the joint values from the group and adjust some of the values:
      self._mja_in_feedback = True
      self._moveit_group.go([np.radians(x) for x in goal.goal], wait=True)
      self._moveit_group.stop()
      self._mja_in_feedback = False
      self._move_joints_act.set_succeeded(MoveJointsFeedback(self._robot_state))


   async def _move_tcp_async(self, goal):
      pose_goal = PoseMsg()
      q = tf_conversions.transformations.quaternion_from_euler(
         np.radians(goal.goal[3]),
         np.radians(goal.goal[4]),
         np.radians(goal.goal[5])
      )
      pose_goal.orientation.x = q[0]
      pose_goal.orientation.y = q[1]
      pose_goal.orientation.z = q[2]
      pose_goal.orientation.w = q[3]
      pose_goal.position.x = goal.goal[0] / 1_000
      pose_goal.position.y = goal.goal[1] / 1_000
      pose_goal.position.z = goal.goal[2] / 1_000
      self._moveit_group.set_pose_target(pose_goal)
      self._mta_in_feedback = True
      self._moveit_group.go(wait=True)
      self._moveit_group.stop()
      self._moveit_group.clear_pose_targets()
      self._mta_in_feedback = False
      self._move_tcp_act.set_succeeded(MoveTCPFeedback(self._robot_state))


   def _on_joint_state(self, joint_state):
      items = dict()
      items['Current_Time'] = datetime.datetime.now().isoformat()
      jointmap = [0, 0, 0, 0, 0, 0]
      for i, name in enumerate(joint_state.name):
         if name == 'shoulder_1_joint': jointmap[0] = i
         if name == 'shoulder_2_joint': jointmap[1] = i
         if name == 'elbow_joint': jointmap[2] = i
         if name == 'wrist_1_joint': jointmap[3] = i
         if name == 'wrist_2_joint': jointmap[4] = i
         if name == 'wrist_3_joint': jointmap[5] = i
      items['Joint_Angle'] = [] if len(joint_state.position) == 0 else [np.degrees(joint_state.position[i]) for i in jointmap]
      items['Joint_Speed'] = [] if len(joint_state.velocity) == 0 else [np.degrees(joint_state.velocity[i]) for i in jointmap]
      items['Joint_Torque'] = [] if len(joint_state.effort) == 0 else [joint_state.effort[i] for i in jointmap]
      self._items = items


   def _simulate_tmserver(self):
      if self._items is not None: super()._tmserver_callback(self._items)
      if not rospy.is_shutdown():
         threading.Timer(0.01, self._simulate_tmserver).start()


   def _shutdown_callback(self):
      rospy.loginfo(f'{self._node_name_pretty} was terminated.')


if __name__ == '__main__':

   # Start main logic
   node = TechmanArmSimNode('techman_arm', 'Techman arm simulation node')

   # Keep node alive until shutdown
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      rate.sleep()
