#!/usr/bin/env python3

import os
import sys
import rospy
import rostopic
import actionlib
import time
import dateutil.parser
import numpy as np
import datetime
import threading
from scipy.spatial.transform import Rotation

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

from geometry_msgs.msg import Pose as PoseMsg, PoseStamped as PoseStampedMsg
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
      rospy.Subscriber('/joint_states', JointStateMsg, self._on_joint_state)

      # Initialize MoveIt
      self._moveit_scene = moveit_commander.PlanningSceneInterface()
      self._moveit_robot = moveit_commander.RobotCommander()
      self._moveit_group = moveit_commander.MoveGroupCommander('manipulator')

      # Start simulated TM server
      self._simulate_tmserver()


   async def _move_joints_async(self, goal):
      joints_goal = [np.radians(x) for x in goal.goal]
      if goal.relative:
         curr_joints = self._moveit_group.get_current_joint_values()
         for i in range(len(curr_joints)): joints_goal[i] += curr_joints[i]

      self._mja_in_feedback = True
      did_succeed = self._moveit_group.go(joints_goal, wait=True)
      self._moveit_group.stop()
      self._mja_in_feedback = False
      if did_succeed: self._move_joints_act.set_succeeded(MoveJointsFeedback(self._robot_state))
      else: self._move_joints_act.set_aborted(MoveJointsFeedback(self._robot_state))


   async def _move_tcp_async(self, goal):

      if goal.relative:
         # Get current pose
         curr_pose_msg = self._moveit_group.get_current_pose().pose
         cpmp, cpmo = curr_pose_msg.position, curr_pose_msg.orientation
         curr_pos = np.array([cpmp.x, cpmp.y, cpmp.z]) * 1_000
         curr_rot = Rotation.from_quat(np.array([cpmo.x, cpmo.y, cpmo.z, cpmo.w]))
         # Transform to TCP
         curr_pos += curr_rot.apply(np.array(goal.tcp))

         # Translate and rotate relative
         tcp_pos = curr_pos + curr_rot.apply(goal.goal[0:3])
         tcp_rot = curr_rot * Rotation.from_euler('xyz', np.array(goal.goal[3:6]), degrees=True)
      else: 
         tcp_pos = np.array(goal.goal[0:3])
         tcp_rot = Rotation.from_euler('xyz', np.array(goal.goal[3:6]), degrees=True)

      # Transform back from TCP
      tcp_pos -= tcp_rot.apply(np.array(goal.tcp))

      # Build pose message
      pose_goal = PoseMsg()
      tcp_rot_arr = tcp_rot.as_quat()
      pose_goal.orientation.x = tcp_rot_arr[0]
      pose_goal.orientation.y = tcp_rot_arr[1]
      pose_goal.orientation.z = tcp_rot_arr[2]
      pose_goal.orientation.w = tcp_rot_arr[3]
      pose_goal.position.x = tcp_pos[0] / 1_000
      pose_goal.position.y = tcp_pos[1] / 1_000
      pose_goal.position.z = tcp_pos[2] / 1_000
      self._moveit_group.set_pose_target(pose_goal)
      self._mta_in_feedback = True
      did_succeed = self._moveit_group.go(wait=True)
      self._moveit_group.stop()
      self._moveit_group.clear_pose_targets()
      self._mta_in_feedback = False
      if did_succeed: self._move_tcp_act.set_succeeded(MoveTCPFeedback(self._robot_state))
      else: self._move_tcp_act.set_aborted(MoveTCPFeedback(self._robot_state))


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
