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
      self._moveit_group.set_joint_value_target({
         'shoulder_1_joint': joints_goal[0],
         'shoulder_2_joint': joints_goal[1],
         'elbow_joint': joints_goal[2],
         'wrist_1_joint': joints_goal[3],
         'wrist_2_joint': joints_goal[4],
         'wrist_3_joint': joints_goal[5]
      })
      result = self._moveit_group.plan()
      for i, value in enumerate(result):
         print(f'{i}: {str(value)[:50]}')
      did_succeed = self._moveit_group.execute(result[1], wait=True)
      self._moveit_group.clear_pose_targets()
      self._moveit_group.stop()
      self._mja_in_feedback = False
      if did_succeed: self._move_joints_act.set_succeeded(MoveJointsFeedback(self._robot_state))
      else: self._move_joints_act.set_aborted(MoveJointsFeedback(self._robot_state))


   async def _move_tcp_async(self, goal):
      goal_pos, goal_rot = None, None
      if goal.relative:
         # Get current pose
         curr_pose_msg = self._moveit_group.get_current_pose().pose
         cpmp, cpmo = curr_pose_msg.position, curr_pose_msg.orientation
         curr_pos = np.array([cpmp.x, cpmp.y, cpmp.z]) * 1_000
         curr_rot = Rotation.from_quat(np.array([cpmo.x, cpmo.y, cpmo.z, cpmo.w]))
         curr_tcp_pos = curr_pos + curr_rot.apply(np.array(goal.tcp))

         # Calculate goal
         if goal.linear:
            goal_pos, goal_rot = [], []
            path_res = int(max(np.linalg.norm(np.array(goal.goal)[0:3]), np.linalg.norm(np.array(goal.goal)[3:6])))
            if path_res == 0: path_res += 1
            for i in range(path_res):
               subgoal = np.array(goal.goal) * (i + 1)/path_res
               # Translate and rotate relative
               tcp_pos = curr_tcp_pos + curr_rot.apply(subgoal[0:3])
               tcp_rot = curr_rot * Rotation.from_euler('xyz', subgoal[3:6], degrees=True)
               goal_pos.append(tcp_pos - tcp_rot.apply(np.array(goal.tcp)))
               goal_rot.append(tcp_rot)
         else:
            # Translate and rotate relative
            tcp_pos = curr_tcp_pos + curr_rot.apply(goal.goal[0:3])
            tcp_rot = curr_rot * Rotation.from_euler('xyz', np.array(goal.goal[3:6]), degrees=True)
            goal_pos = tcp_pos - tcp_rot.apply(np.array(goal.tcp))
            goal_rot = tcp_rot
      else:
         if goal.linear:
            # Get current pose
            curr_pose_msg = self._moveit_group.get_current_pose().pose
            cpmp, cpmo = curr_pose_msg.position, curr_pose_msg.orientation
            curr_pos = np.array([cpmp.x, cpmp.y, cpmp.z]) * 1_000
            curr_rot = Rotation.from_quat(np.array([cpmo.x, cpmo.y, cpmo.z, cpmo.w]))
            curr_tcp_pos = curr_pos + curr_rot.apply(np.array(goal.tcp))

            goal_tcp_pos = np.array(goal.goal[0:3])
            goal_tcp_rot = Rotation.from_euler('xyz', np.array(goal.goal[3:6]), degrees=True)
            relative_rot = goal_tcp_rot.as_euler('xyz', degrees=True) - curr_rot.as_euler('xyz', degrees=True)
            # Normalize relative goal
            for i in range(3):
               while (relative_rot[i] <= -180): relative_rot[i] += 360
               while (relative_rot[i] > 180): relative_rot[i] -= 360

            # Interpolate trajectory
            goal_pos, goal_rot = [], []
            path_res = int(max(np.linalg.norm(goal_tcp_pos - curr_tcp_pos), np.linalg.norm(relative_rot)))
            if path_res == 0: path_res += 1
            for i in range(path_res):
               subpos = goal_tcp_pos - (path_res - i - 1)/path_res * (goal_tcp_pos - curr_tcp_pos)
               subrot = Rotation.from_euler('xyz', goal_tcp_rot.as_euler('xyz', degrees=True) - (path_res - i - 1)/path_res * relative_rot, degrees=True)
               goal_pos.append(subpos - subrot.apply(np.array(goal.tcp)))
               goal_rot.append(subrot)
         else:
            tcp_pos = np.array(goal.goal[0:3])
            tcp_rot = Rotation.from_euler('xyz', np.array(goal.goal[3:6]), degrees=True)
            goal_pos = tcp_pos - tcp_rot.apply(np.array(goal.tcp))
            goal_rot = tcp_rot

      # Helper method to build pose message
      def pose_msg(pos, rot):
         # Build pose message
         pose_goal = PoseMsg()
         rot_arr = rot.as_quat()
         pose_goal.orientation.x = rot_arr[0]
         pose_goal.orientation.y = rot_arr[1]
         pose_goal.orientation.z = rot_arr[2]
         pose_goal.orientation.w = rot_arr[3]
         pose_goal.position.x = pos[0] / 1_000
         pose_goal.position.y = pos[1] / 1_000
         pose_goal.position.z = pos[2] / 1_000
         return pose_goal

      # Plan and execute goal
      self._mta_in_feedback = True
      if isinstance(goal_pos, list):
         waypoints = [pose_msg(goal_pos[i], goal_rot[i]) for i in range(len(goal_pos))]
         plan, conformity = self._moveit_group.compute_cartesian_path(waypoints, 0.01, 0.0)
         import pickle
         with open('/home/jules/Code/Oxilia/oxilia-prima-clean/src/sensors/robotic_arm/trajectory.path', 'wb') as output:
            pickle.dump(plan, output, pickle.HIGHEST_PROTOCOL)
            print('dumped path')
         if conformity < 0.95: 
            rospy.logwarn(f'Could not execute linear trajectory, deviation was {1 - conformity}')
            self._move_tcp_act.set_aborted(MoveTCPFeedback(self._robot_state))
            return
         did_succeed = self._moveit_group.execute(plan, wait=True)
         self._moveit_group.stop()
      else:
         self._moveit_group.set_pose_target(pose_msg(goal_pos, goal_rot))
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
