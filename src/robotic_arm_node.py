#!/usr/bin/env python3

import os
import sys
import rospy
import actionlib
import time
import dateutil.parser
import numpy as np

import tf_conversions
import tf2_ros
import geometry_msgs.msg

import asyncio
import techmanpy
from techmanpy import TechmanException, TMConnectError

from dynamic_reconfigure.server import Server
from robotic_arm.cfg import RoboticArmConfig

from robotic_arm.srv import ExitListen, ExitListenResponse
from robotic_arm.srv import GetMode, GetModeResponse
from robotic_arm.srv import SetTCP, SetTCPResponse

from robotic_arm.msg import RobotState as RobotStateMsg
from robotic_arm.msg import MoveJointsAction, MoveJointsFeedback, MoveJointsResult
from robotic_arm.msg import MoveTCPAction, MoveTCPFeedback, MoveTCPResult


class RoboticArmNode:
   ''' Simple ROS node to interface with a Techman robotic arm. '''


   def __init__(self, node_name, node_name_pretty, robot_ip):
      self._node_name = node_name
      self._node_name_pretty = node_name_pretty
      self._robot_ip = robot_ip
      self._seq_id = 0

      # Initialise node
      rospy.init_node(self._node_name)

      # Bind callbacks
      rospy.on_shutdown(self._shutdown_callback)
      Server(RoboticArmConfig, self._reconfigure_callback)

      # Set up publishers
      self._broadcast_pub = rospy.Publisher(f'/{self._node_name}/state', RobotStateMsg, queue_size = 1)
      
      # Set up services
      rospy.Service(f'/{self._node_name}/exit_listen', ExitListen, self._exit_listen)
      rospy.Service(f'/{self._node_name}/get_mode', GetMode, self._get_mode)
      rospy.Service(f'/{self._node_name}/set_tcp', SetTCP, self._set_tcp)

      # Set up actions
      self._move_joints_act = actionlib.SimpleActionServer(f'/{self._node_name}/move_joints', MoveJointsAction, execute_cb=self._move_joints, auto_start = False)
      self._move_tcp_act = actionlib.SimpleActionServer(f'/{self._node_name}/move_tcp', MoveTCPAction, execute_cb=self._move_tcp, auto_start = False)
      self._mja_started, self._mta_started = False, False
      self._mja_in_feedback, self._mta_in_feedback = False, False

      rospy.loginfo(f'{self._node_name_pretty} has started.')


   def _move_joints(self, goal): asyncio.run(self._move_joints_async(goal))
   async def _move_joints_async(self, goal):
      try:
         async with techmanpy.connect_sct(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            if goal.relative: 
               await conn.move_to_relative_joint_angles_ptp(
                  goal.goal,
                  goal.speed,
                  self._acceleration_duration,
                  use_precise_positioning=self._precise_positioning
               )
            else: 
               await conn.move_to_joint_angles_ptp(
                  goal.goal,
                  goal.speed,
                  self._acceleration_duration,
                  use_precise_positioning=self._precise_positioning
               )               
            self._mja_in_feedback = True
            await conn.set_queue_tag(1, wait_for_completion=True)
            self._mja_in_feedback = False
            self._move_joints_act.set_succeeded(MoveJointsFeedback(self._state))
      except TMConnectError: rospy.logerr('Could not exit listen: SCT not online')


   def _move_tcp(self, goal): asyncio.run(self._move_tcp_async(goal))
   async def _move_tcp_async(self, goal):
      try:
         async with techmanpy.connect_sct(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            if goal.relative: 
               await conn.move_to_relative_point_ptp(
                  goal.goal,
                  goal.speed,
                  self._acceleration_duration,
                  use_precise_positioning=self._precise_positioning
               )
            else: 
               await conn.move_to_point_ptp(
                  goal.goal,
                  goal.speed,
                  self._acceleration_duration,
                  use_precise_positioning=self._precise_positioning
               )               
            self._mta_in_feedback = True
            await conn.set_queue_tag(2, wait_for_completion=True)
            self._mta_in_feedback = False
            self._move_tcp_act.set_succeeded(MoveTCPFeedback(self._state))
      except TMConnectError: rospy.logerr('Could not exit listen: SCT not online')


   def connect(self): asyncio.run(self._connect())
   async def _connect(self):
      try:
         async with techmanpy.connect_svr(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            conn.add_broadcast_callback(self._tmserver_callback)
            rospy.loginfo('Connected to robotic arm')
            await conn.keep_alive(quit=rospy.is_shutdown)
      except TMConnectError:
         rospy.logerr('Robotic arm not online, exiting...')


   def _exit_listen(self, _): return asyncio.run(self._exit_listen_async())
   async def _exit_listen_async(self):
      try:
         async with techmanpy.connect_sct(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            await conn.exit_listen()
            return ExitListenResponse()
      except TMConnectError: rospy.logerr('Could not exit listen: SCT not online')


   def _get_mode(self, _): return asyncio.run(self._get_mode_async())
   async def _get_mode_async(self):
      try:
         async with techmanpy.connect_sta(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            return GetModeResponse(await conn.is_listen_node_active())
      except TMConnectError: rospy.logerr('Could not get mode: STA not online')


   def _set_tcp(self, args): return asyncio.run(self._set_tcp_async(args))
   async def _set_tcp_async(self, args):
      try:
         async with techmanpy.connect_sct(robot_ip=self._robot_ip) as conn:
            await conn.set_tcp(args.tcp, weight=args.weight)
            return SetTCPResponse()
      except TMConnectError: rospy.logerr('Could not set TCP: SCT not online')


   def _tmserver_callback(self, items):
      self._state = RobotStateMsg()

      # Set timestamp
      self._state.time = rospy.Time()      
      formatted_time = items['Current_Time']
      time = dateutil.parser.isoparse(formatted_time).timestamp()
      self._state.time.secs = int(time)
      self._state.time.nsecs = int((time % 1) * 1_000_000_000)

      # Set joint state
      self._state.joint_pos = items['Joint_Angle']
      self._state.joint_vel = items['Joint_Speed']
      self._state.joint_tor = items['Joint_Torque']

      # Set cartesian state
      # self._state.flange_pos = items['Coord_Robot_Flange']

      # Publish to custom topics
      self._broadcast_pub.publish(self._state)
      if self._mja_in_feedback: self._move_joints_act.publish_feedback(MoveJointsFeedback(self._state))
      if self._mta_in_feedback: self._move_tcp_act.publish_feedback(MoveTCPFeedback(self._state))

      # Start action servers
      if not self._mja_started:
         self._mja_started = True
         self._move_joints_act.start()
      if not self._mta_started:
         self._mta_started = True
         self._move_tcp_act.start()


   def _reconfigure_callback(self, config, level):
      self._precise_positioning = config.precise_positioning
      self._acceleration_duration = config.acceleration_duration
      self._speed_multiplier = config.speed_multiplier
      return config


   def _shutdown_callback(self):
      rospy.loginfo(f'{self._node_name_pretty} was terminated.')


if __name__ == '__main__': 

   # Check if robot IP is provided
   if len(sys.argv) < 2:
      rospy.logerr(f'Robot IP address not specified')
      exit()

   # Start main logic
   node = RoboticArmNode('robotic_arm', 'Robotic arm node', sys.argv[1])
   node.connect()
