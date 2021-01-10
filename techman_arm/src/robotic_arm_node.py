#!/usr/bin/env python3

import os
import sys
import rospy
import actionlib
import time
import dateutil.parser
import numpy as np

import asyncio
import techmanpy
from techmanpy import TechmanException, TMConnectError

import tf_conversions
import tf2_ros
import geometry_msgs.msg

from sensor_msgs.msg import JointState as JointStateMsg

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
      self._robot_state = None
      self._joint_state = None

      # Initialise node
      rospy.init_node(self._node_name)

      # Bind callbacks
      rospy.on_shutdown(self._shutdown_callback)
      Server(RoboticArmConfig, self._reconfigure_callback)

      # Set up publishers
      self._broadcast_pub = rospy.Publisher(f'/{self._node_name}/state', RobotStateMsg, queue_size = 1)
      self._joint_states_pub = rospy.Publisher(f'/{self._node_name}/joint_states', JointStateMsg, queue_size = 1)
      self._tf2_pub = tf2_ros.TransformBroadcaster()
      
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
            self._move_joints_act.set_succeeded(MoveJointsFeedback(self._robot_state))
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
            self._move_tcp_act.set_succeeded(MoveTCPFeedback(self._robot_state))
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
         rospy.signal_shutdown('Robotic arm was not online')


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

      # Publish robot state
      self._robot_state = RobotStateMsg()
      self._robot_state.time = rospy.Time()      
      formatted_time = items['Current_Time']
      time = dateutil.parser.isoparse(formatted_time).timestamp()
      self._robot_state.time.secs = int(time)
      self._robot_state.time.nsecs = int((time % 1) * 1_000_000_000)
      self._robot_state.joint_pos = items['Joint_Angle']
      self._robot_state.joint_vel = items['Joint_Speed']
      self._robot_state.joint_tor = items['Joint_Torque']
      # self._robot_state.flange_pos = items['Coord_Robot_Flange']
      self._broadcast_pub.publish(self._robot_state)

      # Publish joint state
      self._seq_id += 1
      self._joint_state = JointStateMsg()
      self._joint_state.header.seq = self._seq_id
      self._joint_state.header.stamp = rospy.Time.now()
      self._joint_state.name = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
      self._joint_state.position = [np.radians(x) for x in items['Joint_Angle']]
      self._joint_state.velocity = [np.radians(x) for x in items['Joint_Speed']]
      self._joint_state.effort = items['Joint_Torque']
      self._joint_states_pub.publish(self._joint_state)

      # Start action servers if not started yet
      if not self._mja_started:
         self._mja_started = True
         self._move_joints_act.start()
      if not self._mta_started:
         self._mta_started = True
         self._move_tcp_act.start()

      # Publish action feedback
      if self._mja_in_feedback: self._move_joints_act.publish_feedback(MoveJointsFeedback(self._robot_state))
      if self._mta_in_feedback: self._move_tcp_act.publish_feedback(MoveTCPFeedback(self._robot_state))


   def _publish_reference_frame(self, name, pos, parent='world'):
      tfmsg = geometry_msgs.msg.TransformStamped()
      tfmsg.header.stamp = rospy.Time.now()
      tfmsg.header.frame_id = parent
      tfmsg.child_frame_id = name
      tfmsg.transform.translation.x = float(pos[0]) / 1_000
      tfmsg.transform.translation.y = float(pos[1]) / 1_000
      tfmsg.transform.translation.z = float(pos[2]) / 1_000
      q = tf_conversions.transformations.quaternion_from_euler(
         np.radians(pos[3]),
         np.radians(pos[4]),
         np.radians(pos[5])
      )
      tfmsg.transform.rotation.x = q[0]
      tfmsg.transform.rotation.y = q[1]
      tfmsg.transform.rotation.z = q[2]
      tfmsg.transform.rotation.w = q[3]
      self._tf2_pub.sendTransform(tfmsg)


   def _reconfigure_callback(self, config, level):
      self._precise_positioning = config.precise_positioning
      self._acceleration_duration = config.acceleration_duration
      self._speed_multiplier = config.speed_multiplier
      return config


   def _shutdown_callback(self):
      rospy.loginfo(f'{self._node_name_pretty} was terminated.')


if __name__ == '__main__': 

   # Check if robot IP is provided
   myargv = rospy.myargv(argv=sys.argv)
   if len(myargv) < 2:
      rospy.logerr(f'Robot IP address not specified')
      exit()

   # Start main logic
   node = RoboticArmNode('robotic_arm', 'Robotic arm node', myargv[1])
   node.connect()
