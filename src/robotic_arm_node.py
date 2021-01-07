#!/usr/bin/env python3

import os
import sys
import rospy
import time
import dateutil.parser

import asyncio
import techmanpy
from techmanpy import TechmanException, TMConnectError

from dynamic_reconfigure.server import Server
from robotic_arm.cfg import RoboticArmConfig

from std_srvs.srv import Empty
from robotic_arm.srv import ExitListen, ExitListenResponse
from robotic_arm.srv import GetMode, GetModeResponse
from robotic_arm.srv import SetTCP, SetTCPResponse

from robotic_arm.msg import RobotState as RobotStateMsg 


NODE_NAME='robotic_arm'
NODE_NAME_PRETTY='Robotic arm node'


class RoboticArmNode:
   ''' Simple ROS node to interface with a Techman robotic arm. '''


   def __init__(self, robot_ip):
      self._robot_ip = robot_ip

      # Bind callbacks
      rospy.on_shutdown(self._shutdown_callback)
      Server(RoboticArmConfig, self._reconfigure_callback)

      # Set up publishers
      self._broadcast_pub = rospy.Publisher(f'/{NODE_NAME}/state', RobotStateMsg, queue_size = 1)
      
      # Set up services
      rospy.Service(f'/{NODE_NAME}/exit_listen', ExitListen, self._exit_listen)
      rospy.Service(f'/{NODE_NAME}/get_mode', GetMode, self._get_mode)
      rospy.Service(f'/{NODE_NAME}/set_tcp', SetTCP, self._set_tcp)


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
      state = RobotStateMsg()

      # Set timestamp
      state.time = rospy.Time()      
      formatted_time = items['Current_Time']
      time = dateutil.parser.isoparse(formatted_time).timestamp()
      state.time.secs = int(time)
      state.time.nsecs = int((time % 1) * 1_000_000)

      # Set joint state
      state.joint_pos = items['Joint_Angle']
      state.joint_vel = items['Joint_Speed']
      state.joint_tor = items['Joint_Torque']
      
      # Publish
      self._broadcast_pub.publish(state)


   def _reconfigure_callback(self, config, level):
      self._precise_positioning = config.precise_positioning
      self._acceleration_duration = config.acceleration_duration
      self._speed_multiplier = config.speed_multiplier
      return config


   def _shutdown_callback(self):
      rospy.loginfo(f'{NODE_NAME_PRETTY} was terminated.')


def main():

   # Initialise node
   rospy.init_node(NODE_NAME)

   # Check if robot IP is provided
   if len(sys.argv) < 2:
      rospy.logerr(f'Robot IP address not specified')
      exit()

   rospy.loginfo(f'{NODE_NAME_PRETTY} has started.')

   # Start main logic
   node = RoboticArmNode(sys.argv[1])
   node.connect()

if __name__ == '__main__': main()
