#!/usr/bin/env python3

import os
import sys
import rospy
import actionlib
import time
import dateutil.parser
import numpy as np
from sympy.physics.vector import ReferenceFrame, express

from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Header as HeaderMsg

# import tf2
import tf_conversions
import tf2_ros
import geometry_msgs.msg

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
from robotic_arm.msg import MoveJointsAction, MoveJointsFeedback, MoveJointsResult
from robotic_arm.msg import MoveTCPAction, MoveTCPFeedback, MoveTCPResult


class SimulationPlaybackNode:
   ''' Simple ROS node to interface with a Techman robotic arm. '''


   def __init__(self, node_name, node_name_pretty):
      self._node_name = node_name
      self._node_name_pretty = node_name_pretty
      self._seq_id = 0

      # Initialise node
      rospy.init_node(self._node_name)

      # Bind callbacks
      rospy.on_shutdown(self._shutdown_callback)

      # Set up subscriber
      rospy.Subscriber('/robotic_arm/state', RobotStateMsg, self._robot_state_callback)

      # Set up transfer functions
      self._tf2_int = tf2_ros.Buffer()
      tf2_ros.TransformListener(self._tf2_int)
      self._tf2_pub = tf2_ros.TransformBroadcaster()

      rospy.loginfo(f'{self._node_name_pretty} has started.')


   def _robot_state_callback(self, state):
      print(state)


   def _shutdown_callback(self):
      rospy.loginfo(f'{self._node_name_pretty} was terminated.')


if __name__ == '__main__': 

   # Start main logic
   node = SimulationPlaybackNode('arm_playback', 'Simulation playback node')

   # Keep node alive until shutdown
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      rate.sleep()
