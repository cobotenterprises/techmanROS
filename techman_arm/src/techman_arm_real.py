#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import asyncio
import tf2_ros
import tf_conversions

import techmanpy
from techmanpy import TMConnectError

from techman_arm_base import TechmanArm

from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import TransformStamped as TransformStampedMsg

from techman_arm.msg import MoveTCPGoal
from techman_arm.msg import MoveJointsGoal
from techman_arm.srv import ExitListen, ExitListenResponse


class TechmanArmReal(TechmanArm):
   ''' ROS node to interact with physical Techman robotic arm. '''


   def __init__(self, robot_ip, planner):    
      super().__init__('techman_arm', 'Techman physical arm node', planner)
      self._robot_ip = robot_ip
      self._seq_id = 0

      # Set up service
      rospy.Service(f'/{self._node_name}/exit_listen', ExitListen, self._exit_listen)

      # Set up publisher
      self._tf2_pub = tf2_ros.TransformBroadcaster()


   def _execute_goal(self, goal): return asyncio.run(self._execute_goal_async(goal))
   async def _execute_goal_async(self, goal):      
      try:
         async with techmanpy.connect_sct(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            trsct = conn.start_transaction()

            if self._planner == 'moveit':
               # Plan trajectory
               plan_success, plan = self._plan_moveit_goal(goal)
               if not plan_success: return False

               # Execute trajectory
               for joint_state in plan.joint_trajectory.points:
                  trsct.move_to_joint_angles_path(
                     np.degrees(np.asarray(joint_state.positions)).tolist(),
                     0, 0, blending_perc=1.0
                  )

            if self._planner == 'tmflow':
               if isinstance(goal, MoveJointsGoal):
                  if goal.relative: 
                     trsct.move_to_relative_joint_angles_ptp(
                        goal.goal,
                        goal.speed,
                        self._acceleration_duration,
                        use_precise_positioning=self._precise_positioning
                     )
                  else:
                     trsct.move_to_joint_angles_ptp(
                        goal.goal,
                        goal.speed,
                        self._acceleration_duration,
                        use_precise_positioning=self._precise_positioning
                     )
               if isinstance(goal, MoveTCPGoal):
                  if goal.relative:
                     trsct.move_to_relative_point_ptp(
                        goal.goal,
                        goal.speed,
                        self._acceleration_duration,
                        use_precise_positioning=self._precise_positioning
                     )
                  else: 
                     trsct.move_to_point_ptp(
                        goal.goal,
                        goal.speed,
                        self._acceleration_duration,
                        use_precise_positioning=self._precise_positioning
                     )

            await trsct.submit()
            await conn.set_queue_tag(1, wait_for_completion=True)
            return True

      except TMConnectError: rospy.logerr('Could not execute motion goal: SCT not online')
      return False


   def _exit_listen(self, _): return asyncio.run(self._exit_listen_async())
   async def _exit_listen_async(self):
      try:
         async with techmanpy.connect_sct(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            await conn.exit_listen()
            return ExitListenResponse()
      except TMConnectError: rospy.logerr('Could not exit listen: SCT not online')


   def connect(self): asyncio.run(self._connect())
   async def _connect(self):
      try:
         async with techmanpy.connect_svr(robot_ip=self._robot_ip, conn_timeout=1) as conn:
            conn.add_broadcast_callback(self._tmserver_callback)
            rospy.loginfo('Connected to Techman arm')
            await conn.keep_alive(quit=rospy.is_shutdown)
      except TMConnectError:
         rospy.logerr('Techman arm not online, exiting...')
         rospy.signal_shutdown('Techman arm was not online')


   def _tmserver_callback(self, items):

      # Build joint state from TM server callback
      joint_state = JointStateMsg()
      joint_state.header.seq = self._seq_id; self._seq_id += 1
      joint_state.header.stamp = rospy.Time.now()
      joint_state.name = self.JOINTS
      joint_state.position = [np.radians(x) for x in items['Joint_Angle']]
      joint_state.velocity = [np.radians(x) for x in items['Joint_Speed']]
      joint_state.effort = items['Joint_Torque']
      super()._on_joint_state(joint_state)

      self._publish_tm_pose('real_flange', items['Coord_Robot_Flange'])


   def _publish_tm_pose(self, name, pos):
      tfmsg = TransformStampedMsg()
      tfmsg.header.stamp = rospy.Time.now()
      tfmsg.header.frame_id = 'world'
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


if __name__ == '__main__': 
   myargv = rospy.myargv(argv=sys.argv)

   # Check if robot IP is provided
   if len(myargv) < 2:
      print('Robot IP address not specified')
      exit()

   # Check if planner is provided
   if len(myargv) < 3:
      print('Motion planner not specified')
      exit()

   if myargv[2] != 'moveit' and myargv[2] != 'tmflow':
      print(f'Specified motion planner \'{myargv[2]}\' is invalid!')
      exit()

   # Start main logic
   node = TechmanArmReal(myargv[1], myargv[2])
   node.connect()
