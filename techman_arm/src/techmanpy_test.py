#!/usr/bin/env python

import sys
import asyncio
import time
import numpy as np

import techmanpy
from techmanpy import TechmanException, TMConnectError

async def test_connection(robot_ip):
   try:
      async with techmanpy.connect_sct(robot_ip=robot_ip, conn_timeout=1) as conn:

         print('sending go...')
         await conn.move_to_joint_angles_ptp(
            [-90, 0, 90, -90, 90, 0],
            0.5,
            100,
            use_precise_positioning=False
         )
         print('waiting for go....')
         await conn.set_queue_tag(1, wait_for_completion=True)
         print('go done!')

         import pickle
         path = None
         with open('/home/jules/Code/Oxilia/oxilia-prima-clean/src/sensors/robotic_arm/trajectory.path', 'rb') as input:
            path = pickle.load(input)

         # print(path.joint_trajectory.points[0])

         trans = conn.start_transaction()
         # trans.enter_joint_pvt_mode()

         np.set_printoptions(suppress=True)
         for i, joint_pose in enumerate(path.joint_trajectory.points):
            print(f'{i+1}/{len(path.joint_trajectory.points)}: {np.degrees(np.asarray(joint_pose.positions)).tolist()}')
            
            # trans.move_to_joint_angles_ptp(
            #    np.degrees(np.asarray(joint_pose.positions)).tolist(),
            #    0.5,
            #    0,
            #    blending_perc=1.0,
            #    use_precise_positioning=False
            # )
            
            trans.move_to_joint_angles_path(
               np.degrees(np.asarray(joint_pose.positions)).tolist(),
               200,
               0,
               blending_perc=1.0
            )
            
            # trans.add_pvt_joint_angles(
            #    np.degrees(np.asarray(joint_pose.positions)).tolist(),
            #    [0, 0, 0, 0, 0, 0],
            #    # np.degrees(np.asarray(joint_pose.velocities)).tolist(),
            #    joint_pose.time_from_start.secs + 0.000000001 * joint_pose.time_from_start.nsecs
            # )
            #print(joint_pose.time_from_start.secs)
            #print(joint_pose.time_from_start.nsecs)
            #print(joint_pose.time_from_start.secs + 0.000000001 * joint_pose.time_from_start.nsecs)

         # exit()
         # trans.exit_pvt_mode()
         await trans.submit()
         await conn.set_queue_tag(1, wait_for_completion=True)

         exit()

         while True:

            print('sending home...') 
            await conn.move_to_joint_angles_ptp(
               [0, 0, 0, 0, 0, 0],
               0.5,
               100,
               use_precise_positioning=False
            )
            print('waiting for home....')
            await conn.set_queue_tag(1, wait_for_completion=True)
            print('home done!')

   except TMConnectError: print('Could not connect: SCT not online')

from moveit_msgs.msg import MoveItErrorCodes
if __name__ == '__main__':
   for attr in dir(MoveItErrorCodes):
      val = getattr(MoveItErrorCodes, attr)
      if isinstance(val, int):
         print(f'{attr}: {getattr(MoveItErrorCodes, attr)}')
      #print(getattr(MoveItErrorCodes, attr))
      #print(type(getattr(MoveItErrorCodes, attr)))
   #print(type(dir(MoveItErrorCodes)))
   exit()
   if len(sys.argv) == 2:
      try: asyncio.run(test_connection(sys.argv[1]))
      except KeyboardInterrupt: print() # terminate gracefully
   else: print(f'usage: {sys.argv[0]} <robot IP address>')
