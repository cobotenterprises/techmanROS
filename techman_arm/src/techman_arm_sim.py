#!/usr/bin/env python3

import rospy

from techman_arm_base import TechmanArm

from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import TransformStamped as TransformStampedMsg


class TechmanArmSim(TechmanArm):
   ''' ROS node to interact with physical Techman robotic arm. '''


   def __init__(self):    
      super().__init__('techman_arm', 'Techman simulated arm node', 'moveit')

      rospy.Subscriber('/joint_states', JointStateMsg, self._on_joint_state)


   def _execute_goal(self, goal):
      # Plan trajectory
      plan_success, plan = self._plan_moveit_goal(goal)
      if not plan_success: return False

      # Buffer exection when plan is part of linear preparation
      if plan is None: return True

      if isinstance(plan, list):
         # Execute joints path
         for i, joint_state in enumerate(plan):
            self._moveit_group.go(joint_state, wait=True)
         return True
      else:
         # Execute MoveIt plan
         did_succeed = self._moveit_group.execute(plan, wait=True)
         self._moveit_group.clear_pose_targets()
         self._moveit_group.stop()
         return did_succeed


if __name__ == '__main__': 

   # Start main logic
   TechmanArmSim()

   # Keep node alive until shutdown
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      rate.sleep()
