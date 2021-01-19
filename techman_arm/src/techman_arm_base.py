#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import JointState as JointStateMsg
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose as PoseMsg

from dynamic_reconfigure.server import Server
from techman_arm.cfg import RoboticArmConfig

from techman_arm.msg import MoveJointsAction, MoveJointsGoal, MoveJointsFeedback, MoveJointsResult
from techman_arm.msg import MoveTCPAction, MoveTCPGoal, MoveTCPFeedback, MoveTCPResult


class TechmanArm:
   ''' Base class for Techman robotic arm nodes. '''

   JOINTS = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
   MIN_MOVEIT_CONFORMITY = 0.95


   def __init__(self, node_name, node_name_pretty, planner):
      self._node_name = node_name
      self._node_name_pretty = node_name_pretty
      self._planner = planner
      self._joint_state = None

      # Initialise node
      rospy.init_node(self._node_name)

      # Set up publisher
      self._joint_states_pub = rospy.Publisher(f'/{self._node_name}/joint_states', JointStateMsg, queue_size = 1)

      # Set up actions
      self._move_joints_act = actionlib.SimpleActionServer(f'/{self._node_name}/move_joints', MoveJointsAction, execute_cb=self._move_joints, auto_start = False)
      self._move_tcp_act = actionlib.SimpleActionServer(f'/{self._node_name}/move_tcp', MoveTCPAction, execute_cb=self._move_tcp, auto_start = False)
      self._mja_started, self._mta_started = False, False
      self._mja_in_feedback, self._mta_in_feedback = False, False

      # Bind callbacks
      rospy.on_shutdown(self._shutdown_callback)
      Server(RoboticArmConfig, self._reconfigure_callback)

      # Initialize MoveIt! if it is used
      if self._planner == 'moveit':
         import moveit_commander
         self._moveit_scene = moveit_commander.PlanningSceneInterface()
         self._moveit_robot = moveit_commander.RobotCommander()
         self._moveit_group = moveit_commander.MoveGroupCommander('manipulator')
         # Define lambda to lookup status code
         def moveit_desc(result_code):
            for attr in dir(MoveItErrorCodes):
               val = getattr(MoveItErrorCodes, attr)
               if isinstance(val, int) and val == result_code: return attr
         self._moveit_desc = moveit_desc

      rospy.loginfo(f'{self._node_name_pretty} has started.')


   def _move_joints(self, goal):
      self._mja_in_feedback = True
      did_succeed = self._execute_goal(goal)
      self._mja_in_feedback = False
      if did_succeed: self._move_joints_act.set_succeeded(MoveJointsFeedback(self._joint_state))
      else: self._move_joints_act.set_aborted(MoveJointsFeedback(self._joint_state))


   def _move_tcp(self, goal):
      self._mta_in_feedback = True
      did_succeed = self._execute_goal(goal)
      self._mta_in_feedback = False
      if did_succeed: self._move_tcp_act.set_succeeded(MoveTCPFeedback(self._joint_state))
      else: self._move_tcp_act.set_aborted(MoveTCPFeedback(self._joint_state))


   def _execute_goal(self, goal):
      # Implemented by subclass
      return False


   def _on_joint_state(self, joint_state):
      self._joint_state = joint_state

      # Start action servers if not started yet
      if not self._mja_started:
         self._mja_started = True
         self._move_joints_act.start()
      if not self._mta_started:
         self._mta_started = True
         self._move_tcp_act.start()

      # Publish action feedback
      if self._mja_in_feedback: self._move_joints_act.publish_feedback(MoveJointsFeedback(self._joint_state))
      if self._mta_in_feedback: self._move_tcp_act.publish_feedback(MoveTCPFeedback(self._joint_state))


   def _plan_moveit_goal(self, goal):
      assert self._planner == 'moveit'

      if isinstance(goal, MoveJointsGoal):
         # Build joints dict
         joints_goal = [np.radians(x) for x in goal.goal]
         if goal.relative:
            curr_joints = self._moveit_group.get_current_joint_values()
            for i in range(len(curr_joints)): joints_goal[i] += curr_joints[i]
         joint_dict = {}
         for i in range(self.JOINTS): joint_dict[self.JOINTS[i]] = joints_goal[i]

         # Plan trajectory
         self._moveit_group.set_joint_value_target(joint_dict)
         plan_success, plan, _, plan_result = self._moveit_group.plan()
         if not plan_success: rospy.logwarn(f'Could not plan joint goal: {self._moveit_desc(plan_result)}')
         return plan_success, plan
      
      if isinstance(goal, MoveTCPGoal):
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

         # Plan goal
         if isinstance(goal_pos, list):
            waypoints = [pose_msg(goal_pos[i], goal_rot[i]) for i in range(len(goal_pos))]
            plan, conformity = self._moveit_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if conformity < self.MIN_MOVEIT_CONFORMITY: rospy.logwarn(f'Could not plan pose goal, deviation was {1 - conformity}')
            return conformity >= self.MIN_MOVEIT_CONFORMITY, plan
         else:
            self._moveit_group.set_pose_target(pose_msg(goal_pos, goal_rot))
            plan_success, plan, _, plan_result = self._moveit_group.plan()
            if not plan_success: rospy.logwarn(f'Could not plan pose goal: {self._moveit_desc(plan_result)}')
            return plan_success, plan


   def _reconfigure_callback(self, config, level):
      self._precise_positioning = config.precise_positioning
      self._acceleration_duration = config.acceleration_duration
      self._speed_multiplier = config.speed_multiplier
      return config


   def _shutdown_callback(self):
      rospy.loginfo(f'{self._node_name_pretty} was terminated.')
