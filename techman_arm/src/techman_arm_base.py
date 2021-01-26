#!/usr/bin/env python3

import rospy
import tf2_ros
import actionlib
import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import JointState as JointStateMsg
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, Constraints, JointConstraint
from geometry_msgs.msg import Pose as PoseMsg, TransformStamped as TransformStampedMsg
from moveit_msgs.srv import GetPositionIK, GetStateValidity

from dynamic_reconfigure.server import Server
from techman_arm.cfg import RoboticArmConfig

from techman_arm.msg import MoveJointsAction, MoveJointsGoal, MoveJointsFeedback, MoveJointsResult
from techman_arm.msg import MoveTCPAction, MoveTCPGoal, MoveTCPFeedback, MoveTCPResult
from techman_arm.srv import FindIKCandidates


class TechmanArm:
   ''' Base class for Techman robotic arm nodes. '''

   JOINTS = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
   LINKS = ['arm_1_link', 'arm_2_link', 'shoulder_1_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
   MIN_MOVEIT_CONFORMITY = 0.95
   MAX_LINEAR_JOINT_FLUCTUATION = 20


   def __init__(self, node_name, node_name_pretty, planner):
      self._node_name = node_name
      self._node_name_pretty = node_name_pretty
      self._planner = planner
      self._joint_state = None

      # Linear buffer variables
      self._in_linear_buffer = False
      self._linear_buffer_size = -1
      self._linear_buffer_start_pose = None
      self._linear_buffer_waypoints = []

      # Initialise node
      rospy.init_node(self._node_name)

      # Set up publishers
      self._tf2_pub = tf2_ros.TransformBroadcaster()
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
            if isinstance(result_code, MoveItErrorCodes): result_code = result_code.val
            for attr in dir(MoveItErrorCodes):
               val = getattr(MoveItErrorCodes, attr)
               if isinstance(val, int) and val == result_code: return attr
         self._moveit_desc = moveit_desc
         # Get proxy for IK candidates lookup
         rospy.wait_for_service(f'/{self._node_name}/find_ik_candidates')
         self._compute_ik_cands = rospy.ServiceProxy(f'/{self._node_name}/find_ik_candidates', FindIKCandidates)
         # Get proxy for IK lookup
         rospy.wait_for_service('/compute_ik')
         self._compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)
         # Get proxy for collision check
         rospy.wait_for_service('/check_state_validity')
         self._check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

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
      self._joint_states_pub.publish(joint_state)

      # Start action servers if not started yet
      if not self._mja_started:
         self._mja_started = True
         self._move_joints_act.start()
      if not self._mta_started:
         self._mta_started = True
         self._move_tcp_act.start()

      # Publish action feedback
      if self._mja_in_feedback: 
         fod = MoveJointsFeedback(self._joint_state)
         #print(f'fod: {fod}')
         fod2 = MoveJointsResult(self._joint_state)
         #print(f'res fod2: {fod2}')
         self._move_joints_act.publish_feedback(fod)
      if self._mta_in_feedback: 
         fod = MoveTCPFeedback(self._joint_state)
         #print(f'fod: {fod}')
         self._move_tcp_act.publish_feedback(fod)


   def _obtain_linear_buffer_path(self):
      assert self._planner == 'moveit'

      ik_cands = self._compute_ik_cands(self._linear_buffer_start_pose).joint_angles
      if len(ik_cands) == 0:
         rospy.logerr('No IK candidates found for the requested pose')
         return None
      print(f'Checking {int(len(ik_cands)/6)} candidates')

      # Define joint constraint builder function
      def build_joint_constraints(joint_state):
         joint_constraints = []
         for i, joint_angle in enumerate(joint_state):
            jc = JointConstraint()
            jc.joint_name = self.JOINTS[i]
            jc.position = joint_angle
            jc.tolerance_above = np.radians(self.MAX_LINEAR_JOINT_FLUCTUATION)
            jc.tolerance_below = np.radians(self.MAX_LINEAR_JOINT_FLUCTUATION)
            jc.weight = 1.0
            joint_constraints.append(jc)
         return joint_constraints
      
      # Build robot state message
      robot_state = self._moveit_robot.get_current_state()
      robot_state.attached_collision_objects = self._moveit_scene.get_attached_objects().values()
      robot_state.joint_state.velocity = []
      robot_state.joint_state.effort = []
      robot_state.joint_state.position = [0, 0, 0, 0, 0, 0]

      # Build IK request message
      ik_request = PositionIKRequest()
      ik_request.group_name = "manipulator"
      ik_request.robot_state = robot_state
      ik_request.avoid_collisions = True
      ik_request.pose_stamped.header.frame_id = 'world'
      ik_request.timeout = rospy.Duration(secs=0.1)

      for ikc_i in range(0, len(ik_cands), 6):
         start_joint_state = np.radians(ik_cands[ikc_i:ikc_i+6])

         robot_state.joint_state.position[0:6] = start_joint_state
         is_valid = self._check_state_validity(robot_state, "manipulator", Constraints()).valid
         if not is_valid: print(f'Invalid: {ik_cands[ikc_i:ikc_i+6]}'); continue
         print(f'Valid: {ik_cands[ikc_i:ikc_i+6]}')

         motion_path, finished_chain = [start_joint_state.tolist()], True
         ik_request.constraints.joint_constraints = build_joint_constraints(start_joint_state)
         for waypoints in self._linear_buffer_waypoints:
            for waypoint in waypoints:
               ik_request.pose_stamped.pose = waypoint
               ik_request.robot_state.joint_state.position[0:6] = motion_path[-1]
               ik_res = self._compute_ik(ik_request)
               if ik_res.error_code.val != 1:
                  print('Couldn\'t finish linear chain, skipping')
                  finished_chain = False
                  break
               # Update constrains for next waypoint
               joint_values = ik_res.solution.joint_state.position
               ik_request.constraints.joint_constraints = build_joint_constraints(joint_values)
               motion_path.append(list(joint_values))
            if not finished_chain: break

         if finished_chain:
            print('Found valid begin joint state!')
            return motion_path
      
      print('Couldn\'t find valid joint state after going through all candidates')
      return None


   def _get_current_pose(self):
      if self._in_linear_buffer:
         if len(self._linear_buffer_waypoints) == 0: return self._linear_buffer_start_pose
         else: return self._linear_buffer_waypoints[-1][-1]
      else: return self._moveit_group.get_current_pose().pose


   def _plan_moveit_goal(self, goal):
      assert self._planner == 'moveit'

      if isinstance(goal, MoveJointsGoal):
         if self._in_linear_buffer: return True, None

         # Build joints dict
         joints_goal = [np.radians(x) for x in goal.goal]
         if goal.relative:
            curr_joints = self._moveit_group.get_current_joint_values()
            for i in range(len(curr_joints)): joints_goal[i] += curr_joints[i]
         joint_dict = {}
         for i in range(len(self.JOINTS)): joint_dict[self.JOINTS[i]] = joints_goal[i]

         # Plan trajectory
         self._moveit_group.set_joint_value_target(joint_dict)
         plan_success, plan, _, plan_result = self._moveit_group.plan()
         if not plan_success: rospy.logwarn(f'Could not plan joint goal: {self._moveit_desc(plan_result)}')
         return plan_success, plan
      
      if isinstance(goal, MoveTCPGoal):
         goal_pos, goal_rot = None, None
         if goal.relative:
            # Get current pose
            curr_pose_msg = self._get_current_pose()
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
               if self._in_linear_buffer: return True, None

               # Translate and rotate relative
               tcp_pos = curr_tcp_pos + curr_rot.apply(goal.goal[0:3])
               tcp_rot = curr_rot * Rotation.from_euler('xyz', np.array(goal.goal[3:6]), degrees=True)
               goal_pos = tcp_pos - tcp_rot.apply(np.array(goal.tcp))
               goal_rot = tcp_rot
         else:
            if goal.linear:
               # Get current pose
               curr_pose_msg = self._get_current_pose()
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
               if self._in_linear_buffer: return True, None

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
      
         if goal.prepare_linear > 0:
            self._in_linear_buffer = True
            self._linear_buffer_size = goal.prepare_linear

         if self._in_linear_buffer:
            if self._linear_buffer_start_pose is None:
               # Store start pose
               assert not isinstance(goal_pos, list)
               self._linear_buffer_start_pose = pose_msg(goal_pos, goal_rot)
            else:
               # Store linear path
               assert isinstance(goal_pos, list)
               self._linear_buffer_waypoints.append([pose_msg(goal_pos[i], goal_rot[i]) for i in range(len(goal_pos))])
            self._publish_waypoints([item for sublist in self._linear_buffer_waypoints for item in sublist])
            if len(self._linear_buffer_waypoints) < self._linear_buffer_size: return True, None
            else:
               # Calculate linear buffer start pose
               motion_path = self._obtain_linear_buffer_path()

               # Reset linear buffer variables
               self._in_linear_buffer = False
               self._linear_buffer_size = -1
               self._linear_buffer_start_pose = None
               self._linear_buffer_waypoints = []

               # Check if joint state was found
               if motion_path is None:
                  rospy.logerr('Could not plan buffered linear goal')
                  return False, None

               # Return the motion path
               return True, motion_path

         # Plan goal
         if isinstance(goal_pos, list):
            waypoints = [pose_msg(goal_pos[i], goal_rot[i]) for i in range(len(goal_pos))]
            plan, conformity = self._moveit_group.compute_cartesian_path(waypoints, 0.005, 0.0)
            if conformity < self.MIN_MOVEIT_CONFORMITY: rospy.logwarn(f'Could not plan pose goal, deviation was {1 - conformity}')
            return conformity >= self.MIN_MOVEIT_CONFORMITY, plan
         else:
            self._moveit_group.set_pose_target(pose_msg(goal_pos, goal_rot))
            plan_success, plan, _, plan_result = self._moveit_group.plan()
            #plan.joint_trajectory.points = plan.joint_trajectory.points[-1:]
            for i in range(len(plan.joint_trajectory.points)):
               plan.joint_trajectory.points[i].time_from_start.secs = 0
               plan.joint_trajectory.points[i].time_from_start.nsecs = i + 1
            print(plan)
            if not plan_success: rospy.logwarn(f'Could not plan pose goal: {self._moveit_desc(plan_result)}')
            return plan_success, plan

   
   def _publish_waypoints(self, waypoints):
      for i, waypoint in enumerate(waypoints):
         tfmsg = TransformStampedMsg()
         tfmsg.header.stamp = rospy.Time.now()
         tfmsg.header.frame_id = 'world'
         tfmsg.child_frame_id = f'waypoint-{i+1}'
         tfmsg.transform.translation.x = waypoint.position.x
         tfmsg.transform.translation.y = waypoint.position.y
         tfmsg.transform.translation.z = waypoint.position.z
         tfmsg.transform.rotation = waypoint.orientation
         self._tf2_pub.sendTransform(tfmsg)


   def _reconfigure_callback(self, config, level):
      self._precise_positioning = config.precise_positioning
      self._acceleration_duration = config.acceleration_duration
      self._speed_multiplier = config.speed_multiplier
      return config


   def _shutdown_callback(self):
      rospy.loginfo(f'{self._node_name_pretty} was terminated.')