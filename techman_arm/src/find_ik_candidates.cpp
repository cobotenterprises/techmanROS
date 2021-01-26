#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "techman_arm/FindIKCandidates.h"
#include <typeinfo>

using namespace moveit;

robot_state::RobotStatePtr* robot_state_ptr;
moveit::core::JointModelGroup* joint_model_group_ptr;
planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor_ptr;

std::vector<double> candidates;

bool collect_candidates(
   robot_state::RobotState* robot_state_cb,
   const robot_model::JointModelGroup* joint_model_group_cb,
   const double* joint_angles
) {

   std::vector<double> candidate;
   for(size_t i = 0; i < 6; i++ ) {
      // Convert to degrees
      double deg = std::fmod(*(joint_angles + i) * (180.0/3.141592653589793238463), 360.0);

      // Normalize between -180 and 180
      deg = std::fmod(deg + 180, 360.0);
      if (deg < 0) { deg += 360; }
      deg -= 180;

      candidate.push_back(deg);
   }

   // std::cout << "Angles: [ ";
   // for (std::vector<double>::const_iterator i = candidate.begin(); i != candidate.end(); ++i) {
   //    std::cout << *i << ' ';
   // }
   // std::cout << "]" << std::endl;

   bool all_too_small = false;
   for (int csi = 0; csi < candidates.size(); ++csi) {
      if (csi % 6 == 0) {
         if (all_too_small) { break; }
         all_too_small = true;
      }

      // std::cout << "index: " << csi << ", cs: " << candidates[csi] << ", c: " << candidate[csi % 6] << std::endl;
      double deg_diff = candidates[csi] - candidate[csi % 6];
      deg_diff += (deg_diff > 180.0) ? -360.0 : (deg_diff < -180.0) ? 360.0 : 0.0;
      // std::cout << "diff: " << deg_diff << std::endl;
      if (deg_diff < -10 || deg_diff > 10) { all_too_small = false; }
   }

   if (!all_too_small) {
      // Save this pose if it does not cause collisions
      robot_state_cb->setJointGroupPositions(joint_model_group_cb, joint_angles);
      robot_state_cb->update();
      bool pose_collides = (*planning_scene_monitor_ptr).getPlanningScene()->isStateColliding(const_cast<const robot_state::RobotState&>(*robot_state_cb), joint_model_group_cb->getName());
      if (!pose_collides) {
         candidates.insert(candidates.end(), candidate.begin(), candidate.end());
         // std::cout << "Saved candidate" << std::endl;
      }
   } 
   // else { std::cout << "All joints too similar to selected candidate, skipping..." << std::endl; }

   // if (candidates.size() == 10 * 6) {
   //    std::cout << "Obtained 10 joint states!" << std::endl;
   //    return true;
   // }

   return false;
}

bool find_ik_candidates(techman_arm::FindIKCandidates::Request &req, techman_arm::FindIKCandidates::Response &res) {

   candidates.clear();
   (*robot_state_ptr)->setFromIK(joint_model_group_ptr, req.pose, 0.3, collect_candidates);
   res.joint_angles = candidates;

   return true;
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "find_ik_candidates");
   ros::NodeHandle n;

   // Get robot state
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
   robot_state::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
   robot_state_ptr = &robot_state;

   // Get joint model group
   joint_model_group_ptr = kinematic_model->getJointModelGroup("manipulator");

   // Get planning scene monitor
   planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor("robot_description");
   planning_scene_monitor.startSceneMonitor("/move_group/monitored_planning_scene");
   planning_scene_monitor.requestPlanningSceneState("/get_planning_scene");
   planning_scene_monitor_ptr = &planning_scene_monitor;

   // Start service
   ros::ServiceServer service = n.advertiseService("/techman_arm/find_ik_candidates", find_ik_candidates);
   ROS_INFO("Find IK candidates node started");
   ros::spin();

   return 0;
}