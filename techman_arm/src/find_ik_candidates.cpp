#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "techman_arm/FindIKCandidates.h"

using namespace moveit;

robot_state::RobotStatePtr* robot_state_ptr;
moveit::core::JointModelGroup* joint_model_group_ptr;
planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor_ptr;

std::vector<const double*> candidates;

// bool collect_candidates(
//    robot_state::RobotState* state,
//    const robot_model::JointModelGroup* jmg,
//    const double* joint_angles
// ) {
//    // Normalize angles

//    // Check if not too close to previous candidates

//    // Apply joint positions
//    state->setJointGroupPositions(jmg, joint_positions);
//    state->update();

//    // Get Joint Values
//    // ^^^^^^^^^^^^^^^^
//    // We can retreive the current set of joint values stored in the state for the Panda arm.
//    std::vector<double> joint_values;
//    const std::vector<std::string>& joint_names = jmg->getVariableNames();
//    state->copyJointGroupPositions(jmg, joint_values);
//    for (std::size_t i = 0; i < joint_names.size(); ++i)
//    {
//       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//    }

//    if( planning_scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName()) ){
//       ROS_INFO("Got colliding state, skipping...");
//       return false;
//    } else {
//       ROS_INFO("No collisions found...");
//       return true;
//    }
// }

bool find_ik_candidates(techman_arm::FindIKCandidates::Request &req, techman_arm::FindIKCandidates::Request &res) {
   ROS_INFO("Checkpoint");
   return true;
}


//    // Get kinematic state
//    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//    robot_state::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
//    kinematic_state->setToDefaultValues();

//    // Get joint model group
//    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

//    std::vector<double> joint_values;
//    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//    for (std::size_t i = 0; i < req.joints.size(); ++i)
//    {
//       // ROS_INFO("Joint %d: %f", i, req.joints[i]);
//       joint_values[i] = (double) req.joints[i];
//    }
//    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
//    kinematic_state->update();

//    std::vector<double> joint_values_2;
//    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
//    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values_2);
//    for (std::size_t i = 0; i < joint_names.size(); ++i)
//    {
//       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//    }

//    // Get planning scene
//    planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor("robot_description");
//    planning_scene_monitor.startSceneMonitor("/move_group/monitored_planning_scene");
//    bool success = planning_scene_monitor.requestPlanningSceneState("/get_planning_scene");

//    //planning_scene::PlanningScene planning_scene = (*psm->getPlanningScene();
//    // lanning_interface::PlanningSceneInterface planning_scene_interface;
//    // plannin::PlanningSceneInterface planning_scene_interface;
//    auto planning_scenee = planning_scene_monitor.getPlanningScene();
//    planning_scenee->printKnownObjects(std::cout);

//    // std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface.getObjects();
//    // for (std::map<std::string, moveit_msgs::CollisionObject>::const_iterator
//    //       iter = objects.begin();
//    //    iter != objects.end(); iter++) {

//    //    std::cout << "Key: " << iter->first << std::endl << "Values:" << std::endl;
//    // }

//    if(  planning_scenee->isStateColliding(const_cast<const robot_state::RobotState&>(*kinematic_state), joint_model_group->getName()) ){
//       ROS_INFO("This WILL collide");
//    } else {
//       ROS_INFO("This WONT collide");
//    }
   
//    // if (false) {
//    //    const moveit::core::GroupStateValidityCallbackFn is_valid=
//    //       std::bind(
//    //          &isValid,
//    //          &planning_scene,
//    //          std::placeholders::_1,
//    //          std::placeholders::_2,
//    //          std::placeholders::_3);

//    //    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

//    //    kinematic_state->setFromIK(joint_model_group, end_effector_state, 5, is_valid);
//    // }

//    return true;
// }

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