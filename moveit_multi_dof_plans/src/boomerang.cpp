/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "nav_msgs/Path.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/RobotState.h"
#include "moveit/robot_state/conversions.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "testing/GetRobotTrajectoryFromPath.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bebop_boomerang");
  ros::NodeHandle node_handle;

   std::string planningGroupName;
    node_handle.param<std::string>("planning_group", planningGroupName, "Bebop");//AndKinect

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = planningGroupName;

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom");
  visual_tools.deleteAllMarkers();

  for (int i = 0; i < move_group.getJointNames().size(); i++)
  {
    ROS_INFO_NAMED("boomerang", "Joint %s", move_group.getJointNames()[i].c_str());
  }
  for (int i = 0; i < move_group.getLinkNames().size(); i++)
  {
    ROS_INFO_NAMED("boomerang", "Link %s", move_group.getLinkNames()[i].c_str());
  }

  ROS_INFO_NAMED("boomerang", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  move_group.setPlanningTime(2);
  move_group.setWorkspace(-10, -10, 0.7, 10, 10, 1.7);

  visual_tools.trigger();
  visual_tools.prompt("next step");
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  move_group.setStartStateToCurrentState();
  
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = 0; //x
  joint_group_positions[1] = 0; //y
  joint_group_positions[2] = 1; //z
//  joint_group_positions[3] = 1; //quat w
//  joint_group_positions[4] = 0; //quat x
//  joint_group_positions[5] = 0; //quat y
//  joint_group_positions[6] = 0; //quat z


  move_group.setJointValueTarget(joint_group_positions);

  bool success = move_group.plan(my_plan);
  ROS_INFO_NAMED("boomerang", "Visualizing plan %s, got %d states.", success ? "" : "FAILED", (int)my_plan.trajectory_.multi_dof_joint_trajectory.points.size());

/*  trajectory_msgs::MultiDOFJointTrajectoryPoint first_point;
  geometry_msgs::Transform transform;
  transform.translation.x = my_plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.x;
  transform.translation.y = my_plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.y;
  transform.translation.z = my_plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.z;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0;
  transform.rotation.w = 1;
  first_point.transforms.push_back(transform);

  my_plan.trajectory_.multi_dof_joint_trajectory.points.insert(my_plan.trajectory_.multi_dof_joint_trajectory.points.begin(), first_point); 

  int size = my_plan.trajectory_.multi_dof_joint_trajectory.points.size()-1;

  trajectory_msgs::MultiDOFJointTrajectoryPoint last_point;
  geometry_msgs::Transform transform2;
  transform2.translation.x = my_plan.trajectory_.multi_dof_joint_trajectory.points[size].transforms[0].translation.x;
  transform2.translation.y = my_plan.trajectory_.multi_dof_joint_trajectory.points[size].transforms[0].translation.y;
  transform2.translation.z = my_plan.trajectory_.multi_dof_joint_trajectory.points[size].transforms[0].translation.z;
  transform2.rotation.x = 0;
  transform2.rotation.y = 0;
  transform2.rotation.z = 0;
  transform2.rotation.w = 1;
  last_point.transforms.push_back(transform2);

  my_plan.trajectory_.multi_dof_joint_trajectory.points.push_back(last_point); */

  for (int i = 0; i < my_plan.trajectory_.multi_dof_joint_trajectory.points.size(); i++)
  {
     my_plan.trajectory_.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(0.2*i);
  }

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);


  visual_tools.trigger();
  visual_tools.prompt("next step");



  move_group.execute(my_plan);

  visual_tools.trigger();
  visual_tools.prompt("next step");


  ros::shutdown();
  return 0;
}
