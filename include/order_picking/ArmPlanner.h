/*
 * MoveTo.h
 *
 *  Created on: Oct 21, 2016
 *      Author: bbrito
 */

#ifndef ORDER_PICKING_INCLUDE_ORDER_PICKING_MOVETO_H_
#define ORDER_PICKING_INCLUDE_ORDER_PICKING_MOVETO_H_

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <order_picking/MoveToAction.h>
#include <order_picking/PlanToAction.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <boost/scoped_ptr.hpp>

class ArmPlanner
{
protected:

  ros::NodeHandle nh_;

  ros::Publisher display_publisher;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<order_picking::MoveToAction> move_to_;

  actionlib::SimpleActionServer<order_picking::PlanToAction> plan_to_;

  std::string action_name_;
  // create messages that are used to published feedback/result
  order_picking::MoveToFeedback feedback_;
  order_picking::MoveToResult result_;

  order_picking::PlanToFeedback plan_feedback_;
  order_picking::PlanToResult plan_result_;

  //MoveIt Planner

  boost::scoped_ptr<move_group_interface::MoveGroup> group;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success;

  geometry_msgs::Pose target_pos;

public:

  bool initialize();

  ArmPlanner(std::string name) : plan_to_(nh_, "PlanTo", boost::bind(&ArmPlanner::plan, this, _1), false), action_name_(name),move_to_(nh_, "MoveTo", boost::bind(&ArmPlanner::move, this, _1), false)
  {

	nh_.setParam("/global_param",name);
	nh_.setParam("/planning_plugin","ompl_interface/OMPLPlanner");
	display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	group.reset(new moveit::planning_interface::MoveGroup("arm"));
    plan_to_.start();
    move_to_.start();
  }

  ~ArmPlanner(void)
  {
  }

  int move(const order_picking::MoveToGoalConstPtr &goal);

  int plan(const order_picking::PlanToGoalConstPtr &goal);

};



#endif /* ORDER_PICKING_INCLUDE_ORDER_PICKING_MOVETO_H_ */
