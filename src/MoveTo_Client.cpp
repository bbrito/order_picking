/*
 * MoveTo_Client.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: bbrito
 */
#include <order_picking/ArmPlanner.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");
  ros::NodeHandle nh;
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<order_picking::PlanToAction> ac("PlanTo", true);
  actionlib::SimpleActionClient<order_picking::MoveToAction> ab("MoveTo", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ab.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  order_picking::PlanToGoal goal;
  order_picking::MoveToGoal target;

  geometry_msgs::PoseStamped pose;
  int b;
  nh.getParam("pos",b);
  pose.header.frame_id = "arm_ee_link";
  pose.pose.position.x = -0.268;
  pose.pose.position.y = -0.880;
  pose.pose.position.z =0.979;
 /* pose.pose.orientation.x = 0.921;
  pose.pose.orientation.y = -0.389;
  pose.pose.orientation.z = 0.0;*/
  pose.pose.orientation.w = 1;

  goal.plan_to = pose;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    target.target_pos=pose;
    ab.sendGoal(target);
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
