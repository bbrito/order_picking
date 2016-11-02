/*
 * MoveTo_Client.cpp
 *
 *  Created on: Oct 21, 2016
 *    Author: bbrito
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

  /*Testing Service Client

    ros::ServiceClient client = nh.serviceClient<order_picking::AddColisionObject>("Add_Obstacle");
    order_picking::AddColisionObject srv;
    srv.request.name_object="obstacle";
    srv.request.dim.x = 0.5;
    srv.request.dim.y = 0.5;
    srv.request.dim.z = 0.5;
    srv.request.CoM.header.frame_id="odom_combined";
    srv.request.CoM.pose.position.x =  0;
    srv.request.CoM.pose.position.y =2;
    srv.request.CoM.pose.position.z = 0 ;
    if (client.call(srv))
    {
      ROS_INFO("Sum: %s", srv.response.attached ? "true" : "false");
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
    ros::ServiceClient client2 = nh.serviceClient<order_picking::AddColisionObject>("Attach_Obstacle");
    order_picking::AddColisionObject srv2;
    srv2.request.name_object="object";
    srv2.request.dim.x = 0.15;
    srv2.request.dim.y = 0.15;
    srv2.request.dim.z = 0.15;
    srv2.request.CoM.header.frame_id="arm_ee_link";
    srv2.request.CoM.pose.position.x =  srv2.request.dim.x/2;
    srv2.request.CoM.pose.position.y =0;
    srv2.request.CoM.pose.position.z = 0;
    if (client2.call(srv2))
    {
      ROS_INFO("Sum: %s", srv2.response.attached ? "true" : "false");
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }


  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ab.waitForServer(); //will wait for infinite time*/

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  order_picking::PlanToGoal goal;
  order_picking::MoveToGoal target;

  geometry_msgs::PoseStamped pose;
  int b;
  nh.getParam("pos",b);
  pose.header.frame_id = "arm_ee_link";
  pose.pose.position.x = 0.51;
  pose.pose.position.y = 0.81;
  pose.pose.position.z = 1.20;
  pose.pose.orientation.x = -0.0026;
  pose.pose.orientation.y = -0.0203;
  pose.pose.orientation.z = 0.6959;
  pose.pose.orientation.w = 0.71785;

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

  sleep(10.0);

  /*ros::ServiceClient client3 = nh.serviceClient<order_picking::AddColisionObject>("Detach_Obstacle");
      order_picking::AddColisionObject srv3;

      if (client3.call(srv3))
      {
        ROS_INFO("Removed: %s", srv.response.attached ? "true" : "false");
      }
      else
      {
        ROS_ERROR("Failed to remove");
        return 1;
      }*/


  //exit
  return 0;
}
