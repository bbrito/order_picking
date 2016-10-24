/*
 * MoveTo.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: bbrito
 */

#include <order_picking/ArmPlanner.h>


int ArmPlanner::move(const order_picking::MoveToGoalConstPtr &goal)
{
	// publish info to the console for the user
	ROS_INFO("Move %s: Executing, trajectory to [%f %f %f]", action_name_.c_str(), goal->target_pos.pose.position.x,goal->target_pos.pose.position.y,goal->target_pos.pose.position.z);

	// check that preempt has not been requested by the client
	if (move_to_.isPreemptRequested() || !ros::ok())
	{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	// set the action state to preempted
	move_to_.setPreempted();
	success = false;
	}

	result_.arrived = group->execute(my_plan);

	// publish the feedback
	move_to_.publishFeedback(feedback_);



	//if(feedback_.actual_pos==goal->position)


	  ROS_INFO("%s: Succeeded", action_name_.c_str());
	  // set the action state to succeeded
	  move_to_.setSucceeded(result_);

}

bool ArmPlanner::initialize(){


	/* Sleep a little to allow time to startup rviz, etc. */
	ROS_INFO("3");
	ros::WallDuration sleep_time(5.0);
	sleep_time.sleep();

}

int ArmPlanner::plan(const order_picking::PlanToGoalConstPtr &goal){

	// publish info to the console for the user
	ROS_INFO("%s: Planning, creating trajectory to [%f %f %f]", action_name_.c_str(), goal->plan_to.pose.position.x,goal->plan_to.pose.position.y,goal->plan_to.pose.position.z);

	// check that preempt has not been requested by the client
	if (plan_to_.isPreemptRequested() || !ros::ok())
	{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	// set the action state to preempted
	plan_to_.setPreempted();
	success = false;
	}

	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

	//target_pos.header.frame_id =  group.getEndEffectorLink().c_str();
	target_pos.position.x = goal->plan_to.pose.position.x;
	target_pos.position.y = goal->plan_to.pose.position.y;
	target_pos.position.z = goal->plan_to.pose.position.z;
	target_pos.orientation.x = goal->plan_to.pose.orientation.x;
	target_pos.orientation.y = goal->plan_to.pose.orientation.y;
	target_pos.orientation.z = goal->plan_to.pose.orientation.z;
	target_pos.orientation.w = goal->plan_to.pose.orientation.w;
	group->setPoseTarget(target_pos);

	//A tolerance of 0.01 m is specified in position and 0.01 radians in orientation
	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	success = group->plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);

	  if (success)
	    {
	      ROS_INFO("Visualizing plan 1 (again)");
	      display_trajectory.trajectory_start = my_plan.start_state_;
	      display_trajectory.trajectory.push_back(my_plan.trajectory_);
	      display_publisher.publish(display_trajectory);
	      /* Sleep to give Rviz time to visualize the plan. */
	      sleep(5.0);
	    }


	  // publish the feedback
	  plan_to_.publishFeedback(plan_feedback_);
	  // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
	  //r.sleep();


	//if(feedback_.actual_pos==goal->position)

	  plan_result_.got_plan = success;
	  ROS_INFO("%s: Succeeded", action_name_.c_str());
	  // set the action state to succeeded
	  plan_to_.setSucceeded(plan_result_);



}
