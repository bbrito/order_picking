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

	add_obstacle = nh_.advertiseService("Add_Obstacle",&ArmPlanner::addcolisionobject,this);
	attach_object = nh_.advertiseService("Attach_Obstacle",&ArmPlanner::attachobject,this);
	detach_object = nh_.advertiseService("Detach_Obstacle",&ArmPlanner::detachobject,this);

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

bool ArmPlanner::addcolisionobject(order_picking::AddColisionObject::Request & req , order_picking::AddColisionObject::Response & res){

	collision_object.header.frame_id = req.CoM.header.frame_id;

	ROS_INFO("Planning frame is %s", collision_object.header.frame_id.c_str());
	/* The id of the object is used to identify it. */
	collision_object.id = req.name_object;

	/* Define a box to add to the world.*/
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = req.dim.x;
	primitive.dimensions[1] = req.dim.y;
	primitive.dimensions[2] = req.dim.z;

	/* A pose for the box (specified relative to frame_id)*/
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = req.CoM.pose.orientation.w;
	box_pose.position.x =  req.CoM.pose.position.x;
	box_pose.position.y = req.CoM.pose.position.y;
	box_pose.position.z =  req.CoM.pose.position.z;
	ROS_INFO("Add an obstacle with dimensions:%f %f %f",primitive.dimensions[0],primitive.dimensions[1],primitive.dimensions[2]);
	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;


	collision_objects.push_back(collision_object);

	ROS_INFO("Add an obstacle into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	res.attached=true;
	sleep(2.0);
	return true;

}

bool ArmPlanner::attachobject(order_picking::AddColisionObject::Request & req , order_picking::AddColisionObject::Response & res){

	object.header.frame_id = req.CoM.header.frame_id;

	ROS_INFO("Planning frame is %s", object.header.frame_id.c_str());
	/* The id of the object is used to identify it. */
	object.id = req.name_object;

	/* Define a box to add to the world.*/
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = req.dim.x;
	primitive.dimensions[1] = req.dim.y;
	primitive.dimensions[2] = req.dim.z;

	/* A pose for the box (specified relative to frame_id)*/
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = req.CoM.pose.orientation.w;
	box_pose.position.x =  req.CoM.pose.position.x;
	box_pose.position.y = req.CoM.pose.position.y;
	box_pose.position.z =  req.CoM.pose.position.z;
	ROS_INFO("Add an object with dimensions:%f %f %f",primitive.dimensions[0],primitive.dimensions[1],primitive.dimensions[2]);
	object.primitives.push_back(primitive);
	object.primitive_poses.push_back(box_pose);
	object.operation = object.ADD;

	attach_objects.clear();
	attach_objects.push_back(object);

	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(attach_objects);

	ROS_INFO("Attach the object to the robot");
	object_ids.push_back(object.id);
	group->attachObject(object.id);
	/* Sleep to give Rviz time to show the object attached (different color). */
	sleep(4.0);

	res.attached=true;
	/* Sleep so we have time to see the object in RViz */
	sleep(2.0);
	return true;

}

bool ArmPlanner::detachobject(order_picking::AddColisionObject::Request & req , order_picking::AddColisionObject::Response & res){

	ROS_INFO("Detach the object from the robot");
	group->detachObject(object.id);
	/* Sleep to give Rviz time to show the object detached. */
	sleep(1.0);

	ROS_INFO("Remove the object from the robot");
	object_ids.clear();
	object_ids.push_back(object.id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	/* Sleep to give Rviz time to show the object is no longer there. */
	sleep(2.0);

	res.attached=true;
	return true;

}

bool ArmPlanner::removeobject(order_picking::RemoveObject::Request & req , order_picking::RemoveObject::Response & res){

	ROS_INFO("Remove the object from the robot");

	remove_object_ids.clear();


	for (int i=0;i<object_ids.size();i++)
	{
		std::size_t found = object_ids[i].find(req.name_object);
		if (found!=std::string::npos)
			remove_object_ids.push_back(object_ids[i]);
	}

	planning_scene_interface.removeCollisionObjects(remove_object_ids);
		/* Sleep to give Rviz time to show the object is no longer there. */
	sleep(2.0);

	res.removed=true;
	return true;

}
