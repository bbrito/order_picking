/*
 * order_picking_node.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: bbrito
 */

#include <ros/ros.h>
#include <order_picking/ArmPlanner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");

  ArmPlanner fibonacci(ros::this_node::getName());

  fibonacci.initialize();
  ros::spin();

  return 0;
}



