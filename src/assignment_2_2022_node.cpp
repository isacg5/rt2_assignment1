/**
* \file assignment_2_2022_node.cpp
* \brief Action client to that controls a cancelled or new goal 
* \author Isabel Cebollada Gracia
* \version 0.1
* \date 27/02/2023
* 
* \details
* Subscribes to: <BR>
*   [None]
* 
* Publishes to: <BR>
* 	[None]
* 
* Service clients: <BR>
*   [None]
*
* Service servers: <BR>
*   [None]
*
* Action clients: <BR>
*   /reaching_goal
*
* Action servers: <BR>
*   [None]
*
* Description:
* 
* This node cancels or sends a new goal depending on the user input.
* 
**/

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PlanningAction.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;

int i_x; ///<x value for the goal desired by the user
int i_y; ///<y value for the goal desired by the user
int cancel_goal = 0; ///<value to control if the goal has to be cancelled
int goal_stablished = 0; ///<value to control if the goal has to be stablished


/**
 * \brief A function that asks the used for a new goal for the robot or to cancel it
*/
void get_user_input()
{
	// Ask the user for the goal position or to cancel the goal
	goal_stablished = 0;
	string user_x;
	string user_y;
	cout << "Enter goal introducing 'x' position or enter 'c' to cancel de goal: ";
	cin >> user_x;

	// If the argument introduced is a number, ask for the second coordinate
	try
	{
		i_x = stoi(user_x);
		cout << "Enter goal introducing 'y' position: ";
		goal_stablished = 1;
	}
	// If not, check if is a c to cancel the goal
	catch (invalid_argument const &e)
	{
		if ((user_x.compare("c")) == 0)
		{
			cancel_goal = 1;
			ROS_INFO("Cancelling goal...");
		}
	}

	if (goal_stablished == 1)
	{
		cin >> user_y;
		goal_stablished = 0;
		try
		{
			i_y = stoi(user_y);
			goal_stablished = 1;
		}
		catch (invalid_argument const &e)
		{
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_node");

	actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/reaching_goal", true);

	ac.waitForServer(); // wait for the action server to start

	ros::Rate rate(1);

	while (ros::ok())
	{
		get_user_input();

		// If goal was entered, send the new goal to the action client
		if (goal_stablished == 1)
		{
			ROS_INFO("Action server started, sending goal (%i,%i).", i_x, i_y);
			// send a goal to the action
			rt2_assignment1::PlanningGoal goal;
			goal.target_pose.pose.position.x = i_x;
			goal.target_pose.pose.position.y = i_y;
			goal.target_pose.pose.position.z = 0;
			ac.sendGoal(goal);
		}

		// If goal was cancelled, send cancel to the action client
		if (cancel_goal == 1)
		{
			cancel_goal = 0;
			ac.cancelGoal();
			ROS_INFO("Goal has been cancelled");
		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
