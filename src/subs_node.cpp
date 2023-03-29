/**
* \file subs_node.cpp
* \brief Controller for the euclidean distance
* \author Isabel Cebollada Gracia
* \version 0.1
* \date 27/02/2023
* 
* \param frequency defines the frequency the node will execute.
*
* \details
* Subscribes to: <BR>
*   /robot_info
*   /reaching_goal/goal
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
*   [None]
*
* Action server: <BR>
*   [None]
*
* Description:
* 
* This node calculates the euclidean distance from the robot position to the goal position and prints it onto the terminal. It executes with a frequency defined by the parameter "frequency".
* 
**/

#include "ros/ros.h"
#include "rt2_assignment1/Info.h"
#include <rt2_assignment1/PlanningAction.h>
#include <math.h>

float current_x; ///<x value for current robot position
float current_y; ///<y value for current robot position

float goal_x; ///<x value for goal robot position
float goal_y; ///<y value for goal robot position

float euclidean_distance; ///<value for euclidean distance

/**
 * \brief A callback for the goal position
 * \param msg defines the position of the goal
*/
void goalCallback(const rt2_assignment1::PlanningActionGoal::ConstPtr &msg)
{
    goal_x = msg->goal.target_pose.pose.position.x;
    goal_y = msg->goal.target_pose.pose.position.y;
    ROS_INFO("[goal: %f, %f] ", goal_x, goal_y);
}

/**
 * \brief A callback for the current position of the robot
 * \param msg defines the current position of the robot
*/
void robotInfoCallback(const rt2_assignment1::Info::ConstPtr &msg)
{
    current_x = msg->x;
    current_y = msg->y;
    ROS_INFO("[CURRENT: %f, %f] ", current_x, current_y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subs_node");

    // Get the parameter obtained in the launch file for the frequency of execution of the node
    float frequency;
    ros::param::get("/vel_publishing", frequency);

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/robot_info", 1, robotInfoCallback);
    ros::Subscriber sub_goal = n.subscribe("/reaching_goal/goal", 1, goalCallback);

    goal_x = current_x;
    goal_y = current_y;

    ros::Rate rate(frequency);

    while (ros::ok())
    {
        // Get the euclidean distance between the current position of the robot and the goal, by subscribing to the action clinet and the topic created for the custom message
        int x = goal_x - current_x;
        int y = goal_y - current_y;
        euclidean_distance = sqrt(pow(x, 2) + pow(y, 2));
        ROS_INFO("[DISTANCE: %f] ", euclidean_distance);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
