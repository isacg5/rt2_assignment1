/**
* \file srv_server_node.cpp
* \brief Server service for goals 
* \author Isabel Cebollada Gracia
* \version 0.1
* \date 27/02/2023
* 
* \details
* Subscribes to: <BR>
*   /reaching_goal/result
* 
* Publishes to: <BR>
* 	[None]
* 
* Service clients: <BR>
*   [None]
*
* Service servers: <BR>
*   /goals
*
* Action clients: <BR>
*   [None]
*
* Action server: <BR>
*   [None]
*
* Description:
* 
* This node updates the counter of goals completed and cancelled.
* 
**/

#include "ros/ros.h"
#include "rt2_assignment1/Goals.h"
#include <rt2_assignment1/PlanningAction.h>

int counter_reached; ///<counter for the goals tht have been reached
int counter_cancelled; ///<counter for the goals tht have been cancelled

/**
 * \brief A callback that updates the counter of the goals reached and cancelled
 * \param msg Message of Planning.action that contains the information of the goals
*/
void subsCallback(const rt2_assignment1::PlanningActionResult::ConstPtr &msg)
{
    // Update the counter of the goals reached and cancelled by subscribing to /reaching_goal action created
    int result;
    result = msg->status.status;

    if(result == 3){
        counter_reached += 1;
    }
    else if(result == 2){
        counter_cancelled += 1;
    }
}

/**
 * \brief A callback that updates the service server with new values of goals cancelled and reached
 * \param req requests values of the service server
 * \param res response values of the service server
*/
bool goalsCallback(rt2_assignment1::Goals::Request &req, rt2_assignment1::Goals::Response &res)
{
    // Update the service server with the new values
    res.reached = counter_reached;
    res.cancelled = counter_cancelled;

    return true;
}

int main(int argc, char **argv)
{
    counter_reached = 0;
    counter_cancelled = 0;

    ros::init(argc, argv, "goals_server");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, subsCallback);
    ros::ServiceServer service = n.advertiseService("/goals", goalsCallback);
    ros::spin();
    
    return 0;
}