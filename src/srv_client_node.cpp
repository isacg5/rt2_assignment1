/**
* \file srv_client_node.cpp
* \brief Service client controller
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
*   /goals
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
* This node prints the number of goals reached and cancelled obtained through the service client.
* 
**/

#include "ros/ros.h"
#include "rt2_assignment1/Goals.h"
#include <rt2_assignment1/PlanningAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rt2_assignment1::Goals>("/goals");

    ros::Rate rate(1);

    rt2_assignment1::Goals server_result;

    while (ros::ok())
    {
        // Print the number of goals reached and cancelled obtained through the service /goals
        client.call(server_result);
        ROS_INFO("[REACHED: %i] [CANCELLED: %i]", server_result.response.reached, server_result.response.cancelled);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
