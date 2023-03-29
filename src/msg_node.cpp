/**
* \file msg_node.cpp
* \brief Message publisher of position and velocities
* \author Isabel Cebollada Gracia
* \version 0.1
* \date 27/02/2023
* 
* \details
* Subscribes to: <BR>
*   /odom
* 
* Publishes to: <BR>
* 	/robot_info
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
* This node publishes the velocity and position values onto the message Info.msg.
* 
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "rt2_assignment1/Info.h"

float x; ///<x value for the current robot position
float y; ///<y value for the current robot position
float vel_x; ///<x value for the robot velocity
float vel_z; ///<z value for the robot velocity

/**
 * \brief A callback that continously get the positions and velocities of the robot
 * \param msg Message from Info.msg
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    vel_x = msg->twist.twist.linear.x;
    vel_z = msg->twist.twist.angular.z;

    ROS_INFO("[Pos x: %f], [Pos y: %f], [Linear vel: %f], [Angular vel: %f]",
             x, y, vel_x, vel_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_node");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1, odomCallback);
    ros::Publisher pub_msg = nh.advertise<rt2_assignment1::Info>("/robot_info", 1);

    ros::Rate rate(1);

    while (ros::ok())
    {
        // Publish the values obtained in the subscription to /odom to a new topic called /robot_info
        rt2_assignment1::Info new_info;
        new_info.x = x;
        new_info.y = y;
        new_info.vel_x = vel_x;
        new_info.vel_z = vel_z;

        pub_msg.publish(new_info);

        ros::spinOnce();
        rate.sleep();
    }
}