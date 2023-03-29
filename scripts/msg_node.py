# /**
# * \file msg_node.cpp
# * \brief Message publisher of position and velocities
# * \author Isabel Cebollada Gracia
# * \version 0.1
# * \date 27/02/2023
# * 
# * \details
# * Subscribes to: <BR>
# *   /odom
# * 
# * Publishes to: <BR>
# * 	/robot_info
# * 
# * Service clients: <BR>
# *   [None]
# *
# * Service servers: <BR>
# *   [None]
# *
# * Action clients: <BR>
# *   [None]
# *
# * Action server: <BR>
# *   [None]
# *
# * Description:
# * 
# * This node publishes the velocity and position values onto the message Info.msg.
# * 
# **/

#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from  assignment_2_2022.msg import Info

x = 0 #///<x value for the current robot position
y = 0#///<y value for the current robot position
vel_x = 0 #///<x value for the robot velocity
vel_z = 0  #///<z value for the robot velocity

# /**
#  * \brief A callback that continously get the positions and velocities of the robot
#  * \param msg Message from Info.msg
# */
def odomCallback(msg):
    global x, y, vel_x, vel_z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_z = msg.twist.twist.angular.z

    rospy.loginfo("[Pos x: %f], [Pos y: %f], [Linear vel: %f], [Angular vel: %f]",
             x, y, vel_x, vel_z)

def main():
    global x, y, vel_x, vel_z
    rospy.init_node("msg_node")

    # ros::NodeHandle nh;
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    pub_msg = rospy.Publisher("/robot_info", Info, queue_size=10)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Publish the values obtained in the subscription to /odom to a new topic called /robot_info
        new_info = Info()
        new_info.x = x
        new_info.y = y
        new_info.vel_x = vel_x
        new_info.vel_z = vel_z

        pub_msg.publish(new_info)

        # rospy.spin()
        rate.sleep()


if __name__ == "__main__":
    main()
