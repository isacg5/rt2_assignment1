#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import actionlib
from rt2_assignment1.msg import PlanningGoal
import rt2_assignment1.msg

i_x = 0
i_y = 0
cancel_goal = 0
goal_stablished = 0


def get_user_input():
    global cancel_goal, goal_stablished, i_x, i_y

    # Ask the user for the goal position or to cancel the goal
    goal_stablished = 0

    user_x = input(
        "Enter goal introducing 'x' position or enter 'c' to cancel de goal: ")

    # If the argument introduced is a number, ask for the second coordinate
    try:
        i_x = int(user_x)
        user_y = input("Enter goal introducing 'y' position: ")
        goal_stablished = 1

    # If not, check if is a c to cancel the goal
    except:
        if (user_x == "c"):
            cancel_goal = 1
            rospy.loginfo("Cancelling goal...")

    if (goal_stablished == 1):
        goal_stablished = 0
        try:
            i_y = int(user_y)
            goal_stablished = 1
        except:
            pass


def main():
    global cancel_goal, goal_stablished, i_x, i_y

    rospy.init_node("action_node")

    ac = actionlib.SimpleActionClient(
        '/reaching_goal', rt2_assignment1.msg.PlanningAction)

    ac.wait_for_server()  # wait for the action server to start

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        get_user_input()

        # If goal was entered, send the new goal to the action client
        if (goal_stablished == 1):
        
            rospy.loginfo("Action server started, sending goal @[%i, %i].", i_x, i_y)
            #  send a goal to the action
            goal = PlanningGoal()
            goal.target_pose.pose.position.x = i_x
            goal.target_pose.pose.position.y = i_y
            goal.target_pose.pose.position.z = 0
            ac.send_goal(goal)
        
        #  If goal was cancelled, send cancel to the action client
        if (cancel_goal == 1):
            cancel_goal = 0
            ac.cancel_goal()
            rospy.loginfo("Goal has been cancelled")

        rate.sleep()
        # rospy.spin()

if __name__ == "__main__":
    main()
