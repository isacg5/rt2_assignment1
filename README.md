ROS
================================

This is the second assignment of Research Track course, based on simple implementations using ROS.


Installing and running
----------------------

[ROS](http://wiki.ros.org/ROS/Installation) must be installed in your computer.

How to run it

After include the repository in your workspace and do catkin_make

```bash
$ roslaunch assignment_2_2022 assignment2.launch vel_pub:=x
```
Where x is the frequency rate that the third node will execute.

Functions in the code
----------------------
To have a better approach to the knowledge of how the code works, the different folders that appear in the repository are deeply explained here.
- action folder: Includes the file to create the custom action.
- msg folder: Includes the file to create the custom message.
- srv folder: Includes the file to create the custom service.
- config, scripts, template, urdf and world are folders that let create th environment.
- launch folder: Contains the launcher that let the user launch all the nodes with one command.
- src folder: Includes all the nodes created to reach the goal of the assigment. Are explained here below:
  * msg_node: Is a node that is part from the first requirement. This node is subscribing to odom to get the position x and y and linear and angular velocities and publishing this information in the customized message through the topic /robot_info.
  * assigment_2_2022_node: Is the other part from the first requirement. Through the user input, gets the goal position of the robot or the to cancel the goal, and through the action created /reaching_goal, updates the action.
  * srv_server_node: Is part of the second requirement og the assignment. By subscribing to the action created in the first requirement, implements a counter with the goals reached or cancelled, and updates the service with this values.
  * srv_client_node: Is the other part of the second requirement. By simply calling the service gets and print the amount of goals reached and cancelled.
  * subs_node: The third part of the assigment is composed by this node. It is subscribing to the topic /robot_info created with the custom message to access to the robot information and subscribing as well to the action created to get the coordinated of the goal. With these values is calculating the euclidean distance that the robot miss until reach the goal. This node also has a paramenter that the user has to introduce when launching the assigment called vel_pub that corresponds to the execution frequency of this node.
  
Flowchart
----------------------
Flowchart is a diagram that shows each step of the progress of a program in a sequential order. In this case, the flowchart present is the one of the first part of the assigment. The one at the left corresponds to msg_node and the one at the right to assigmnet_2_2022_node.

<p align="center">
<img src="https://github.com/isacg5/assignment2_rt/blob/main/resources/flowchart1.png" width="300"/> <img src="https://github.com/isacg5/assignment2_rt/blob/main/resources/flowchart2.png" width="200"/>
</p>

Pseudocode of msg_node
----------------------
```c++
Create subscriber to /odom
Create published to /robot_info (custom message)
while()
  publish_new_vels()
  
Notice that the callback subcriber is getting continuosly the information from /odom. 
```

Pseudocode of assigment_2_2022
----------------------
```c++
Create action client /reaching_goal
Wait for server
while()
  GUI getting a new goal or to cancell a goal
  Send goal or cancel it
```
