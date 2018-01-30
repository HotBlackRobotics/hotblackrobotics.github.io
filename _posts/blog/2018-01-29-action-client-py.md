---
title: "Sending Goals to the Navigation Stack - Python ROS node version"
layout: post
date: 2018-01-29
image: /assets/imgs/2018-01-29-goal/cover.png
headerImage: true
lang: en
otherlanglink: /2018/01/29/action-client-py-ita/
tag:
 - ROS
 - actionclient
 - action client
 - navigation stack
 - actionlib
 - python ROS
 - rospy
 - actionserver
category: blog
author: fiorellazza
description: "Sending a goal to the ROS navigation stack using a Python node"
---
[> Passa all versione Italiana]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-29-action-client-py-ita %})

Hi there!

This post aim is to provide you with an example Python code for sending a goal pose (desired position and orientation) to a robot, in my case a simulated [TurtleBot3](http://wiki.ros.org/Robots/TurtleBot), exploiting the [ROS Navigation Stack](http://wiki.ros.org/navigation). 
Usually an autonomous mobile robot is tasked to reach a goal location. In order to do so, it must have some informations and combine it: have a map of the environment it is in, perceive its sorroundings, localize itself and plan its movements. The ROS Navigation Stack takes on the role of driving the mobile base to move to that goal pose, avoiding obstacles and combining all the available informations. 
Using code, the user can send to the navigation stack a desired pose for the robot to reach. To write down the node code correctly, I have followed the ["Sending Goals to the Navigation Stack" C++ node tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals), so my aim is to provide an equivalent tutorial for a Python ROS node.

**Remark**: I am using ROS Kinetic. I will assume that the reader has knowledge about ROS [Packages](http://wiki.ros.org/Packages), [Nodes](http://wiki.ros.org/Nodes), [Topics](http://wiki.ros.org/Topics), [Messages](http://wiki.ros.org/msg) and [Actions](http://wiki.ros.org/actionlib#Overview). Some information about the latter will be provided during libraries description.

#### Index:
1. [The actionlib Library](#1-the-actionlib-library)
2. [The MoveBase node](#2-the-movebase-node)
3. [Creating the Node - Code](#3-creating-the-node---code)
4. [Creating the Node - Code and comments](#4-creating-the-node---code-and-comments)

## 1. The ***actionlib*** Library
The ROS navigation stack is based on ROS Actions: indeed Actions are the best choice for cases when a node wants to send a request to another node and will receive a response after a relatively long time. To avoid leaving the user wondering what's happening and if all is going as desired, Actions implement a *feedback* mechanism, which let the user receive information every now and then. Actions are Client-Server-based: the [**actionlib** library](http://wiki.ros.org/actionlib#Client-Server_Interaction) provides the tools and interface to set up an Action Server to execute the requested goals sent by the Client. The main elements of an action mechanisms are: *goal*, *result*, and *feedback*. Each one of them is specified by a ROS Message type, contained in an action definition file, with "*.action*" extension.

For further details see the [actionlib detailed description](http://wiki.ros.org/actionlib/DetailedDescription).

[**<<Back to Index**](#index)
## 2. The ***MoveBase*** node 
The [move_base ROS Node](http://wiki.ros.org/move_base), is a major component of the navigation stack which allows to configure, run and interact with the latter. The move_base node implements a SimpleActionServer, an action server with a single goal policy, taking in goals of [*geometry_msgs/PoseStamped*](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) message type. To communicate with this node, the SimpleActionClient interface is used. The move_base node tries to achieve a desired pose by combining a global and a local motion planners to accomplish a navigation task which includes obstacle avoidance. 
<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/movebase.png" /> 
</p>
<br>
[**<<Back to Index**](#index)
## 3. Creating the Node - Code
Here is the whole node code without comments. For comments see [Section 4](#4-creating-the-node---code-and-comments).

```
#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
```

[**<<Back to Index**](#index)
## 4. Creating the Node - Code and comments
Here is the whole node code with comments. The whole node code without comments is provided in [Section 3](#3-creating-the-node---code).

```
#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 0.5
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
```

[**<<Back to Index**](#index)

We are done! This is a simple Python node to send a pose goal to the navigation stack to move a mobile base. As you can see, for sake of simplicity, being this a basic tutorial, the feedback mechanisms which characterize Actions are not exploited and the result is not a clear indication of the goal actual status. In order to get a further more complete example, I suggest you to read my post ["Sending a sequence of Goals to ROS NavStack with Python"]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-29-seq-goals-py %}).

## Thanks for the attention, See ya! :hibiscus:
