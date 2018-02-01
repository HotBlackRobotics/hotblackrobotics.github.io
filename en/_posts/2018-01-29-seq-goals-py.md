---
title: "Sending a sequence of Goals to ROS NavStack with Python"
layout: post
date: 2018-01-29
image: /assets/imgs/2018-01-29-goal/cover_seq.png
headerImage: true
lang: en
otherlanglink: /2018/01/29/seq-goals-py-ita/
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
description: "Sending a sequence of desired poses to the ROS navigation stack using a Python node"
---
![cover](/assets/imgs/2018-01-29-goal/coverpost.png)

[> Passa all versione Italiana]({{ site.baseurl }}{% post_url /blog/2018-01-29-seq-goals-py-ita %})

Hi everyone!

If you have read my post, ["Sending Goals to the Navigation Stack - Python ROS node version"]({{ site.baseurl }}{% post_url /blog/2018-01-29-action-client-py %}) you should now be able to send a single goal to you mobile base using a python node. What about sending a sequence of desired poses? In this post I will provide an example code for sending several desired poses (cartesian positions + orientations expressed with quaternions) to the [ROS Navigation Stack](http://wiki.ros.org/navigation). This tutorial is developed choosing the TurtleBot 3 simulated robot as a mobile base, but the Python node is valid for any chosen robot. I will first give some overview about the chosen solution then the code will be explained.

**Remark**: I am using ROS Kinetic. I will assume that the reader has knowledge about ROS [Packages](http://wiki.ros.org/Packages), [Nodes](http://wiki.ros.org/Nodes), [Topics](http://wiki.ros.org/Topics), [Messages](http://wiki.ros.org/msg), [Actions](http://wiki.ros.org/actionlib#Overview) and [ROS Parameters](http://wiki.ros.org/Parameter%20Server). Reading the above cited [post]({{ site.baseurl }}{% post_url /blog/2018-01-29-action-client-py %}) and related ROS documentation is suggested.

#### Index:
1. [Github project and turtlebot3 package download](#1-github-project-and-turtlebot3-package-download)
2. [Goals as ROS parameters](#2-goals-as-ros-parameters)
3. [Launch files](#3-launch-files)
4. [Python Node - Code](#4-python-node---code)
5. [Python Node - Code and comments](#5-python-node---code-and-comments)
6. [Setup and simulation](#6-setup-and-simulation)<br>
    6.1. [Setting the model for your turtlebot](#61-setting-the-model-for-your-turtlebot)<br>
    6.2. [Launching Gazebo and Rviz](#62-launching-gazebo-and-rviz)<br>
    6.3. [Set the current pose of turtlebot](#63-set-the-current-pose-of-turtlebot)<br>
    6.4. [Launching the movebase_seq node and load parameters](#64-launching-the-movebase_seq-node-and-load-parameters)

## 1. Github project and turtlebot3 package download
 In order to work with my example, clone the github project, which you can find [here](https://github.com/FiorellaSibona/turtlebot3_nav), in your preferred location.

 Moreover the ROS turtlebot3 package is needed to run the simulation. For ROS kinetic:

 ```
 sudo apt-get install ros-kinetic-turtlebot3-*
 ```

[**<<Back to Index**](#index)
## 2. Goals as ROS parameters
The idea is to save as ROS parameters the sequence of desired poses we want the Action Server to process and execute on our mobile robot. Once these data are saved on the ROS Parameter Server, they can be easily accessed and successively stuffed in pre-determined Goal ROS messages through the Python node, in order to be correctly interpreted and executed by the Action Server.

Saving the goals as parameters, lets the user to just edit the launch file, in which parameters are set, without any change to the node code.

[**<<Back to Index**](#index)
## 3. Launch files
The [**movebase_seq.launch**](https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/launch/movebase_seq.launch) launch file is very simple and, as anticipated, has the role of setting the desired position and orientation for the mobile base to assume. As you can see, the node, contained in the package called "simple_navigation_goals" and with file name specified in the *type* argument, is launched with some [*private ROS parameters*](http://wiki.ros.org/Parameter%20Server#Private_Parameters) specified within the [<*rosparam*> tag](http://wiki.ros.org/rosparam). Note that private parameters will have to be referenced as *node_name/param_name*.

First the desired sequence of point in the Cartesian coordinate frame is defined. The *p_seq* list, e.g. having *n* points, must be interpreted as follows:
```
p_seq = [x1,y1,z1,x2,y2,z2,...xn,yn,zn] 
```
Afterwards the sequence of desired yaw angles, expressed in degrees, must be specified. Indeed, being the motion of a mobile robot on the xy plane, we can have a orientation variation only around the map Reference Frame z-axis. Of course our robot cannot rotate going inside the floor!
<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/rpy.png" /> 
</p>
<br>
Angles are requested to be specified using degrees to make it simple and will be converted into radians in the node. More details are given in the successive sections.

The [**gazebo_navigation_rviz.launch**](https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/launch/gazebo_navigation_rviz.launch) launch file, sets up and launches the needed nodes for displaying the TurtleBot3 simulation with the map I have retrieved using the mapping provided by the turtlebot3_slam package (which exploits [gmapping](http://wiki.ros.org/gmapping)). 

The launch folder contains also a copy of some files we would launch as they are from the turtlebot3 package, but I needed to make some editing for specifying my map and my rviz configuration (files contained in the [/config](https://github.com/FiorellaSibona/turtlebot3_nav/tree/devel/catkin_ws/src/simple_navigation_goals/config) folder), to provide you with a working and already set-up example.

[**<<Back to Index**](#index)
## 4. Python Node - Code
I've used [this code](https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/nodes/move_base_square.py) as reference.

Here is the whole node code without comments. For comments see [Section 5](#5-python-node---code-and-comments).
```
#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

```

[**<<Back to Index**](#index)
## 5. Python Node - Code and comments
Here is the whole node code with comments. The whole node code without comments is provided in [Section 4](#4-python-node---code).
```
#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
```

- Note that the Python [* operator](https://docs.python.org/2/tutorial/controlflow.html#unpacking-argument-lists) is here used as unpacking operator, i.e. it "extracts" the list values.

- For further information about the *quaternion_from_euler* function, see [here](https://www.lfd.uci.edu/~gohlke/code/transformations.py.html) and [here](https://answers.ros.org/question/53688/euler-angle-convention-in-tf/).

- Note that the Python node has been defined as a class in order to simplify code in future usage.

[**<<Back to Index**](#index)
## 6. Setup and simulation
Now that you have understood everything about my solution (I hope so!), you just have to launch files and you will see the turtlebot moving to the desired poses!
### 6.1. Setting the model for your turtlebot
To avoid having the model error at each turtlebot3 package node launch, I suggest you to run the following command:
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc 
```

[**<<Back to Index**](#index)
### 6.2. Launching Gazebo and Rviz
Always remember to run the setup files for ROS and catkin. 
Then run:
```
roslaunch simple_navigation_goals gazebo_navigation_rviz.launch
```
The [gazebo_navigation_rviz.launch](https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/launch/gazebo_navigation_rviz.launch) launch file brings up the Gazebo and Rviz environments together with the navigation nodes.
<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/6.2.0.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.2.1.png"  height="250" /> 
</p>

[**<<Back to Index**](#index)
### 6.3. Set the current pose of turtlebot
In order to perform all the needed steps for navigating to the goal pose, the turtlebot needs to know (at least more or less) where it is in the map. To do so, in Rviz, press on the **2D Pose Estimate** button, click in the approximate position where the turtlebot is visulized in the Gazebo view and, before releasing the click, set also its orientation.
<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/6.3.2.png"  height="30"/>
</p>
<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/6.2.1.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.3.0.png"  height="250" /> 
</p>
<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/6.2.1.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.3.1.png"  height="250"/> 
</p>

[**<<Back to Index**](#index)
### 6.4. Launching the movebase_seq node and load parameters
In a new terminal run the following command:
```
roslaunch simple_navigation_goals movebase_seq.launch
```

The navigation may take some time but you should be able to see the turtlebot going to the positions and orientations defined in the launch file.
The *green* path is the global planner path while the *blue* one is the path of the local planner which changes frequently, depending on the robot sorroundings perception in time.

<p align="center"> 
    <image src="/assets/imgs/2018-01-29-goal/6.4.0.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.4.1.png"  height="250"/>
</p>

On the terminal you should receive some information about how the current goal execution is proceeding.

[**<<Back to Index**](#index)

You should now have a working example of sending a sequence of poses to the navigation stack on the robot.

## See ya! :hibiscus: