---
title: "NTBD: a step by step guide I"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-part1.png
headerImage: true
lang: en
tag:
 - Robotics
 - NTBD
 - Containers
 - container
 - docker
 - ROS
 - nginx
 - 3D printing
 - 3D
 - stampa 3D
category: blog
author: fiorellazza
description: "What is and how to use NTBD: step by step guide, first article of the series"
---
[> Passa all versione Italiana]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-I %})

[>> Go to Post Part II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-II %})

[>>Go to Post Part III: integration with other manipulators]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-III %})

## Part I: an overview
Hi everyone! It's me again, Fiorella and, with this post *trilogy*, I'd like to present my Master Thesis work for the M.Sc. in Mechatronic Engineering degree at Politecnico di Torino, "*Development of a Standard Architecture
to enable Fast Software Prototyping
for Robot Arms*",  and provide a step by step guide as clearly as possible (hopefully!) for using and re-using the developed architecture, called NTBD.
In this first part I present the proposed architecture as you will find it browsing the [github project](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c).  In the second post, instead, there will be a tutorial on how to use the architecture integrated with the chosen Open-Source robot arm, having a PC with Intel processor (or an ARM-processor host). Finally in the third article I'll explain how to exploit NTBD using another manipulator different from the one I've presented, highlighting the limits.

#### Index:
 1. [Motivations](#motivations)
 2. [NTBD: Name To Be Decided](#ntbd-name-to-be-decided)
 3. [NTBD - Conceptual Level](#ntbd---conceptual-level)
 4. [NTBD - Docker Level](#ntbd---docker-level)
 5. [NTBD - ROS Level](#ntbd---ros-level)

### Motivations
The idea of my thesis came up noting that, when it comes to rapid prototyping, hardware has always has plenty valid choices, for example the Open-Source board [Arduino](http://www.arduino.org/) and [RaspBerry Pi](https://www.raspberrypi.org/); for what concerns Robotics, there are countless Open-Source projects for robot contruction, equipped with instructions, advices about the control hardware to be used, necessary pieces (which may be 3D printed) and [BOM](https://en.wikipedia.org/wiki/Bill_of_materials) lists. For what regards the software, however, there isn't a standard architecture upon which build your robotic application easily. Here comes into the picture **NTBD**, with the aim of developing robot arm applications.

[**<<Back to Index**](#index)

### NTBD: Name To Be Decided
Well, yes, here it is the so-longly-searched name in all its glory! After afternoon-sessions trying to find a cool and catchy name, that's the result...
<!-- gif*Not so impressive*-->
![enter image description here](https://media.giphy.com/media/rwedxv8kWXBaU/giphy.gif)

Anyway, I think that the most important part is content. Talking about content, I'll now give a brief top-down overview of NTBD.

[**<<Back to Index**](#index)

### NTBD - Conceptual Level
![ntbd-conceptual](/assets/imgs/2018-01-17-ntbd/4_architect.png)

The architecture is composed of several elements among which we have abstract components, that are generic user-implemented components . I subsequently list the role that each abstract component takes on:
 - **IK & FK**: they implement the inverse kinematics and forward kinematics for the manipulator end effector.
 - **URDF**: it is a file ([Universal Robot Description Format](http://wiki.ros.org/urdf)) which describes the kinematic structure of the arm in order to define its simulation model.
 - **P2V**: here the joint angles are converted to obtain the corresponding values to move the simulation model accordingly.
 - **Motor Values**: in this element the joints values and the gripper value (if present) are put together into one single array.
 - **HW**: this component represents the used hardware for the manipulator control, in this case an Arduino board.

 The following are the robot arm structure-independent components:
 - **Position Limiter**:  this element has the role of limiting the desired position determined by the lower and upper limits of the Cartesian coordinates x,y,z.
 - **Path Planner**: I have chosen, for the sake of simplicity, to plan the end effector path in a linear fashion, assuming that no obstacles are present.
 - **Values Limiter**: also the motor values must be limited, so as to avoid damages and unexpected behaviour.
 - **Rosserial**: thi element represents the importance of the [Rosserial](http://wiki.ros.org/rosserial) tool, needed to bridge a serial connection with the rapid prototyping control platform.
 - **Rosbridge**: this component represents the ROS tool [Rosbridge](http://wiki.ros.org/rosbridge_suite), which allow our manipulator simulation through the integration of the ROS system with the WEeb, using a JavaScript library, [roslibjs](http://wiki.ros.org/roslibjs).

[**<<Back to Index**](#index)

### NTBD - Docker Level
The architecture is portable thanks to its definition in a [Docker](https://www.docker.com/) container, which guarantees its execution always in the same way, with the only requirement of a Docker compliant system. Those who already have some basic knowledge of Docker, surely know that a Docker application is developed starting from a base image which usually is an Operating System image. In this case, the starting image is Ubuntu 16.04 with ROS Kinetic Kame (10th ROS distribution, 2016) installed on it, gathered from the [HotBlack Robotics github](https://github.com/HotBlackRobotics/docker-ros-base). As can be seen from the [NTBD repository](https://hub.docker.com/u/hbrobotics/) on[Docker Hub](https://docs.docker.com/docker-hub/), there are two *layers* which make up the architecture, *base* and *manipulator*. The first one contains the architecture's elements which are commont to all robotic  arms (Position Limiter, Path Planner, Values Limiter, Rosserial e Rosbridge); upon this layer is built the second image in which the abstract components are implemented.

[**<<Back to Index**](#index)

### NTBD - ROS level
The components previously presented have been implemented in [ROS](http://www.ros.org/), Robot Operating System, as Python nodes. I assume that the reader knows the basic dynamics of ROS framework, in order to avoid expatiating in further explanations and proceed to the listing and relative explanation of this architecture's [nodes](http://wiki.ros.org/Nodes) and [topics](http://wiki.ros.org/Topics).

In the Figure we can see nodes and topics involved in NTBD dynamics.
![ntbd-ros](/assets/imgs/2018-01-17-ntbd/4_archrosgraph.png)
The following table presents topics and relative [ROS messages](http://wiki.ros.org/Messages).

**Note**:
- Motors_Array is a custom ROS message type, created to enhance the message content and have a simple structure.
- the */girpper_value* message tipe depends on the user implementation.

| Topic         |  ROS Message Type     |
|:--------------|:---------------------|
| /desired_position_nolim | [geometry_msgs/Point](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html)|
| /desired_position_nointerp |geometry_msgs/Point|
| /desired_position | geometry\_msgs/Point|
| /motors_nogripper | Motors\_Array|
| /gripper_value | [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)|
| /motors_nolim | Motors_Array|
| /motors | Motors_Array|
| /actual\_position | geometry\_msgs/Point|
| /joint\_states | [sensor\_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)|

<br>
 - Topic **desired_position_nolim**: the desired position is published on this topic, as a Point message.
 - Nodo  **/position_limiter**: here the values which lie outside the limits (defined as [ROS parameters](http://wiki.ros.org/Parameter%20Server#Parameters)), are saturated such that errors due to unreachable positions are avoided.
 - Topic **/desired_position_nointerp**: the positions, "filtered" by node */position_limiter*, are oublished on this topic.
 - Nodo  **/path_planner**: the aim of this node is to compute the linear interpolation  between two consecutive positions published on */desired_position_nointerp*.
 - Topic **/desired_position**: here the sequence of space points computed by the path planning node is published.
 - Nodo **/IK**: the desired position is converted in motor values (angles, in the case of servo-motors).
 - Topic **/motors_nogripper**: the values obtained in */IK* are made available here.
 - Topic **/gripper_value**: provides in output the desired value for the robot arm gripper, in this case the gripper can be open or closed.
 - Nodo **/motors\_values**: here the motors and gripper values are combined in a unique data vector.
 - Topic **/motors\_nolim**: the set of all desired values is published in this topic.
 - Nodo **/motors\_limiter**: this node reads the motors values and, if necessary, it limits them by comparison with the ROS parameters- defined limit values. This node is crucial in the case that the user wishes to controle the robor in the joint space directly.
 -  Topic **/motors**: the limited values are finally available in this topic.
 - Nodo Rosserial **/init\_serial\_node**: this node, which corresponds to the [*/serial_node*](http://wiki.ros.org/rosserial_python#serial_node.py) node, permits to establish a connection between the serial node loaded on the hardware platform (Arduino in this case) and the ROS system running on the PC. Indeed the motors values are read by the specific topic and processed by the board in order to move the servos.
 - Nodo **/FK**: motors values can also be used for computing the forward kinematics, leading in fact to the actual reached position, which can be differnt from the desired one, due to physical limits.
 - Topic **/actual\_position**: the reached position is published here.

For what concerns the robot simulation on a web page, I had to make available in a server the robot arm URDF file and the HTML file for the application. The adopted solution consisted in bringing up a server, within the containerized environment, using [*nginx*](https://www.nginx.com/resources/glossary/nginx/), an Open-Source web server which can be used to provide on the network HTTP contents. In order to display the simulation on the host machine browser, the port 80 of the container has been mapped to a host port of choice, e.g. 1234 (in the final versione, port 80 is mapped on the host standard port i.e. port 80).
The nginx default root folder (/var/www/html) contains the HTML document, where the Rosbridge and ROS interactions happen and the where the URDF necessary for rendering is available.
![ntbd-simulation](/assets/imgs/2018-01-17-ntbd/4_sim.png)

Going back to the ROS level description, the following are nodes and topics involved in the robot arm simulation:

 -  Nodo **/physical_2_visual** :  this node is one of the abstract components, thus dependent on the manipulator choice. It is a conversion node which maps the physical values, provided by topic */motors* to the equivalent visualization values.
 - Topic **/joint\_states**: here the visualization joint values are published.

From here on, the topic message management and nodes are the typical ones for ROS dealing with 3D data: the joint values are passed to the [**/robot\_state\_publisher**](http://wiki.ros.org/robot_state_publisher) topic, which gives in output messages of [*geometry_msgs/Transform*](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Transform.html) type, exploiting the [*tf2*](http://wiki.ros.org/tf2) ROS package, that enables the user to track the coordinate systems in time. These information are then discoled to Rosbridge; since nodes and topics involved in this part are pretty standard, I won't explain it further.
![graph-simulation](/assets/imgs/2018-01-17-ntbd/5_rosbr.png)

Connecting to "ourselves" from the browser, on the defined port, we can see the arm model moving together with the physical manipulator. The Rosbridge connection is initiated on the default port 9090 on which another computer can interact with the ROS system by publishing and subscribing on its topics.

[**<<Back to Index**](#index)

## END OF PART I

[>> Go to Post Part II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-II %})

[>>Go to Post Part III: integration with other manipulators]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-III %})
