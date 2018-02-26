---
title: "How to create custom message types in ROS"
layout: post
date: 2018-02-26
image: /assets/imgs/2018-02-26-ros-custom-messages/ros_custom.png
lang: en
tag:
 - ROS

author: fiorellazza
description: "Creating new ROS message types"
---
Hi everyone! Today I will put together informations which I have gathered while I was trying to create a new message type with ROS for my thesis project. 
This may be your situation when you need a basic message type which aims at simplifying your applications: indeed some [ROS standard message types](http://wiki.ros.org/std_msgs/) are way too complex for the simple use you need to fit with.
I hope this post can summarize and speed up the custom message creation operation. Let's get started!

### Index
* TOC
{:toc}

# Creating the .msg file 
I assume you have your catkin workspace (created following [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)) and you have created your ROS package as explained [here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.Creating_a_catkin_Package).

>**RMK**: it is a good habit to create a specific package to define your messages, e.g., create a `custom_msgs` package.

First of all, from command line, enter the package folder exploiting the `roscd` ROS command:

```bash
roscd custom_msgs
```

Once in the package folder, create a new folder called `msg`, such that the custom messages contained in it will be automatically recognized at build time:

```bash
mkdir msg
cd msg
```

Create your new message definition file by directly specifying its content and saving it in a `.msg` file; in my case, I needed to have a simple array of integer data which I have called the `Servo_Array` type.

```bash
echo "uint16[] data" > msg/Servo_Array.msg
```

To check whether your message definition file has been correctly saved you can simply check its content:

```bash
cat Servo_Array.msg
```

# Triggering the message generation
In order to instruct the catkin build operation to generate the newly defined messages, we have to edit the `package.xml` and `CmakeLists.txt` package files as follows:

- Open `package.xml`, and make sure these two lines are in it and **uncomment** them:

```xml
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

- Open `CmakeLists.txt`, add `message_generation` to the list of `COMPONENTS` as follows:

```txt
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

- Export the message runtime dependency:

```txt
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

- Then uncomment the following lines (remove `#`) and replace `Message*.msg` with your .msg file (in my case `Servo_Array.msg`):

```txt
add_message_files(
  FILES
  Servo_Array.msg
)
```
- Finally uncomment these lines:

```txt
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

>**RMK**: if you have more than one custom message to add, just create the relative .msg files and add it when a .msg file has to be added in the `CmakeLists.txt` file (as specified above).
# Re-building your package
Now that we have created some new messages, we need to make our package again:
#In your catkin workspace

```bash
roscd custom_msgs
cd ../..
catkin_make
```

>**RMK**: suppose you are writing Python scripts within a package called, for example, `my_package`: to import the custom message in your script you'll just need the line `from custom_msgs.msg import Motors_Array`. Note that Python scripts are usually contained in a `my_package/scripts` folder.

Most informations in this post have been retrieved from [here](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Common_step_for_msg_and_srv).

# Custom Messages and Rosserial Arduino
In case you need to use your custom message within your serial node on Arduino, you just need to copy your `custom_msgs` package in the `ros_lib` folder (*Arduino_sketches_folder*/libraries/ros_lib/). After re-opening the Arduino editor, you can refer the new message in your sketch with `#include <custom_msgs/Motors_Array.h>`.

**Bye!** :hibiscus: