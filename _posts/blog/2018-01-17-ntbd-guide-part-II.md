---
title: "NTBD: a step by step guide II"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-part2.png
headerImage: true
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
 - tutorial
category: blog
author: fiorellazza
description: "What is and how to use NTBD: step by step guide, second article of the series"
---
[> Passa all versione Italiana]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %})

[>> Go back to Post Part I: an overview]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %})

[>> Go to Post Part III: integration with other manipulators]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-III %})

## Part II: tutorial
In this article I will explain how to reproduce the project, complete of robot arm integrated to the NTBD architecture, introduced in post [Part I: an overview]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %}).

### Ingredients
We'll need:
- 1 3D printed EEZYBOT MK2 arm, along with the needed servo-motors (3 mini-servos, 1 micro-servo)
- 1 Arduino board (in my case I have used a Mega ADK 2560)
- 1 PC with Intel processor on which install Docker
 - 1 [Leap Motion](https://www.leapmotion.com/) controller to use one of the implemented robotic applications.
 - 1 external power supply, 2A-7.5V
- 1 Breadboard
- Jumpers to taste

**Nota:** I am currently also working  on an image built for *Raspberry PI 3: the files on github and docker hub are still in development*.

#### Index
- [1. Print the Robot Arm](#print-the-robot-arm) 
- [2. Download the Arduino Sketch
](#download-the-arduino-sketch)
- [3. Download Docker](#download-docker)
- [4. Download the Docker Images](#download-the-docker-images)
- [5. Connections](#connections)
- [6. Run the Docker Container](#run-the-docker-container)
- [7. WebApps for NTBD and siBOT](#webapps-for-ntbd-and-sibot)
- [8.  Play with the WebApps](#play-with-the-webapps)

### 1. Print the Robot Arm
The robot arm which I have chosed is [EEZYbotARM MK2](http://www.eezyrobots.it/eba_mk2.html), an Italian Open-Source project by Carlo Franciscone, Design Engineer e Maker. 

Following the instructions available on [*Thingiverse*](https://www.thingiverse.com/thing:1454048) and [*Instructables*](http://www.instructables.com/id/EEZYbotARM-Mk2-3D-Printed-Robot/) for this project, I successfully completed the 3D printing and mounting of the manipulator.
I have used the [Cura](https://ultimaker.com/en/products/ultimaker-cura-software) software with parameters configured for a [DeltaWASP](http://www.wasproject.it/w/stampa-3d/) printer.
![3d printing](https://drive.google.com/uc?id=1h_PMk-9e0L2nUEC-HEjK5cF2037-N018)
For the DeltaWASP printer, download the following [profile](http://www.personalfab.it/en/downloads-2/download-info/profili-cura/) and load it on Cura following this [guide](https://ultimaker.com/en/resources/52032-manage-profiles).

Following are reported the most significant parameter for printing the neceassary pieces and to improve their definition:

|Parameter       |  Value    | 
|:--------------|:---------------------:| 
| Infill | 30-100%. I'd recommend printing with 30% infill only the robot part which are undergoing few stress whereas the small mechanical parts should be printed with 100% infill. |
| Printing Temperature | 200-210°C|
| Build Plate Temperature| 40°C|
|Filament Diameter| 1.8 mm
| Print Speed | 60 mm/s|
| Infill Speed| 80 mm/s|
| Travel Speed| 150 mm/s|
| Support| Enabled|
|Platform Adhesion| Enabled|

<br>
This is the result:

![eezybot lateral](/assets/imgs/2018-01-17-ntbd/5_eezybotlateral.jpg)

[**<<Back to Index**](#index)

### 2. Download the Arduino Sketch
On the NTBD github page you can find the [myServoControl_ntbd.ino](https://github.com/HotBlackRobotics/ntbd/blob/devel/myServoControl_ntbd.ino) sketch which has to be uploaded on the Arduino board. In my case I have used an Arduino Mega ADK 2560.
![arduino](/assets/imgs/2018-01-17-ntbd/5_mega.jpeg)

[**<<Back to Index**](#index)

### 3. Download Docker
For Ubuntu use this [link](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#uninstall-old-versions).

[**<<Back to Index**](#index)

### 4. Download the Docker Images
At this step, in order to download the NTBD images it is sufficient to execute the following command on the *command line*, so as to retrieve them from [Docker Hub](https://docs.docker.com/docker-hub/), using the [*docker pull*](https://docs.docker.com/engine/reference/commandline/pull/) command:
```
docker pull hbrobotics/ntbd_base:intel
```
```
docker pull hbrobotics/ntbd_manipulator:intel
```
[**<<Back to Index**](#index)

### 5. Connections
As you can see from the sketch, the servomotors, numbered as in Figure, are connected to the Arduino as reported in the table below.
 
 ![arduino](https://drive.google.com/uc?id=14FckU-3X92ctl0lBV2TV1LyZg3t6n894)

| Servo         |  Arduino Pin  | 
|:--------------:|:---------------------:| 
| 1 | 2|
| 2 | 3|
| 3 | 4|
| 4 | 5|

<br>
To make all servomotors mode at the same time without overloading the board, connect it to the external power supply at 7.5 V/ 2 A.![arduino](https://www.modmypi.com/image/data/tutorials/how-to-power-my/6.jpg)
Connect the mini-servos to the breadboard having the V+ connected to the *Vin* Arduino pin and V- connected to one *GND* (ground) Arduino pin of choice.

To avoid overheating the micro-servo, connect its V+ to the 5V Arduino pin and its V- to a GND pin.

**WARNING:** always check that the ground is common to all components (Breadboard GND  = Arduino GND ).

Connect the board and the Leap Motion controller to the PC via USB cable.

![leap](/assets/imgs/2018-01-17-ntbd/5_leapmotion.jpg)

To correctly install the Leap Motion drivers for Ubuntu 16.04, follow this [guide](https://support.leapmotion.com/hc/en-us/articles/223782608-Linux-Installation) along with this slight [edit](https://forums.leapmotion.com/t/tip-ubuntu-systemd-and-leapd/2118).

[**<<Back to Index**](#index)

### 6. Run the Docker Container
Now that all external devices are connected, download the [.yaml file](https://github.com/HotBlackRobotics/ntbd/blob/devel/docker-compose.hbr_ntbd_intel.yml) for the ntbd-manipulator:intel image that is the one we are interested in. So as to boot the container (where the Web applications lie), you have to execute the following command within the folder in which the .yaml file is contained, exploiting the [Docker Compose](https://docs.docker.com/compose/overview/) tool:
```
docker-compose -f docker-compose.hbr_ntbd_intel.yml up
```
[**<<Back to Index**](#index)

### 7. WebApps for NTBD and siBOT
The combination of NTBD with the EEZYBOT arm controlled with ROS is what I have called **siBOT**.
![sibot](/assets/imgs/2018-01-17-ntbd/sibot.png)

Once the container has been run, open a browser page and connect to the loopback address, *localhost*, in order to connect to our computer: indeed the web application server is on our PC. Having called the HTML file "index.html", by connecting to our IP address on the default port 80 (as expected, having mapped the container port 80 to the host port 80), the web address *homepage* will be indeed the WebApp **NTBD-Visualizer**, providing a link to the Leap Motion application.

![webapp](/assets/imgs/2018-01-17-ntbd/5_ntbdviz.png)

[**<<Back to Index**](#index)

### 8.  Play with the WebApps
#### Simulation and control using NTBD - Visualizer Application
Now you are provided with the simulation application from which you can impose a desired end effector position, moving the Cartesian coordinates sliders; clicking on the **Execute** button you will see the arm model moving together with the phisical arm.

**Remark**: the input values inserted by the user through the NTBD - Visualizer WebApp, are used to control both the simulation model and the physical arm.

#### Leap Motion Application
If you want to try out the second developed application, just click on the "*NTBD Leap Motion WebApp*" link which will open a new web page. By keeping this window/tab selected, you can see that positioning your hand above the Leap Motion controller, the space positions are interpreted and sent to the physical robot that will emulate your hand movements. 
![leapspace](/assets/imgs/2018-01-17-ntbd/5_leapref.png)

**Remark**: to open and close the gripper simply open and close your hand!

[**<<Back to Index**](#index)

## END OF PART II

[>> Go back to Post Part I: an overview]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %})

[>> Go to Post Part III: integration with other manipulators]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-III %})
