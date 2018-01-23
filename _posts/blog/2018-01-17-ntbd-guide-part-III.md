---
title: "NTBD: a step by step guide III"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-part3.png
headerImage: true
lang: en
otherlanglink: /2018/01/17/ntbd-guida-parte-III/
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
 - manipulator
category: blog
author: fiorellazza
description: "What is and how to use NTBD: step by step guide, third article of the series"
---
[> Passa all versione Italiana]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-III %})

[>> Go back to Post Part I: an overview]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %})

[>> Go back to Post Part II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-II %})

## Parte III: integration with other manipulators
In order to adapt the NTBD architecture to a another manipulator, different from EEZYBOT, it is necessary to re-define the architecture abstract components. Furthermore I assume that the control platform is an Arduino board.
The abstract components are implemented in the *ntbd_manipulator* Docker Image and, as explained in post ["Part I: an overview"]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %}), they depend on the chosen robot arm and thus require editing according to the new structure.
**Remarks**:
- I suggest reading posts [Part I]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %}) and [Part II]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-II %}) in order to understand what has to be modified to integrate your manipulator with NTBD and how to connect external devices for the control and the WebApps.
- To build the Docker Images it is necessary to [install Docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#uninstall-old-versions). In this tutorial I assume that the reader has knowledge about developing apps with Docker. For some pieces of advice, read the [Appendix](#appendix-docker-prod-vs-docker-devel).

### Index
1. [Edit the NTBD abstract components](#1-edit-the-ntbd-abstract-components)
2. [Building the new Image](#2-building-the-new-image)
3. [ Run the new Container](#3-run-the-new-container)
4. [Appendix: Docker prod vs Docker devel](#appendix-docker-prod-vs-docker-devel)

### 1. Edit the NTBD abstract components
It is necessary to download the [NTBD project from github](https://github.com/HotBlackRobotics/ntbd) so as to edit the following files before proceeding with the build of the new *ntbd_manipulator* Image:

- [*joint_names_sibot_urdf.yaml*](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/config/joint_names_sibot_urdf.yaml): in this file are defined the robot joints names, useful for ROS messages exchange. This definition is used, for example, in node [physical_2_visual](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/physical_2_visual), which must be modified accordingly.
- [*/meshes/*](https://github.com/HotBlackRobotics/ntbd/tree/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/meshes): this folder containes the [meshes](https://it.wikipedia.org/wiki/Mesh_poligonale) needed for the manipulator visualization, in [*STL*](https://it.wikipedia.org/wiki/STL_(formato_di_file) format.
 - [siBOT_noEE.urdf](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/urdf/siBOT_noEE.urdf): this file contains the [URDF](http://sdk.rethinkrobotics.com/wiki/URDF) definition for the new robot armd; it can thus be renamed with the only precaution of changing the file name in the launch file [NTBD_launch.launch](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/launch/NTBD_launch.launch).
 - [index.html](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/web/index.html) and [ntbd_lm.html](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/web/ntbd_lm.html) define the Web Applications and must be modified depending on the new configuration (e.g., with new limits).
 - [IK](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/IK), [FK](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/FK), [motor_values](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/motors_values) and [physical_2_visual](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/physical_2_visual): these files are all manipulator dependent; for further information about their role, [>> go to Post Part I: an overview]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %}).
- [*NTBD_launch.launch*](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/launch/NTBD_launch.launch): this file must be edited to update the [end effector position coordinates limits](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/launch/NTBD_launch.launch#L16) and [motors limits](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/launch/NTBD_launch.launch#L24).
- [myServoControl_ntbd.ino](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/myServoControl_ntbd.ino): obviously, changing the manipulator, also the physical configuration will most likely change (number and type of motors) and consequently the Arduino sketch must be edited.

**Remark**:
- the ROS package [*manipulator_urdf*](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf), which contains all the info needed for using URDF definition, can be automatically generated starting from the robot assembly file, exploiting the [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly) tool.
- In the event that a file name must be modified, it must be taken into account that such file could be "called" within another file and the latter should be edited accordingly. My advice is to *avoid re-naming file* in order to prevent errors.

[**<<Back to Index**](#index)

### 2. Building the new Image
Once the necessary files have been mofidied, it's time to build the new Docker Image. Enter into the [NTBD_manipulator](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator) folder in which we have the [Dockerfile.intel](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/Dockerfile.intel) file, and execute the following command.
```
docker build -t ntbd/manipulator:intel .
```

[**<<Back to Index**](#index)

### 3. Run the new Container
After connecting all external devices, run the container executing:
```
docker-compose -f docker-compose.intel.yml up
```
which exploits the [Docker Compose](https://docs.docker.com/compose/overview/) tool with configuration specified in the [docker-compose.intel.yml](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/docker-compose.intel.yml) file.

The NTBD integration with a new robot arm ends here, for further information about the WebApp usage, see [Post Part II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-II %}).

[**<<Back to Index**](#index)

### Appendix: Docker prod vs Docker devel
When working with Docker, it is convenient, for the sake of organization, to use a Development Image and a Production Image. The latter is the final version of the Docker application, ready for release, whilst the former is used during the development phase. A common Image is built on which the Prod and Dev versions are generated and, with some tricks, it is possibile to harness the *caching * mechanism provided during the building stage. Indeed, if the edited files are still in debug phase, at each Image build, the layers (even if previously built up) will be re-built.

![docker](/assets/imgs/2018-01-17-ntbd/4_dockerdev.png)

It is thus suggested to have all files still in dev stage in a single folder, shared among the host system and the container: through a bash script, executed at boot time in the container, the files are copied into the container. This clearly increases the boot time but also avoids the re-building issue. The Prod Image, which for NTBD is the one [provided on Docker Hub](https://hub.docker.com/r/hbrobotics/ntbd_manipulator/), copies the definitive files from the *building context* (the folder containing all necessary resources for the container execution) to the container file system, since now all files should be at their final version.

The following is a building context example:

![buildingcontext](/assets/imgs/2018-01-17-ntbd/building-context.png)

Here I provide the NTBD_devel_entrypoint.sh bash file content, used for my development version:
```
#!/usr/bin/env bash
set -e
echo "export TERM=xterm" >> ~/.bashrc
echo ". /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc
# Source form host:
# NTBD_core scripts & launch
cp /src/IK /catkin_ws/src/ntbd_core/scripts/IK
cp /src/physical_2_visual /catkin_ws/src/ntbd_core/scripts/physical_2_visual
cp /src/FK /catkin_ws/src/ntbd_core/scripts/FK
cp /src/motors_values /catkin_ws/src/ntbd_core/scripts/motors_values
# Make scripts executable to be used as nodes!
cd /catkin_ws/src/ntbd_core/scripts/
chmod +x IK && chmod +x physical_2_visual && chmod +x FK && chmod +x motors_values

cp /src/NTBD_launch.launch /catkin_ws/src/ntbd_core/launch/NTBD_launch.launch

# NTBD_urdf
cp -rf /src/manipulator_urdf/ /catkin_ws/src/manipulator_urdf/
# setup ros3djs config (comprehends nginx config)
cp -rf /src/manipulator_urdf/ /var/www/html/manipulator_urdf/
cp /src/NTBD_viz.html /var/www/html/NTBD_viz.html

cp /src/ntbd_lm.html /var/www/html/ntbd_lm.html

cp -rf /src/ros3djs/roswebconsole/ /var/www/html/roswebconsole/

source /catkin_ws/devel/setup.bash
cd /catkin_ws/ && catkin_make

# setup ros environment
 source /opt/ros/kinetic/setup.bash
 source /catkin_ws/devel/setup.bash

# start nginx
service nginx start

# Launch my ROS nodes and ros3djs URDF visualization
roslaunch ntbd_core NTBD_launch.launch

exec "$@"
```
[**<<Back to Index**](#index)

## END OF NTBD TRILOGY

[>> Go back to Post Part I: an overview]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %})

[>> Go back to Post Part II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-II %})
