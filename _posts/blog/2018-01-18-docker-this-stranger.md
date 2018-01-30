---
title: "Docker, this stranger!"
layout: post
date: 2018-01-18
image: https://logz.io/wp-content/uploads/2016/01/docker-facebook.png
headerImage: true
lang: en
otherlanglink: /2018/01/18/docker-questo-sconosciuto/
tag:
 - Docker
 - Containers
 - container
 - VM
 - contenitore
 - LC
 - linux container
 - immagine docker
category: blog
author: fiorellazza
description: "Why you should use Docker and my experience"
---
![docker logo](https://logz.io/wp-content/uploads/2016/01/docker-facebook.png)

[> Passa all versione Italiana]({{ site.baseurl }}{% post_url /blog/2018-01-18-docker-questo-sconosciuto %})

Hello everybody!
Today I'd like to talk about a technology that is gathering more and more attention, both in the developers' world and the companies one: [Docker](https://www.docker.com).

#### Index:
1. [What is a container?](#1-what-is-a-container)<br>
	1.1. [Linux Containers vs. Virtual Machines](#11-linux-containers-vs-virtual-machines)<br>
	1.2. [Docker Container](#12-docker-container)
2. [Key concepts for working with Docker](#2-key-concepts-for-working-with-docker)<br>
	2.1. [Docker Image and Docker Container](#21-docker-image-and-docker-container)<br>
	2.2. [Dockerfile](#22-dockerfile)<br>
	2.3.  [What is the Build context?](#23-what-is-the-build-context)<br>
	2.4. [COPY: use with caution!](#24-copy-use-with-caution)<br>
	2.5. [Docker Compose](#25-docker-compose)
3. [Useful Commands](#3-useful-commands)
4. [Why Docker?](#4-why-docker)

## 1. What is a container?
Some of you will think " Well, easy! It is a box, something where to put something else, to move it in a compact way". Well, I'll be honest with you, your reasoning make sense, let's see why: the concept of container first appeared with the Linux Containers [LXC]( https://linuxcontainers.org/it/http://assemble.io) technology, which is an Operating System level virtualization method allowing to execute multiple Linux systems, called, *containers*,  that are isolated and sharing the same Linux Kernel. In 2008, the Linux Kernel 2.6.24 version was released: it permitted, for the very first time, resources isolation upon shared hardware without the need for Virtual Machines, the most used virtualization method at that time.

[**<<Back to Index**](#index)
### 1.1. Linux Containers vs. Virtual Machines
 - *Virtualization*:  as anticipated, Linux Containers (LCs) provide OS level virtualization, whilst VMs offer hardware virtualization.
 - *Guest OS*: LCs don't need further layers upon the host OS; instead, VMs, in order to be execute, require a complete guest OS copy installation. Most of Docker Containers examples, for applications' development, consist in the installation of new software on, e.g., Ubuntu which is not really installed but it is represented by the Filesystem contents necessary for the application execution.
• *Performances and weight*: considering the above observations, LCs are lightweight and fast while VMs present a significant overhead at boot time, due to all the steps that a complete OS boot requires.
• *Hypervisor* : LCs can be executed simultaneously having resources isolation guaranteed by the OS system resources decomposition in separated groups. On the contrary, so that several VMs can run concurrently, an Hypervisor (also known as Virtual Machine Monitor, VMM) is needed, representing a further layer upone the host OS.
The following Figures highlight differences between LCs and VMs at a layer level.

![dockervsVM1](/assets/imgs/2018-01-18-docker/4_dockerVM1.png)

![dockervsVM2](/assets/imgs/2018-01-18-docker/4_dockerVM2.png)

[**<<Back to Index**](#index)
### 1.2. Docker Container
 Containers became popular when Docker came into the picture, thanks to its ease of use guaranteed by the provided high level API. Docker allows developers to *wrap* and isolate their own apps, nurturing their *modularity* and *portability*. Indeed, the "*containerized*" software will always execute in the same way, independently of the environment, with the only requirement of a Docker compliant host Operating System. Containers' weakness is that they are *less safe* than VMs since the latter isolation is robust and real while in containers isolation can be violated due to resources sharing.
 For this reason Cloud and IoT applications are containerized and installed on VMs.
*Containerization* technology, together with some provided standard procedures, make up the *Docker Engine*, a client-server applications with the following components:

 - A persistent process or daemon, called *dockerd*, which manages containers and images;
 - a  [REST](https://spring.io/understanding/REST) API specifying the interfaces used by programs for communicating with and instructing the daemon;
 - a command line interface, used by the user to interact with the Docker Engine in order to execute and manage containers and images.

[**<<Back to Index**](#index)
## 2. Key concepts for working with Docker
After the boring theoretical concept part, let's go to the fun part: some useful nuggets for using Docker, working with it and understanding what's happening!
### 2.1. Docker Image and Docker Container
Docker concepts of Image and Container may be confusing, expecially for newbies: a *Docker Image* is a stand-alone executable incapsulating all needed resources for running it, e.g., pieces of code, libraries, runtime code, settings and system tools. A Docker image which is being run is called a *Docker Container*: it is possible to have several containers, based on the same image.

[**<<Back to Index**](#index)
### 2.2. Dockerfile
A Docker Imange is built from a stack of layers defined in a file called *Dockerfile*. The typical Image is defined starting from an OS Image on which software is installed and operations executed, defined with [specific syntax](https://docs.docker.com/engine/reference/builder/)-BASH commands.
Let's see and example Dockerfile:
```
# Pull base image
FROM ubuntu:16.04
SHELL ["/bin/bash","-c"]
# Install software
RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get install ...
#Copy files from Host to Container
COPY /source/path/del/file/locale/ /destination/path/nel/contenitore
# Copy and definition of a boot-time executed file of operations, i.e., entrypoint
COPY /path/locale/entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

The file executed at boot time can contain copy operations from the Host to the container, execution of other BASH scripts, etc. In the following example file, used for a container with ROS installed, some setup file are executed, an [nginx](https://nginx.org/en/) server and a ROS launch file are launched.
```
!/usr/bin/env bash
set -e
echo "export TERM=xterm" >> ~/.bashrc
# Setup ROS environment
source /opt/ros/kinetic/setup.bash
source /catkin_ws/devel/setup.bash

# Start nginx
service nginx start

# Launch ROS nodes
roslaunch ntbd_core NTBD_launch.launch
exec "$@"
```
Docker performs the Images *build* exploiting a useful caching system which allows to speed up the process when layers are left untouched.

In order to build an Image, the following command must be used:
```
 docker build -t tagname .
```
 This command will try to find, by default, the file called Dockerfile in the specified path, '' . '' in the example, ie. the current folder. It is possible to name as desired the created Image (-t option) or defined another build file(option -f, for example,  docker build -f ./myDockerfile).


[**<<Back to Index**](#index)
### 2.3. What is the Build context?
The build context is the folder containing the Dockerfile. When a file has to be copied from the Host to the container, the realtive path must refer to this folder., e.g.:
```
COPY ./src/file/to/be/copied /path/file/in/the/container
```
In this case, the file is in the *src* folder, in the same folder where the *Dockerfile* lies.

[**<<Back to Index**](#index)

### 2.4. COPY: use with caution!
Guys, be sure to use COPY in your Dockerfile only when files you want to copy are at their final version: indeed the COPY command will add a new layer to the ones making up your Image, thus, in the event that such file is modified, the Docker Image building process would start over from that layer on, without expoliting the already-built Image cache.

![docker-rebuild](/assets/imgs/2018-01-18-docker/4_dockerdev.png)

 When your applications are still in development phase, my advice is to perform the copy operation within the entrypoint file, such that the Image is not re-built over and over again. Obviously, being executed at the container boot, the boot time will increase.

For more information see the Appendix of this [post]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guide-part-III %}).

[**<<Back to Index**](#index)
### 2.5. Docker Compose
[Docker Compose](https://docs.docker.com/compose/overview/) is tool for defining and run multi-containers applications with configuration specified in a [YAML](http://yaml.org/) file. I thingh, though, that this tool is very useful also for single container execution since it permits to use the simple command
```
docker-compose up
```
which gathers configuration information (port mapping, volumes, tags, etc.), by default,  from a file called *docker-compose.yml* and executes the container along with all its options.
Here's an example of *docker-compose.yml * file:
```
service_name:
  image: ntbd/manipulator:intel
  container_name: ntbd_manipulator_intel
  ports:
    - "80:80"
  privileged: true
  devices:
    - "/dev/ttyACM0:/dev/ttyACM0"
  volumes:
    - /tmp/.X11-unix:/tmp/.X11-unix:ro
  environment:
    - DISPLAY=$DISPLAY
```

[**<<Back to Index**](#index)
## 3. Useful Commands
I leave you to exeperimentation with Docker, providing some command line commands, useful for Images and Containers management:

- List all running or stopped containers:
```
 docker ps -a -q
```
- List all created Docker Images:
```
 docker images
```
- Stop all running containers:
```
 docker stop $(docker ps -a -q)
```
- Remove a container:
```
docker rm ID_container
```
- Remove all containers:
```
docker rm $(docker ps -a -q)
```
- Remove an image:
```
docker rmi ID_immagine
```
- Remove all un-tagged Images:
```
docker rmi $(docker images | grep "^<none>" | awk "{print $3}")
```

[**<<Back to Index**](#index)
## 4. Why Docker?
For sure, Docker has multiple advantages which I am currently discovering and which you will discover, but I feel like suggest its usage for the following reasons:
  1. **Portability**: your applications can be *dockerized* and executed on all Docker compliant machines since they are equipped with all they need to run. A container is exactly a box for bringing your applications where you want.
		**Remark**:  *an Image built with a specific processor machine can be run on machines with the same processor (e.g. Intel on Intel, ARM on ARM)*.
2. **Experimentation**: Docker allowed me to try to install or execute whatever was on my mind, without having to worry about the corrupting the entire system. Once the container is executed, all modifications executed at run-time will be deleted at the container stop, without leaving a trace of system modifications. This, in my opinion, is very useful also for who's trying to work with a new OS and what to try in order to understand how's working!
  3. **Developed Work Tracking**:  the Docker system based on layering images, allows to have a file which gives an immediate flavour of the Image building history and to delete a layer when it doesn't satisfy us. Dockerfile consulting enables the user to find errors or missing key step (e.g. a package downloading operation) and have a general overview on the implemented steps.

[**<<Back to Index**](#index)

### **Have a nice Docker!**
