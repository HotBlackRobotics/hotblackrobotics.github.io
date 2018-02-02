---
title: "How to Build ARM Docker Images on Intel host"
redirect_from:
 - /2018/01/22/docker-images-arm/
layout: post
date: 2018-01-22
image: /assets/imgs/2018-01-22-docker-arm/Processor_ARM_anteprima.png
tag:
 - Docker
 - ARM
 - qemu
author: fiorellazza
description: "Build Docker Images for ARM hosts on Intel"
---
![docker_arm_intel](/assets/imgs/2018-01-22-docker-arm/Processor_ARM.png)

Hiya everyone!

Ever wondered how to build ARM [Docker](https://www.docker.com/) Images directly on you PC? I had to struggle finding out how to do so, in order to build up the ARM version for my Docker-based M.Sc. thesis work architecture, NTBD (trilogy post 'bout that [here]()).

The following is a quick guide to how to setup your Intel host to build Docker Images that can be run on an ARM-processor-based host, exploiting [QEMU](https://www.qemu.org/).

**Remark**: in this tutorial I will assume that the reader has knowledge about Docker. For a quick overview about Docker see my post, "[Docker, this stranger!]()". This guide will give advices for building a Docker Image for a **Raspberry Pi** board on a **Ubuntu host**.

### Index:
* TOC
{:toc}

# 1. What is QEMU and QEMU installation
 QEMU (Quick EMUlator) is an Open-Source hosted hypervisor, i.e. an hypervisor running on a OS just as other computer programs, which performs hardware virtualization. QEMU emulates CPUs of several architectures, e.g. x86, PPC, ARM and SPARC. It allows the execution of non-native target executables emulating the native execution and, as we require in this case, the cross-building process.

 Since I am using an Ubuntu host these are the commands to install *qemu*, *qemu-user-static* and *binfmt-support* packages from the command line:

 ``` 
 sudo apt update
 sudo apt install -y qemu qemu-user-static qemu-user binfmt-support
 ```

 Note that package *qemu-user-static* provides statically built emulation QEMU binaries, which do not have dependencies. 

# 2. QEMU and Docker Images
So as to obtain a Docker Image which can be correctly built and run on an ARM host, you need to have a base Image with the target qemu statically linked executable, *qemu-arm-static* in this case. There are plenty ready Images complete of the qemu static binary: 
- [Hypriot rpi-alpine Image](https://hub.docker.com/r/hypriot/rpi-alpine/)
- [Resin rpi-raspbian Image](https://hub.docker.com/r/resin/rpi-raspbian/)
- [Resin raspberry-pi-alpine-node:slim Image](https://hub.docker.com/r/resin/raspberry-pi-alpine-node/). 

However you can also use "simple" Raspberry Images and then copy into the container the binary you can find at */usr/bin/* after having downloaded the qemu packages (see [step 1](#1-what-is-qemu-and-qemu-installation)).

Suppose you are starting from the "official" Ubuntu Docker images for the ARMv7(armhf), available [here](https://hub.docker.com/r/armv7/armhf-ubuntu/), and that you have made a copy of the required binary, *qemu-arm-static*, in your build context (folder containing Dockerfile, i.e. "."). The first lines of your Dockerfile file will be:

```Dockerfile
FROM armv7/armhf-ubuntu:16.04

COPY ./qemu-arm-static /usr/bin/qemu-arm-static
```

In my case, for [NTBD Base Image](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_base/Dockerfile.rpi3), I have used as base Image the [HotBlack Robotics hbrobotics/ros-base:rpi3 image](https://hub.docker.com/r/hbrobotics/ros-base/), which is based on the "Official" ARM Image I mentioned before, with ROS Kinetic installed on it. I then copied on it the QEMU ARM static binary, getting:

```Dockerfile
FROM  hbrobotics/ros-base:rpi3

COPY ./qemu-arm-static /usr/bin/qemu-arm-static
```

Then complete your image Dockerfile with all the needed layers.

# 3. Register QEMU in the build agent
To add QEMU in the build agent there is a specific Docker Image performing what we need, so just run in your command line:
```
docker run --rm --privileged multiarch/qemu-user-static:register --reset
```

# 4. Build the Image
Now you are ready to build your image. Just build with the usual *docker build* command. In my case, having a specific Dockerfile name, the command is:

```bash
docker build -f ./Dockerfile.rpi3 -t ntbd/base:rpi3 .
```

That's it! Your Docker Image should be ready to be pushed on your Docker Hub repository and eventually downloaded and deployed on your Raspberry Pi board.

Thanks, for most the informations, to [Hypriot](https://blog.hypriot.com/post/setup-simple-ci-pipeline-for-arm-images/).
<br>
**See ya!** :hibiscus: