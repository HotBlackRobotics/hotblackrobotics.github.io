---
title: "Come compilare Immagini Docker per ARM su host Intel"
redirect_from:
 - /2018/01/22/docker-immagini-arm/
layout: post
date: 2018-01-22
image: /assets/imgs/2018-01-22-docker-arm/Processor_ARM_anteprima.png
tag:
 - Docker
 - ARM
 - qemu
author: fiorellazza
starred: true
description: "Buildare Immagini Docker per host ARM su Intel"
---
![docker_arm_intel](/assets/imgs/2018-01-22-docker-arm/Processor_ARM.png)

Ciao a tutti!

Mai avuto il dubbio su come compilare Immagini [Docker](https://www.docker.com/) per ARM direttamente sul vostro PC? Io ho dovuto faticare per scoprire come farlo, per buildare la versione ARM dell'architettura sviluppata per il mio lavoro di tesi, NTBD (trilogia di post a proposito [qui]()).

La seguente è una guida veloce su come configurare la vostra macchina Intel per poter compilare Immagini Docker eseguibili su host con processori ARM, sfruttando [QEMU](https://www.qemu.org/).

**Nota**: in questo tutorial assumerò che il lettore conosca Docker. Per una panoramica a proposito di Docker date un'occhiata al mio post, "[Docker, questo sconosciuto!]()". Questa guida darà consigli su come compilare l'Immagine Docker per una **Raspberry Pi** su un **host Ubuntu**.

### Indice:
* TOC
{:toc}

# 1. Cos'è QEMU ed installazione di QEMU
 QEMU (Quick EMUlator) è un hosted hypervisor, cioè un hypervisor eseguito su un sistema operativo esattamente come altri programmi, il quale fornisce virtualizzazione hardware. QEMU emula le CPU di diverse archietture, per esempio x86, PPC, ARM and SPARC. Permette di eseguire eseguibili non nativi, emulando l'escuzione nativa e, come richiesto in questo caso, di eseguire operazioni di cross-building.

 Dal momento che uso un host Ubuntu, questi sono i comandi per installare i pacchetti *qemu*, *qemu-user-static* e *binfmt-support* da linea di comando:

```bash
 sudo apt update
 sudo apt install -y qemu qemu-user-static qemu-user binfmt-support
```

 Il package *qemu-user-static* fornisce eseguibili QEMU buildati staticamente, cioè eseguibili che non hanno dependencies.

# 2. QEMU ed Immagini Docker
Per ottenere un'Immagine Docker che possa essere correttamente *buildata* ed eseguita su un host ARM, è necessario avere un'Immagine base provvista dell'eseguibile qemu richiesto, *qemu-arm-static* in questo caso. Ci sono un po' di Immagini pronte complete di questo eseguibile:
- [Hypriot rpi-alpine Image](https://hub.docker.com/r/hypriot/rpi-alpine/)
- [Resin rpi-raspbian Image](https://hub.docker.com/r/resin/rpi-raspbian/)
- [Resin raspberry-pi-alpine-node:slim Image](https://hub.docker.com/r/resin/raspberry-pi-alpine-node/).

In ogni caso è possibile usare un'Immagine "semplice" per Raspberry e poi copiare nel contenitore il binary che puoi trovare in */usr/bin/*, dopo aver scaricato i pacchetti di QEMU (vedi [step 1](#1-cosè-qemu-ed-installazione-di-qemu)).

Supponiamo che partiate dall'Immagine Docker "ufficiale" per Ubuntu su piattaforme ARMv7(armhf), disponibile [qui](https://hub.docker.com/r/armv7/armhf-ubuntu/), e che abbiate fatto una copia dell'eseguibile di cui abbiamo bisogno, *qemu-arm-static*, nel vostro build context (cartella contenente il Dockerfile, i.e. "."). Le prime righe del vostro Dockerfile saranno:

```Dockerfile
FROM armv7/armhf-ubuntu:16.04

COPY ./qemu-arm-static /usr/bin/qemu-arm-static
```

Nel mio caso, per l'[Immagine Base di NTBD](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_base/Dockerfile.rpi3), ho usato come immagine di partenza l'[Immagine HotBlack Robotics hbrobotics/ros-base:rpi3](https://hub.docker.com/r/hbrobotics/ros-base/), basata sull'Immagine "ufficiale" per ARM citata sopra, con installato ROS Kinetic. Ho quindi copiato il binary ARM di QEMU, ottenendo:

```Dockerfile
FROM  hbrobotics/ros-base:rpi3

COPY ./qemu-arm-static /usr/bin/qemu-arm-static
```

Quindi completate il vostro Dockerfile con tutti i layers necessari.

# 3. Registrare QEMU sul build agent
Per registrare QEMU sul build agent, c'è un'Immagine Docker apposita, quindi eseguite semplicemente il seguente comando nella command line:

```bash
docker run --rm --privileged multiarch/qemu-user-static:register --reset
```

# 4. Build dell'Immagine
Adesso siete pronti per *buildare* la vostra immagine. Usate il solito comando *docker build*. Nel mio caso, avendo un nome specifico per il Dockerfile, il comando è:

```bash
docker build -f ./Dockerfile.rpi3 -t ntbd/base:rpi3 .
```

Ecco fatto! La vostra Immagine Docker dovrebbe essere pronta per essere *pushata* sulla vostra repository Docker Hub e alla fine scaricata e caricata sulla vostra Raspberry Pi.

Grazie, per la maggior parte delle informazioni, a [Hypriot](https://blog.hypriot.com/post/setup-simple-ci-pipeline-for-arm-images/).
<br>
**A presto!** :hibiscus: