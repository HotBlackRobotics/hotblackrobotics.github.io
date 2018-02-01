---
title: "Docker, questo sconosciuto!"
layout: post
date: 2018-01-18
image: https://logz.io/wp-content/uploads/2016/01/docker-facebook.png
lang: it
otherlanglink: /2018/01/18/docker-this-stranger/
tag:
 - Docker
author: fiorellazza
description: "Perchè utilizzare Docker e la mia esperienza"
---

![docker logo](https://logz.io/wp-content/uploads/2016/01/docker-facebook.png)

[> Switch to the English version]({{ site.baseurl }}{% post_url /en/2018-01-18-docker-this-stranger %})

Ciao a tutti!
Oggi vorrei parlarvi di una tecnologia che sta acquistando sempre più importanza, nel mondo dei developers ed anche in quello aziendale: [Docker](https://www.docker.com).

#### Indice:
 1. [Cos'è un contenitore?](#1-cosè-un-contenitore)<br>
  1.1. [Contenitori Linux vs. Macchine Virtuali](#11-contenitori-linux-vs-macchine-virtuali)<br>
  1.2. [Container Docker](#12-container-docker)
 2. [Concetti chiave per lavorare con Docker](#2-concetti-chiave-per-lavorare-con-docker)<br>
  2.1. [Immagine Docker e Contenitore Docker](#21-immagine-docker-e-contenitore-docker)<br>
  2.2. [Dockerfile](#22-dockerfile)<br>
  2.3.  [Build context, cos'è?](#23-build-context-cosè)<br>
  2.4. [COPY: usare con cautela!](#24-copy-usare-con-cautela)<br>
  2.5. [Docker Compose](#25-docker-compose)
 3. [Comandi utili](#3-comandi-utili)
 4. [Perchè Docker?](#4-perchè-docker)

## 1. Cos'è un contenitore?
Alcuni di voi penseranno "Bhè chiaro! Una scatola, dove mettere qualcosa, per trasportarlo in modo compatto". Vi dirò che questo vostro pensiero ha senso, andiamo a vedere perchè: il concetto di contenitore è apparso per la prima volta con la tecnologia dei Linux Containers [LXC]( https://linuxcontainers.org/it/http://assemble.io), cioè un metodo di virtualizzazione a livello di sistema operativo che permette di  eseguire molteplici  sistemi Linux, chiamati *containers*,  i quali sono isolati e condividono lo stesso Kernel Linux.  Nel 2008 è stata rilasciata la versione 2.6.24 del Kernel Linux, la quale permetteva, per la prima volta, l'isolamento di risorse su hardware condiviso senza il bisogno delle Virtual Machines, il metodo di virtualizzazione più utilizzato fino ad allora.

[**<< Torna all'indice**](#indice)
### 1.1. Contenitori Linux vs. Macchine Virtuali
 - *Virtualizzazione*:  come anticipato, i Linux Containers (LCs) forniscono virtualizzazione a livello di sistema operativo, mentre le Virtual Machines offrono la virtualizzazione dell'hardware.
 - *Guest OS*: i LCs non necessitano di ulteriori layers al di sopra del sistema operativo Host. Invece, le VMs, per poter essere eseguite, richiedono che la copia completa di un sistema operativo Guest venga installata.
La maggior parte degli esempi di Docker Container, per lo sviluppo di applicazioni, sono basati sull'installazione di nuovo software su, per esempio, Ubuntu, il quale non è realmente installato ma è rappresentato da contenuti del Filesystem necessari affinchè l'applicazione possa essere eseguita.
• *Prestazioni e peso*: considerate le osservazioni di cui sopra, i LCs sono leggeri e veloci mentre le VMs presentano un considerevole overhead all'avvio dovuto a tutti gli step che l'avvio di un sistema operativo completo comporta.
• *Hypervisor* : i LCs possono essere eseguiti contemporaneamente e l'isolamento tra le risorse di ognuno è garantinto dalla divisione delle risorse del sistema operativo in gruppi separati. Al contrario, affinchè diverse macchine virtuali possano essere eseguite contemporaneament, è necessario un Hypervisor (conosciuto anche come Virtual Machine Monitor, VMM), ulteriore strato sopra il sistema operativo Host.
Le seguenti immagini riportano le differenze a livello di layers tra i LCs e le VMs.
![dockervsVM1](/assets/imgs/2018-01-18-docker/4_dockerVM1.png)

![dockervsVM2](/assets/imgs/2018-01-18-docker/4_dockerVM2.png)

[**<< Torna all'indice**](#indice)
### 1.2. Container Docker
 I contenitori sono diventati popolari con la nascita di Docker, grazie alla facilità di utilizzo fornita dalla API ad alto livello. Docker permette ai developers di  *impacchettare* ed isolare le proprie applicazioni, favorendo la *modularità* e la *portabilità* di queste ultime. Infatti, il software "*containerizzato*" eseguirà sempre nello stesso modo, indipendentemente dall'ambiente in cui si trova, con l'unico requisito che il sistema operativo Host sia compatibile con Docker. L'unica pecca dei container è che sono *meno sicuri* delle VMs poichè l'isolamento in queste ultime è reale e robusto mentre nei containers l'isolamento può essere violato a causa delle condivisione di risorse. Per questo motivo le applicazioni Cloud e IoT, per adesso, sono containerizzate ed installate su VMs.
 La tecnologia di *containereizzazione* insieme alle procedure standard fornite, definiscono il *Docker Engine*, un'applicazione client-server con i seguenti componenti:

 - Un processo persistente o daemon, chiamato *dockerd*, il quale gestisce containers ed immagini;
 - una API [REST](https://spring.io/understanding/REST) che specifica le interfacce utilizzate dai programmi per comunicare col daemon, per dirgli cosa fare;
 - una interfaccia da linea di comando, usata dall'utente per interagire con il Docker Engine per eseguire e gestire in generale containers ed immagini.

[**<< Torna all'indice**](#indice)
## 2. Concetti chiave per lavorare con Docker
Dopo avervi annoiato con un po' di concetti teorici, passiamo alla parte divertente: qualche pillola utile per utilizzare Docker, lavorarci e capire cosa succede!

### 2.1. Immagine Docker e Contenitore Docker
I concetti di Docker Image e Docker Container, per un nuovo utente, possono essere motivo di confusione: un' *Immagine Docker* è un eseguibile stand-alone che incapsula tutte le risorse neccessarie per eserguirlo, per esempio, codice, librerie, codice runtime, impostazioni e  strumenti di sistema. Un'Immagine Docker che viene eseguita è chiamata *Docker Container*: possono "*runnare*" vari containers basati sulla stessa immagine.

[**<< Torna all'indice**](#indice)
### 2.2. Dockerfile
Una Immagine Docker viene costruita a partire da una "pila" di strati definiti in un file chiamato *Dockerfile*. La tipica Immagine è definita partendo dall'immagine di un sistema operativo di base su cui viene installato software e vengono eseguite operazioni, che possono essere definite utilizzando linguaggio BASH e seguendo un certa [sintassi](https://docs.docker.com/engine/reference/builder/).
Vediamo un esempio breve di Dockerfile:

```Dockerfile
# Pull dell'immagine di base
FROM ubuntu:16.04
SHELL ["/bin/bash","-c"]
# Installazione di software
RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get install ...
#Copia di file dall'Host al Container
COPY /source/path/del/file/locale/ /destination/path/nel/contenitore
# Copia e definizione di un file di operazioni da eseguire all'avvio, i.e., entrypoint
COPY /path/locale/entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

Il file che viene eseguito all'avvio può contenere operazioni di copia dall'Host al container, escuzione di altri script BASH ecc. Nel seguente file di esempio, utilizzato per un contenitore su cui è installato ROS, vengono eseguiti dei file di setup, viene avviato un server [nginx](https://nginx.org/en/) e un launch file ROS.

```Python
!/usr/bin/env bash
set -e
echo "export TERM=xterm" >> ~/.bashrc
# Setup dell'ambiente ROS
source /opt/ros/kinetic/setup.bash
source /catkin_ws/devel/setup.bash

# Avvia nginx
service nginx start

#Launch dei nodi ROS
roslaunch ntbd_core NTBD_launch.launch
exec "$@"
```

Docker fa il *build* delle immagini sfruttando un utilissimo sistema di caching che permette di velocizzare questo processo ogni qualvolta i layer non siano stati modificati.
Per "*buildare*" un'Immagine, bisogna usare il comando:

```bash
 docker build -t nometag .
```

 Questo comando cercherà (di default) il file chiamato Dockerfile nel path specificato, nell'esempio '' . '', ovvero la cartella corrente. E' possibile dare un nome identificativo alla Immagine creata (opzione -t) oppure definire un altro file per il build (opzione  -f, per esempio, docker build -f ./mioDockerfile).

 [**<< Torna all'indice**](#indice)
### 2.3. Build context, cos'è?
Il build context è la cartella contenente il Dockerfile per la creazione di un'Immagine. Quando si deve copiare un file dall'Host al container il path relativo deve riferirsi a questa cartella, per esempio:
```
COPY ./src/file/da/copiare /path/file/nel/container
```

In questo caso il file si trova nella cartella *src* contenuta nella cartella contenente il *Dockerfile*.

[**<< Torna all'indice**](#indice)
### 2.4. COPY: usare con cautela!
Mi raccomando usate COPY nel Dockerfile solo nel momento in cui il file che volete copiare è alla sua versione finale: infatti il comando COPY creerà uno dei "layers" che compone la vostra immagine, quindi, nel caso che il file venisse modificato, il build dell'immagine Docker ripartirebbe da quel layer, senza sfruttare l'uso della cache dell'immagine già "*buildata*".

![docker-rebuild](/assets/imgs/2018-01-18-docker/4_dockerdev.png)

 Quando la vostra applicazione è ancora in fase di sviluppo, il consiglio è quindi quello di eseguire la copia dei file necessari (programmi in development) all'interno del file di entrypoint in modo tale che l'Immagine non venga re-buildata ogni volta che i file cambiano. Ovviamente, essendo eseguito all'avvio del contenitore, il tempo di boot sarà maggiore.

 Per ulteriori informazioni, consultare l'Appendice di questo [post]().

 [**<< Torna all'indice**](#indice)
### 2.5. Docker Compose
[Docker Compose](https://docs.docker.com/compose/overview/) è un tool per definire e *runnare* applicazioni multi-container tramite la configurazione definita in un file [YAML](http://yaml.org/). Trovo, però, che l'utilizzo di questo tool sia molto utile anche solo per eseguire un solo container perchè ti permette di usare un semplice comando, i.e.,
```
docker-compose up
```

il quale estrapola le informazioni di configurazione (mapping di porte, volumi, tag), per default, da un file chiamato *docker-compose.yml* ed esegue il container con tutte le relative opzioni.
Ecco un esempio di un file *docker-compose.yml *:
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

[**<< Torna all'indice**](#indice)
## 3. Comandi utili
Vi lascio alla sperimentazione con Docker con alcuni comandi da command line, utili per la gestione di Immagini e Contenitori:

- Visualizzare i contenitori che sono attualmente eseguiti o *stoppati*:
```
 docker ps -a -q
```

- Visualizzare tutte le Immagini Docker create:
```
 docker images
```

- Fermare tutti i container attualmente eseguiti:
```
 docker stop $(docker ps -a -q)
```

- Rimuovere un container:
```
docker rm ID_container
```

- Rimuovere tutti i container:
```
docker rm $(docker ps -a -q)
```

- Rimuovere un'immagine:
```
docker rmi ID_immagine
```

- Rimuovere tutte le immagini senza tag:
```
docker rmi $(docker images | grep "^<none>" | awk "{print $3}")
```

[**<< Torna all'indice**](#indice)
## 4. Perchè Docker?
Sicuramente Docker ha molti altri vantaggi che scoprirò e scoprirete, ma mi sento di consigliarlo per i seguenti motivi:
  1. **Portabilità**: le vostre applicazioni potranno essere *dockerizzate* ed eseguite su ogni macchina su cui ci sia installato Docker perchè avranno tutto ciò che serve per essere eseguite senza problemi. Un contenitore è proprio una scatola per portare le vostre applicazioni dove volete!

  **Nota**: *un'Immagine buildata con una macchina che ha un certo processore potrà essere eseguita su macchine con lo stesso processore (per esempio, Intel su Intel, ARM su ARM)*.
  2. **Sperimentazione**: a me Docker ha dato la possibilità di provare ad installare o eseguire qualsiasi cosa, senza avere il pensiero di corrompere l'intero sistema. Una volta che il contenitore è eseguito, tutte le modifiche fatte al run-time verranno eliminate allo stop del contenitore stesso, senza lasciare traccia delle modifiche apportate al sistema. Questo è, secondo me, utilissimo anche per chi è alle prese con nuovi sistemi operativi e vuole provare, per capire come funziona!
  3. **Tracking del lavoro fatto**:  il sistema Docker di definire l'immagine uno strato alla volta, permette di avere un file che ci dice tutto su come l'immagine è stata costruita e ci permette di eliminare uno strato nel caso che non ci soddisfi. La consultazione del Dockerfile permette di trovare subito eventuali errori o step fondamentali dimenticati (per esempio, il download di un pacchetto) ed avere un quadro generale dei vari step implementati.

[**<< Torna all'indice**](#indice)

### **Buon Docker a tutti!**
