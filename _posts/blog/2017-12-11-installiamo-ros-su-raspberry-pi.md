---
title: "Installiamo ROS su Raspberry Pi"
layout: post
date: 2017-12-11
image: https://cdn.instructables.com/FPN/U31F/HBNXVG6G/FPNU31FHBNXVG6G.MEDIUM.jpg
headerImage: false
tag:
 - Ros
 - Tutorial
 - Turtlesim
 - Raspberry
 - Robotica
category: blog
redirect_from:
 - /blog/posts/2017-01-13-installiamo-ros-su-raspberry-pi
 - /2017/01/13/installiamo-ros-su-raspberry-pi/
author: ludusrusso
lang: it
description: "Un breve tutorial su come utilizzare ROS sul Raspberry Pi"
---

![ROS+Raspberry Logo](https://cdn.instructables.com/FPN/U31F/HBNXVG6G/FPNU31FHBNXVG6G.MEDIUM.jpg)


Ciao a tutti, torniamo con questo primo tutorial per installare ed utilizzare [ROS](http://www.ros.org/) (the Robot Operating System) su Raspberry Pi.


## ROS: Il Sistema Operativo dei robot

Prima di installarlo ed utilizzarlo, cerchiamo di capire cosa è, quali sono le sue finalità e perchè è diventato un standard di fatto a livello accademico e perchè molte aziende lo stanno iniziando ad adottare.

### Cosa è ROS?

Secondo il sito ufficiale, ROS è un framework per la programmazione Robot orientato alla creazione di Sistemi Robotici distribuiti che interagiscono con l'ambiente umano. È quindi un framework per lo sviluppo di applicazione Robotiche di servizio. È chiamato *OS*, in modo leggermente improprio, perchè offre le stesse funzionalità che normalmente offre un sistema operativo, ma in ambiente multi macchina e multi robot.

### Perchè ROS?

Perchè sviluppare applicazioni robotiche di servizio è difficile! ROS è quindi nato per essere modulare e per favorire la collaborazione tra gruppi di ricerca interessati a temi diversi. Facciamo un esempio del sistema di complessità di un'applicazione robotica reale.

Usando ROS, diveri gruppi di ricerca posso creare moduli sulla loro expertize tecnica, e poi condividere tra loro i module ed integrarli con altri moduli per costruire l'applicazione.

Questa caratteristica ha fatto si che ROS sia diventato in poco tempo uno standard utilizzato da praticamente tutti i gruppi di ricerca accademici che si interessano alla robotica di servizio. E ultimamente anche molte aziende si sono avvicinate a questa tecnologica. In italia citiamo **TIM** che dal 2013 si interessa di queste tematiche.

### ROS e HotBlack Robotics

Noi di **HBRobotics** siamo esperti di ROS, in quanto lo abbiamo utilizzato per anni durante il nostro percorso di Ph.D. al Politecnico di Torino. Quando abbiamo fondato questa società abbiamo deciso di fondare su ROS tutta la nostra tecnologica, e dato che ROS ci ha dato tanto, vogliamo anche contribuire al suo sviluppo e alla sua diffusione anche sul suolo Italiano.

## ROS su Raspberry Pi

Quando è stato presentato il primo **Raspberry PI**, molti utilizzatori di ROS (tra cui il sottoscritto, *maker* da prima di conoscere la parola "*maker*") hanno capito le potenzialità di questo piccolo computer nell'ambito della robotica di servizio. Finalmente esisteva un piccolo computer a bassissimo costo in grado di supportare ROS e quindi di alimentarne la diffusione non solo in ambito accademico e industriale ma anche a livello hobbistico. Purtroppo le prime procedure per l'installazione di ROS su questa macchina erano molto complicate e lente, in quanto era necessario compilare l'intero sistema operativo su Debian, che non supportava ufficialmente. Ricordo ancora la prima volta che riuscii ad installare ROS sul primo Raspberry Pi Model B dopo un mese di tentativi (ammetto che quell'esperienza fu altamente istruttiva in quanto, a fuoria di risolvere errori e dipendenze, imparai tantissimo di Linux).

Ai tempi spuntavano online varie SD già pronte con ROS installato sopra. Il problema era che, ogni volta che serviva installare un nuovo pacchetto ROS, bisognava ricompilare tutto dall'inizio, cosa che dopo un po' diventava infattibile.

Ad ogni modo, i tempi sono cambiati e adesso è molto facile, con qualche trucchetto, installare ROS su un Raspberry Pi senza troppi problemi.

### Cosa Serve?

Ecco i materiali che servono:

- Un Raspberry Pi (consiglio caldamente Raspberry Pi 3 Model B, o almeno il 2 Model B, non ho mai provato questa procedura sul Raspberry Pi 1).
- Una SD con sopra Raspbian. Trovate l'ultima versione [qui](https://www.raspberrypi.org/downloads/raspbian/).
- Un po' di pazienza :D

### Cosa Faremo?

Lo scopo del tutorial è quello di installare ROS e iniziare ad utilizzarlo!


## Installiamo ROS

Accediamo al Raspberry ed apriamo il terminale.

Per prima cosa, è importante aggiornare tutti i pacchetti all'ultima versione, la procedura potrebbe richiedere un po' di tempo, nella mia prova ci ha messo più di 30 minuti per completare l'installazione!

```bash
sudo apt-get update
sudo apt-get upgrade
```


Una volta lanciato, digitiamo i seguenti comandi

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-ros-base
```

Inseriamo la password quando serve e procediamo selezionando sempre `Y` (yes) durante la fasi di installazione.
Anche questa procedura è un po' lenta, dovuta al fatto che sono tantissimi i moduli da installare e la poca potenza del raspberry!

Una volta installato, dobbiamo abilitare ROS all'avvio di ogni shell, in modo da avere tutti i comandi principali attivi. Si fa con il seguente comando

```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

## Utilizzo di TurtleSim

Turtle Sim è un modulo ROS sviluppato per imparare ad utilizzarlo. È in particolare una suite di sperimentazione che permette di controllare tramite ROS una tartaruga virtuale Robotica.

### Installazione di TurtleSim

Per installare il modulo, utilizziamo il comando

```
sudo apt-get install ros-indigo-turtlesim
```

A questo punto siamo pronti a muovere i primi passi. Ma prima di tutto è necessario **Chiudere la Shell**.

### Utilizzo di TurtleSim

A questo punto iniziamo a divertirci. È importante, prima di tutto, dobbiamo aprire un po' di shell contemporaneamente (è la prassi quando si utilizza ROS, quindi abitutevi al disordine).

Nella prima shell, digitiamo il comando

```
roscore
```

che lancia il cuore di ROS, ed è necessario per inizializzare una rete ROS.

![roscore ROS shell](https://raw.githubusercontent.com/ludusrusso/images/master/ros_tutorial/roscore.png)

Dovreste vedere un output del tipo

```shell

... logging to /home/pi/.ros/log/e3e26302-d999-11e6-94cd-b827ebf7d5f2/roslaunch-raspberrypi-10902.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://raspberrypi:45912/
ros_comm version 1.11.20


SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20

NODES

auto-starting new master
process[master]: started with pid [10914]
ROS_MASTER_URI=http://raspberrypi:11311/

setting /run_id to e3e26302-d999-11e6-94cd-b827ebf7d5f2
process[rosout-1]: started with pid [10927]
started core service [/rosout]

```

Lasciate questa shell aperta e, in una seconda shell, lanciamo il simulatore della nostra bellissima tartaruga

```
rosrun turtlesim turtlesim_node
```

che aprirà una finestra grafica in cui viene visualizzata una tartaruga stilizzata in grafica 2D. In realtà la tartaruga è un robot ROS simulato, a cui possiamo mandare comandi di velocità tramite ROS

![ROS TurtleSim Finestra](https://raw.githubusercontent.com/ludusrusso/images/master/ros_tutorial/turtlesim.png)

A questo punto, non ci resta che lanciare un terzo nodo per mandare comandi di velocità alla tartaruga. Questo nodo si lancia (di nuovo su una shell distinta) con

```
rosrun turtlesim turtle_teleop_key
```
![Shell ROS teleop](https://raw.githubusercontent.com/ludusrusso/images/master/ros_tutorial/teleop.png)

Questa volta non usciamo dalla shell ma, lasciandola attiva, premiamo le frecce della tastiera. Se tutto funziona, dovreste vedere la vostra tartaruga muoversi in accordo con i comandi impartiti.

Questa è lo screen sul mio Raspberry Pi quando tutto è in funzione.

![Screen ROS Raspberry Pi](https://raw.githubusercontent.com/ludusrusso/images/master/ros_tutorial/screen_rasp.jpg)

## Conclusioni

In questo tutorial abbiamo visto come installare ed utilizzare ROS su Raspberry Pi. Seguiranno altri tutorial per approfondire l'argomento!
