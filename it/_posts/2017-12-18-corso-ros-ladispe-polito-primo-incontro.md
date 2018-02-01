---
title: "Laboratori ROS LADISPE Politecnico di Torino. Primo Incontro"
layout: post
date: 2017-12-15
image: /assets/imgs/2017-12-15-corso-ros-ladispe-polito.md/lab.png
headerImage: false
tag:
 - ROS
 - Corso
 - Robotica
 - Politecnico di Torino
category: blog
author: ludusrusso
lang: it
description: "Il primo incontro del laboratorio ci permetterà di iniziare a prendere confidenza con ROS"
---

Benvenuti in questo tutorial, su cui sarà basato il primo incontro dei
[Laboratori di ROS presso LADISPE del Politecnico di Torino](http://www.ladispe.polito.it/flatpages/laboratorio).

![Lab](/assets/imgs/2017-12-15-corso-ros-ladispe-polito.md/lab.png)

Lo scopo del primo incontro è dare ai partecipanti la possibilità di prendere
confidenza con le basi di ROS e con Linux, sistema operativo su cui ROS è
stato costruito e mantenuto.

## Alcune informazioni su ROS

Prima di iniziare il tutorial, vorrei fare alcuni chiarimenti su cosa è (e cosa non è)
ROS e su quali sono le sue finalità.

Nonostante il nome (*Robot Operating System*) ROS è un **framework** per lo sviluppo
di *applicazioni* robotiche. È nato nel 2007 sulla necessità di evitare che ogni
laboratorio di ricerca sulla robotica di servizio dovesse costruirsi il proprio
set di funzioni base su cui poi costruire le proprie applicazioni. È stato quindi
rilasciato con licenza OpenSource ed è completamente aperto e modificabile.

Data la grossa necessità a cui questo rispondeva, è stato adottato in modo massiccio
in pochi anni da quasi la totalità dei centri di ricerca (pubblici e privati) sulla
Robotica nel mondo, tanto che al momento è considerato uno *standard di fatto* per
la prototipazione e lo sviluppo di applicazioni robotiche di servizio.

ROS è stato sviluppato in modo da essere fortemente estendibile, dando la possibilità
ai vari laboratori di ricerca di contribuire rilasciando il loro codice all'interno
di pacchetti ROS. In questo modo, all'interno di ROS è facile trovare tutti gli algoritmi
allo stato dell'arte per abilitare il robot a risolvere compiti complessi e standard (come la navigazione
autonoma), rendendo molto più semplice concentrarsi sull'applicazioni ad alto livello e
non sui problemi matematici.

Ovviamente ROS ha anche alcuni problemi, i principali sono, a mio avviso, i seguenti:

1. ROS nasce 10 anni fa avendo come obiettivo principalmente i laboratori di ricerca, un use case che al tempo prevedeva un robot, un server su cui fare le computazioni più pesanti e una connessione locale tra il robot e il server. Ovviamente, questo caso d'uso è superato e la ricerca è andata avanti verso tecnologie più evolute, come la *Cloud Robotics* o la *Collaborazione Multi robot*. In questi casi, ROS pone grosse limitazioni architetturali.
Molti progetti ([RoCon](http://wiki.ros.org/rocon), [Rapyuta](https://www.rapyuta-robotics.com/)) cercano di mettere una pezza a questi limiti di ROS, ma si sta anche cercando di [riprogettarlo dalle fondamenta](http://design.ros2.org/) per risolvere questi problemi.
2. Alcune decisioni dei programmatori iniziali sono contemporaneamente il punto forte e il punto debole del progetto. Ad esempio, la scelta di mantenere lo standard di scambio messaggi il più generico possibile rispetto all'hardware, da una parte fa si che gli stessi algoritmi vadano bene per robot con forme e caratteristiche diverse (un robot umanoide uno su ruote e un quadricottero, in ROS, sono controllabili nello stesso modo). Dall'altro crea un grosso overhead nella dimensione dei messaggi.
3. La curva di apprendimento di ROS è bella tosta, specialmente all'inizio (ve ne accorgerete se seguirete queste guide).

## Cosa fare in questo laboratorio

Avrete a disposizione un Raspberry Pi 3 model B con sopra installato ROS Kinetic.
 - username: ros
 - password: ros

Il primo laboratorio consiste nell'iniziare a prendere confidenza con il sistema publishing
subscribing di ROS seguendo le seguenti guide:

 - [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
 - [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
 - [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
 - [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
 - [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

Alla fine dei tutorial, provare a scrivere un semplice nodo publisher che fa muovere la
turtlesim.

## Rimaniamo in contatto

Abbiamo creato un gruppo facebook all'interno del quale riunire varie persone che hanno
o vogliono farsi competenze su ROS: [robot developer italiani](https://www.facebook.com/groups/493163691070528/).
