---
title: "Corso di ROS Robot Operating System e navigazione autonoma per aziende"
layout: post
date: 2018-03-9
image: https://raw.githubusercontent.com/sgabello1/sganagesh/master/nav-auto-ros.jpg
lang: it
tag:
 - industry 4.0
 - ROS
 - Navigazione autonoma

author: sgabello
description: "Corso di ROS Robot Operating System e navigazione autonoma per aziende"
---
<div class="left-element mailchimp"> <h1> Iscriviti al corso </h1> <p> Ricevi maggior maggior informazioni ed il simulatore ROS per la navigazione autonoma </p> <a href="http://bit.ly/2IkaHZv" role="button" class="btn" target="blank"> Iscriviti </a>
</div>


![navigazione autonoma](https://raw.githubusercontent.com/sgabello1/sganagesh/master/nav-auto-ros.jpg)

## Robot Operating System ##

ROS (Robot Operating System) è il framework *standard de-facto* per la robotica di servizio. Fornisce librerie e tools per aiutare gli sviluppatori a creare robot applications.

Esso fornisce astrazione dell'hardware, driver dei dispositivi, librerie, strumenti di visualizzazione, comunicazione a scambio di messaggi tra processi (message passing), gestione dei pacchetti e molto altro. ROS è rilasciato sotto una licenza open source, BSD license.


## Navigazione Autonoma ##

La navigazione autonoma sta diventando un tema sempre più importante sia per la robotica mobile e di servizio sia per come sta rivoluzionando il mercato automotive. [Qui](http://bit.ly/2FBUksW) puoi scaricare una breve introduzione scritta dal [Professor Basilio Bona](http://www.ladispe.polito.it/robotica/Curriculum/bben.htm) del Politecnico di Torino.

L'obiettivo di questo corso è di dare gli strumenti e la conoscenza di base per capire e creare semplici progetti di navigazione autonoma in ROS. Imparerai a creare mappe 2D, localizzare il robot all'interno della mappa, utilizzare i pacchetti software per il *path planning*, visualizzare dati di diversi processi di navigazione, monitorare, simulare tramite Rviz e configurare i nodi di navigazione.

## Materiale propedeutico pre-corso ##

Iscrivendoti (al fondo della pagina) riceverai gratis un video tutorial ed una macchina virtuale già configurata con ROS ed il pacchetto di navigazione dove potrai simulare l'utilizzo di un robot mobile (Turtlebot 3) tramite Rviz.

<iframe width="1120" height="630" src="https://www.youtube.com/embed/dQrrTUCkyWk" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

# Programma dettagliato#

## Giorno 1 ##

Lo scopo principale del primo giorno è quello di fornire una panoramica sulla tecnologia e permettere ai partecipanti di capire i concetti base su cui si fonda ROS e la teoria abilitante alla navigazione autonoma.

* Introduzione
  - La robotica di Servizio, cosa è, quali sono le tecnologie e le necessità?
  - ROS: Robot Operating System. Un framework per sviluppare applicazioni Robotiche.

* L'ecosistema ROS
  - La rete ROS (ROSGraph): nodes, topics, services, actions
  - Organizzazione del codice: i pacchetti e i metapacchetti
  - Tools linea di comando: catkin, rostopic, rosmsg, roscode
  - ROS Tools & Community

* Navigazione autonoma: Sensori e Algoritmi
  - Introduzione alla navigazione Autonoma
  - Bayesian Filters e i suoi derivati
  - Localizzazione e pose tracking
  - Mapping autonomo
  - Simultaneous Localization and Mapping
  - Path Planning
  - Path Following e Obstacle Avoidance
  - I Tipi di sensori utilizzabili in queste tecnologie, proprietà e limiti
  - Dove trovare queste tecnologie in ROS

* L'ecosistema ROS
  - La rete ROS (ROSGraph): nodes, topics, services, actions
  - Organizzazione del codice: i pacchetti e i metapacchetti
  - Tools linea di comando: catkin, rostopic, rosmsg, roscode ...
  - ROS Tools & Community

# Giorno 2 #

Il secondo giorno comprende esercizi pratici per lo sviluppo di nodi ROS in Python e in C++. In questo percorso, diviso in due parti, i partecipanti impareranno a sviluppare semplici nodi ROS fino a complesse applicazioni di robotica mobile, incluso anche lo sviluppo di applicazioni grafiche Web-based per il controllo e la visualizzazione dello stato del robot da parte di un utente.

Il docente metterà a disposizione tutto il suo know-how sviluppato in anni di lavoro su ROS e la best practice sviluppate durante la sua attività, insieme a vari codici di esempio per ogni argomento trattato.

* Parte 1
  - Setup ambiente di sviluppo ed introduzione ai comandi fondamentali
  - Creazione di un pacchetto ROS ed organizzazione dei codici
  - Sviluppiamo i primi nodi ROS
  - Interagiamo con RVIZ e RQT per la visualizzazione dei dati scambiati e della rete ROS
  - Utilizzo dei messaggi ROS principali std_msgs, geometry_msgs, sensor_msgs
  - Sviluppo di Applicazioni grafiche con RosLibjs

* Parte 2
  - Setup di un simulatore con ROS e Gazebo
  - Setup dello stack di navigazione per un robot mobile
  - Creazione della mappa con gmapping
  - Controllo del movimento con move_base e RVIZ
  - Utilizzo delle API dello stack di navigazione per sviluppare un robot autonomo.

## Docente ##

![ludovico orlando russo](https://avatars3.githubusercontent.com/u/8146506?s=400&v=4)

**Ludovico O. Russo** è un esperto di Robotica e di ROS (Robot Operating System), che dal 2012 si occupa di sviluppo di applicazioni connesse. Ha frequentato il Ph.D. in Robotica presso il Politecnico di Torino (conseguito in Maggio 2017 con Lode) in collaborazione con TIM, all’interno del laboratorio di ricerca TIM Joint Open Lab. I suoi interessi principali sono la navigazione autonoma, la cloud robotics e lo sviluppo di applicazioni robotiche connesse.

## A chi è rivolto il corso ##

Il corso è rivolto ad aziende, professionisti, personale tecnico, studenti del settore.
Sono richieste competenze base di Linux Shell/CLI e Python/C++.


## Materiale ##

Ogni partecipante avrà la necessità di utilizzare un computer con Ubuntu 16.04 e ROS (Kinetic). Saranno messi a disposizione per tutta la durata del corso alcuni robot mobili la sperimentazione:

|![](http://www.robotsepeti.com/turtlebot-3-burger-4521-38-B.png)|![](https://www.robot-advance.com/EN/ori-turtlebot-2-complete-assembly-1189.png)|![](http://www.nuzoo.it/images/slideshow_raro/RaRo_Sourveillance_Robot--15.jpg)|
|:------------:|:------------:|:------------:|
| Turtlebot 3 | Turtlebot 2 | Nuzoo robot |


<div class="left-element mailchimp"> <h1> Iscriviti al corso </h1> <p> Ricevi maggior maggior informazioni ed il simulatore ROS per la navigazione autonoma </p> <a href="http://bit.ly/2IkaHZv" role="button" class="btn" target="blank"> Iscriviti </a>
</div>
