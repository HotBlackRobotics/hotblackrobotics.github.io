---
title: "NTBD: guida step by step II"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-parte2.png
headerImage: true
lang: it
otherlanglink: /2018/01/17/ntbd-guide-part-II/
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
description: "Cos'è e come utilizzare NTBD step by step, secondo articolo della serie"
---
[> Switch to the English version]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guide-part-II %})

[<< Torna a Post Parte I: una panoramica]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guida-parte-I %})

[>> Vai a Post Parte III: integrazione con altri manipolatori]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guida-parte-III %})

## Parte II: tutorial
In questo articolo vi spiegherò come riprodurre il progetto, completo di braccio robotico integrato all'architettura NTBD, introdotta nel post [Parte I: una panoramica]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guida-parte-I %}).

### Ingredienti
Ci serviranno:
- 1 braccio EEZYBOT MK2 stampato in 3D, completo di servo-motori (3 mini-servo, 1 micro-servo)
- 1 scheda Arduino (nel mio caso ho usato una Mega ADK 2560)
- 1 pc con processore Intel su cui installare Docker
 - 1 controller [Leap Motion](https://www.leapmotion.com/) per una delle applicazioni robotiche implementate
 - 1 Alimentatore esterno da 2A-7.5V
- 1 Breadboard
- Jumper q.b.

**Nota:** Questo tutorial si riferisce all'Immagine Docker per host Intel ma è disponibile anche l'**Immagine ARM** (può essere eseguita su Raspberry 3). Per ottenere la versione ARM, quando un comando è necessario o un documento è citato, è sufficiente *sostituire la parola **intel** con **rpi3***.

#### Indice
 1. [Stampare il Braccio Robotico](#1-stampare-il-braccio-robotico)
 2. [Scaricare lo sketch per Arduino](#2-scaricare-lo-sketch-per-arduino)
 3. [Scaricare Docker](#3-scaricare-docker)
 4. [Scaricare le Immagini Docker](#4-scaricare-le-immagini-docker)
 5. [Collegamenti](#5-collegamenti)
 6. [Avviare il Container Docker](#6-avviare-il-container-docker)
 7. [WebApp per NTBD e siBOT](#7-webapp-per-ntbd-e-sibot)
 8. [Giocare con le WebApps](#8-giocare-con-le-webapps)

### 1. Stampare il Braccio Robotico
Il braccio robotico da me scelto è [EEZYbotARM MK2](http://www.eezyrobots.it/eba_mk2.html), progetto open source italiano di Carlo Franciscone, Design Engineer e Maker.

Seguendo le istruzioni presenti su [*Thingiverse*](https://www.thingiverse.com/thing:1454048) ed [*Instructables*](http://www.instructables.com/id/EEZYbotARM-Mk2-3D-Printed-Robot/) relativi a questo progetto, ho completato con successo la stampa 3D ed il montaggio del braccio robotico.
Il software per la stampa che ho utilizzato è [Cura](https://ultimaker.com/en/products/ultimaker-cura-software) con parametri configurati per stampare con una [DeltaWASP](http://www.wasproject.it/w/stampa-3d/).
![3d printing](/assets/imgs/2018-01-17-ntbd/5_piece1.jpg)
Per la stampante DeltaWASP scaricate il seguente [profilo](http://www.personalfab.it/en/downloads-2/download-info/profili-cura/) e caricatelo seguendo la seguente [guida](https://ultimaker.com/en/resources/52032-manage-profiles).

Di seguito i parametri più significativi utilizzati per stampare i pezzi, per migliorarne la definizione:

|Parametro        |  Valore     |
|:--------------|:---------------------:|
| Infill | 30-100%. Vi consiglierei di stampare con infill del 30% le parti del braccio sottoposte a poco sforzo mentre le parti meccaniche più piccole sono state stampate con il 100% di infill. |
| Printing Temperature | 200-210°C|
| Build Plate Temperature| 40°C|
|Filament Diameter| 1.8 mm
| Print Speed | 60 mm/s|
| Infill Speed| 80 mm/s|
| Travel Speed| 150 mm/s|
| Support| Enabled|
|Platform Adhesion| Enabled|

<br>
Questo è il risultato:

![eezybot lateral](/assets/imgs/2018-01-17-ntbd/5_eezybotlateral.jpg)

[**<<Torna all'indice**](#indice)

### 2. Scaricare lo sketch per Arduino
Sulla pagina github di NTBD troverete lo sketch [myServoControl_ntbd.ino](https://github.com/HotBlackRobotics/ntbd/blob/devel/myServoControl_ntbd.ino) da caricare sulla scheda Arduino. Nel mio caso ho usato una Arduino Mega ADK 2560.
![arduino](/assets/imgs/2018-01-17-ntbd/5_mega.jpeg)

[**<<Torna all'indice**](#indice)

### 3. Scaricare Docker
Per Ubuntu usare questo [link](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#uninstall-old-versions).

[**<<Torna all'indice**](#indice)

### 4. Scaricare le Immagini Docker
A questo punto per scaricare le immagini di NBTD basterà eseguire i seguenti comandi nella vostra *command line*, per ottenerle da [Docker Hub](https://docs.docker.com/docker-hub/), usando il comando [*docker pull*](https://docs.docker.com/engine/reference/commandline/pull/):
```
docker pull hbrobotics/ntbd_base:intel
```
```
docker pull hbrobotics/ntbd_manipulator:intel
```
[**<<Torna all'indice**](#indice)

### 5. Collegamenti
 Come possibile vedere dallo sketch, i servomotori, numerati come in figura, sono collegati all'Arduino come riportato in tabella.

 ![servo](/assets/imgs/2018-01-17-ntbd/5_eezybotfrontservo.jpg)

| Servo         |  Pin Arduino     |
|:--------------:|:---------------------:|
| 1 | 2|
| 2 | 3|
| 3 | 4|
| 4 | 5|

<br>
Per muovere tutti i servomotori contemporaneamente senza sovraccaricare la scheda, colleghiamola ad un alimentatore esterno a 2 A e 7.5 V.![arduino](https://www.modmypi.com/image/data/tutorials/how-to-power-my/6.jpg)
Colleghiamo su una breadboard i mini-servo con V+ collegato al pin *Vin* di Arduino e V- collegato ad uno qualsiasi dei pin *GND* (ground) della scheda.

Per evitare di surriscaldare il micro-servo, lo colleghiamo il suo V+ al pin da 5V dell'Arduino e V- ad un pin GND.

**ATTENZIONE:** controllate sempre che il ground sia comune a tutti i motori (GND breadboard = GND arduino).

Colleghiamo l'Arduino e il controller Leap Motion al computer tramite USB.

![leap](/assets/imgs/2018-01-17-ntbd/5_leapmotion.jpg)

Per installare correttamente i driver per Leap Motion su Ubuntu 16.04, seguite questa [guida](https://support.leapmotion.com/hc/en-us/articles/223782608-Linux-Installation) insieme a questa piccola [modifica](https://forums.leapmotion.com/t/tip-ubuntu-systemd-and-leapd/2118).

[**<<Torna all'indice**](#indice)

### 6. Avviare il Container Docker
Adesso che tutti i dispositivi esterni sono collegati, scaricate il [file .yaml](https://github.com/HotBlackRobotics/ntbd/blob/devel/docker-compose.hbr_ntbd_intel.yml) per l'immagine ntbd_manipulator:intel, che è quella che vogliamo avviare. Per avviare il container (dove è presente l'applicazione Web), basta eseguire il seguente comando nella cartella contenente il file .yaml, sfruttando il tool [Docker Compose](https://docs.docker.com/compose/overview/):
```
docker-compose -f docker-compose.hbr_ntbd_intel.yml up
```
[**<<Torna all'indice**](#indice)

### 7. WebApp per NTBD e siBOT
L'insieme di NTBD e il braccio EEZYBOT controllato da ROS è quello che ho voluto chiamare **siBOT**.
![sibot](/assets/imgs/2018-01-17-ntbd/sibot.png)

Una volta che il contenitore è stato avviato, apriamo una pagina Browser e connettiamoci all'indirizzo di loopback, *localhost*, per collegarci al nostro stesso computer: infatti il server dell'applicazione è sul nostro pc. Avendo chiamato il file HTML "index.html", connettendoci al nostro indirizzo IP sulla porta default 80 (come ci aspettiamo, avendo mappato la porta 80 del container a quella 80 dell'host) la *homepage* dell'indirizzo sarà proprio la nostra WebApp per NTBD-Visualizer, provvista di link all'applicazione con Leap Motion.

![webapp](/assets/imgs/2018-01-17-ntbd/5_ntbdviz.png)

[**<<Torna all'indice**](#indice)

### 8. Giocare con le WebApps
#### Simulazione con NTBD - Visualizer
A questo punto avrete a disposizione l'applicazione, dalla quale potete impostare una posizione desiderata nello spazio, muovendo gli sliders per le coordinate Cartesiane; Premendo il bottone **Execute** vedrete il modello del braccio muoversi insieme al braccio fisico.

**Nota**: i valori di input inseriti dall'utente tramite la WebApp NTBD - Visualizer, sono utilizzati per il controllo sia del modello in simulatione che del braccio fisico.

#### Controllo con Leap Motion
Nel caso si volesse utilizzare la seconda applicazione sviluppata, basterà premere sul link "*NTBD Leap Motion WebApp*" il quale aprirà una nuova pagina. Tenendo selezionata questa nuova finestra/scheda, vedremo che, posizionando la mano davanti al dispositivo Leap Motion, le posizioni nello spazio vengono interpretate ed inviate al robot fisico che riprodurrà i movimenti della mano.
![leapspace](/assets/imgs/2018-01-17-ntbd/5_leapref.png)

**Nota**: per aprire e chiudere la pinza, semplicemente aprite e chiudete la mano!

[**<<Torna all'indice**](#indice)

## FINE PARTE II

[<< Torna a Post Parte I: una panoramica]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guida-parte-I %})

[>> Vai a Post Parte III: integrazione con altri manipolatori]({{ site.baseurl }}{% post_url /blog/2018-01-17-ntbd-guida-parte-III %})
