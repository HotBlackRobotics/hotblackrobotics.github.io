---
title: "NTBD: guida step by step I"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-parte1.png
headerImage: true
lang: it
otherlanglink: /2018/01/17/ntbd-guide-part-I/
tag:
 - NTBD
 - Docker
 - ROS
 - 3D Printing

author: fiorellazza
description: "Cos'è e come utilizzare NTBD step by step, primo articolo della serie"
---

* TOC
{:toc}

# Parte I: una panoramica
Ciao a tutti! Sono di nuovo io, Fiorella e, con questa *trilogia* di post, vorrei presentarvi il lavoro svolto per la mia Tesi di Laurea Magistrale in Ingegneria Meccatronica conseguita al Politecnico di Torino, "*Development of a Standard Architecture
to enable Fast Software Prototyping
for Robot Arms*",  e fornirvi una guida step by step il più possibile chiara (si spera!) per poter utilizzare e ri-utilizzare l'architettura sviluppata, chiamata NTBD.
In questa prima parte vi presento l'architettura proposta così come la troverete esplorando il [progetto su github](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c). Nel secondo post invece ci sarà un tutorial su come utilizzare l'architettura integrata ad un braccio robotico Open-Source da me scelto, avendo a disposizione un PC con processore Intel (oppure un host con processore ARM). Infine nel terzo articolo spiegherò come poter sfruttare NTBD utilizzando un braccio a scelta diverso da quello presentato, evidenziandone i limiti.

#### Indice:
 1. [Motivazioni](#motivazioni)
 2. [NTBD: Name To Be Decided](#ntbd-name-to-be-decided)
 3. [NTBD - Livello Concettuale](#ntbd---livello-concettuale)
 4. [NTBD - Livello Docker](#ntbd---livello-docker)
 5. [NTBD - Livello ROS](#ntbd---livello-ros)

### Motivazioni
L'idea della mia tesi è nata dall'osservazione che nell'ambito della prototipazione rapida, l'hardware ha sempre avuto dei validi rappresentanti, per esempio le schede Open-Source [Arduino](http://www.arduino.org/) e [RaspBerry Pi](https://www.raspberrypi.org/); per quanto riguarda la Robotica, esistono innumerevoli progetti Open-Source per la costruzione di robot, muniti di istruzioni, indicazioni sull'opportuno hardware per il controllo, pezzi necessari alla costruzione (i quali possono essere stampati in 3D) e lista di minuteria necessaria.
Per quanto riguarda invece il software, non c'è a disposizione un'architettura standard su cui costruire in modo semplice la propria applicazione robotica. Ecco che entra in scena **NTBD**, pensata per lo sviluppo di applicazioni con bracci robotici.

### NTBD: Name To Be Decided
Ebbene sì, ecco il nome tanto ricercato in tutta la sua gloria! Dopo pomeriggi passati a scegliere un nome che fosse accattivante e cool questo è il risultato...
<!-- gif*Not so impressive*-->
![enter image description here](https://media.giphy.com/media/rwedxv8kWXBaU/giphy.gif)

Sono però dell'idea che l'importante sia il contenuto.
A proposito di contenuto, adesso procederò a darvi una veloce panoramica top-down su NTBD.

### NTBD - Livello Concettuale
![ntbd-conceptual](/assets/imgs/2018-01-17-ntbd/4_architect.png)

L'architettura è composta da vari elementi tra cui troviamo dei componenti astratti, ovvero componenti generici che possono essere implementati dall'utente. Ecco il significato di ogni componente astratto:
 - **IK & FK** implementano cinematica inversa (Inverse Kinematics) e cinematica diretta (Forward Kinematics) per l'end effector del braccio robotico.
 - **URDF**: è un file ([Universal Robot Description Format](http://wiki.ros.org/urdf)) che descrive la struttura cinematica del braccio per definirne il modello per la simulazione.
 - **P2V**: qui vengono convertiti ed adattati gli angoli dei giunti del braccio per ottenere i valori corrispondenti per muovere conformemente il modello in simulazione.
 - **Motor Values**: in questo elemento vengono uniti, in un unico array, i valori dei giunti e della pinza del braccio, se presente.
 - **HW**: questo componente rappresenta l'hardware utilizzato per il controllo del manipolatore, in questo caso una scheda Arduino.

 I seguenti sono invece i componenti che sono indipendenti dalla struttura del manipolatore scelto:
 - **Position Limiter**:  questo elemento serve a limitare la posizione che si desidera raggiungere determinata dai limiti inferiori e superiori della coordinate Cartesiane x,y,z.
 - **Path Planner**: ho scelto, per questioni di semplicità, di pianificare il percorso dell'end effector in modo lineare, assumendo che non ci siano ostacoli.
 - **Values Limiter**: anche i valori dei motori devono essere limitati per evitare danni o comportamenti inattesi.
 - **Rosserial**: questo elemento rappresenta l'importanza del tool [Rosserial](http://wiki.ros.org/rosserial) necessario per stabilire una connessione seriale con la scheda di prototipazione rapida scelta per il controllo.
 - **Rosbridge**: questo componente rappresenta il tool di ROS [Rosbridge](http://wiki.ros.org/rosbridge_suite), che permette di simulare il nostro braccio robotico tramite l'integrazione di ROS con il mondo Web utilizzando una libreria JavaScript, [roslibjs](http://wiki.ros.org/roslibjs).

[**<<Torna all'indice**](#indice)

### NTBD - Livello Docker
L'architettura è portabile grazie alla sua definizione all'interno di un contenitore [Docker](https://www.docker.com/) il quale ne consente l'esecuzione dei componenti sempre uguale a se' stessa, con l'unico requisito di un sistema compatibile con Docker.
Chi ha qualche concetto base di Docker, saprà bene che un'applicazione Docker viene sviluppata partendo da un'immagine base che solitamente corrisponde all'immagine di un sistema operativo. Nel mio caso l'immagine di partenza è Ubuntu 16.04 su cui è pre-installato ROS Kinetic Kame (decima distribuzione di ROS, 2016) ottenuti dal [github di HotBlack Robotics](https://github.com/HotBlackRobotics/docker-ros-base). Come possibile vedere dalla [repository NTBD](https://hub.docker.com/u/hbrobotics/) su [Docker Hub](https://docs.docker.com/docker-hub/) ci sono due *layer* che compongono l'archiettura, *base* e *manipulator*. La prima contiene gli elementi dell'architettura comuni a tutti i bracci robotici (Position Limiter, Path Planner, Values Limiter, Rosserial e Rosbridge); su quest'ultima è costruita la seconda immagine in cui vengono implementati i componenti astratti, analizzati prima.

[**<<Torna all'indice**](#indice)

### NTBD - Livello ROS
I componenti presentati precedentemente sono stati implementati in [ROS](http://www.ros.org/), Robot Operating System, come nodi Python. Assumo che il lettore conosca le dinamiche base del framework ROS così da non dilungarmi in spiegazioni ulteriori e procedere all'elenco e relativa spiegazione di [nodi](http://wiki.ros.org/Nodes) e [topics](http://wiki.ros.org/Topics) di questa architettura.

 In Figura vediamo i nodi e i topics coinvolti nelle dinamiche di NTBD.
![ntbd-ros](/assets/imgs/2018-01-17-ntbd/4_archrosgraph.png)
<!--Per meglio individuare il ruolo di ogni nodo e topic, li suddividerò in categorie di attinenza.
 Nodi e Topic per il controllo del robot -->
La tabella riportata qui sotto presenta i topics e i relativi tipi di [messaggi ROS](http://wiki.ros.org/Messages).

**Note**:
- Motors_Array è un tipo custom, creato per rendere più chiaro il contenuto del messaggio stesso ed avere una struttura semplice.
- il tipo di messaggio di */girpper_value* dipende dall'implementazione dell'utente.

| Topic         |  ROS Message Type     |
|:--------------|:---------------------|
| /desired_position_nolim | [geometry_msgs/Point](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html)|
| /desired_position_nointerp |geometry_msgs/Point|
| /desired_position | geometry\_msgs/Point|
| /motors_nogripper | Motors\_Array|
| /gripper_value | [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)|
| /motors_nolim | Motors_Array|
| /motors | Motors_Array|
| /actual\_position | geometry\_msgs/Point|
| /joint\_states | [sensor\_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)|

<br>
 - Topic **desired_position_nolim**: la posizione desiderata viene pubblicata su questo topic come messaggio di tipo Point.
 - Nodo  **/position_limiter**: qui i valori che cadono al di fuori dei limiti (che sono definiti come [parametri ROS](http://wiki.ros.org/Parameter%20Server#Parameters)), vengono saturati in modo tale da evitare errori dovuti a posizioni non raggiungibili.
 - Topic **/desired_position_nointerp**: le posizioni "filtrate" dal nodo */position_limiter* vengono pubblicate su questo topic.
 - Nodo  **/path_planner**: lo scopo di questo nodo è quello di calcolare l'interpolazione lineare tra due posizioni consecutive pubblicate su */desired_position_nointerp*.
 - Topic **/desired_position**: qui viene pubblicata la sequenza di punti nello spazio calcolata dal nodo di path planning.
 - Nodo **/IK**: la posizione desiderata viene qui convertita in valori per i motori (angoli , nel caso dei servo-motori).
 - Topic **/motors_nogripper**: i valori ottenuti in */IK* sono pubblicati qui.
 - Topic **/gripper_value**: fornisce in output il valore desiderato per la pinza del braccio, in questo caso la pinza può essere aperta o chiusa.
 - Nodo **/motors\_values**: qui i valori dei motori e della pinza vengono uniti in un unico vettore di dati.
 - Topic **/motors\_nolim**: l'insieme di tutti i valori desiderati viene pubblicato su questo topic.
 - Nodo **/motors\_limiter**: questo nodo legge i valori dei motori e, se necessario li limita confrontandoli con i valori limite definiti come parametri ROS. Questo nodo è fondamentale nel caso che l'utente voglia comandare il robot direttamente nello spazio dei giunti.
 -  Topic **/motors**: i valori limitati sono finalmente disponibili su questo topic.
 - Nodo Rosserial **/init\_serial\_node**: questo nodo, che corrisponde al nodo [*/serial_node*](http://wiki.ros.org/rosserial_python#serial_node.py), consente di stabilire una connessione tra il nodo seriale caricato sull'hardware (in questo caso Arduino) e il sistema ROS sul computer. Infatti i valori dei motori vengono letti dall'apposito topic e processati dalla scheda per muovere i servo.
 - Nodo **/FK**: i valori dei motori possono essere utilizzati per il calcolo della cinematica diretta per ottenere la posizione reale, che può essere diversa da quella desiderata a causa dei limiti fisici.
 - Topic **/actual\_position** la posizione raggiunta viene pubblicata su questo topic.

Per quanto riguarda la simulazione del robot su una pagina web, ho dovuto rendere disponibili su un server il file URDF del braccio e il file HTML dell'applicazione. La soluzione è stata quella di tirare su un server, all'interno dell'ambiente containerizzato, utilizzando [*nginx*](https://www.nginx.com/resources/glossary/nginx/), un web server Open-Source che può essere utilizzato per fornire sulla rete contenuti HTTP. Per visualizzare la simulazione sul browser del sistema operativo host è bastato mappare la porta 80 del contenitore su una porta a scelta, per esempio 1234 dell'host (nella versione definitiva la porta 80 è mappata alla porta standard dell'host ovvero la porta 80).
La cartella root di default di nginx (/var/www/html) contiene il documento HTML, dove avvengono le interazioni tra Rosbridge e ROS e la descrizione URDF del robot necessaria al rendering di quest'ultimo.

![ntbd-simulation](/assets/imgs/2018-01-17-ntbd/4_sim.png)

Ripartendo da dove eravamo rimasti per la descrizione a livello ROS, i seguenti sono i nodi e topics coinvolti nella simulazione del nostro braccio:

 -  Nodo **/physical_2_visual** :  questo nodo fa parte dei componenti astratti, dipendente quindi dal braccio scelto. Si tratta di un nodo di conversione che mappa i valori fisici forniti dal topic */motors* ai valori corrispondenti in visualizzazione.
 - Topic **/joint\_states**: qui vengono pubblicati i valori dei joint in visualizzazione.
Da qui in avanti, la gestione dei messaggi su topic ed i nodi è quella tipica di ROS per i dati 3D: i valori dei joint sono passati al topic [**/robot\_state\_publisher**](http://wiki.ros.org/robot_state_publisher) che fornisce in output messagi di tipo [*geometry_msgs/Transform*](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Transform.html) sfruttando il package [*tf2*](http://wiki.ros.org/tf2) di ROS, il quale consente all'utente di tener traccia dei sistemi di coordinate nel tempo.  A questo punto, queste informazioni vengono rese note a Rosbridge; dal momento che i nodi e topic coinvolti in questa parte sono abbastanza standard, non mi dilungherò oltre.
![graph-simulation](/assets/imgs/2018-01-17-ntbd/5_rosbr.png)

Collegandoci quindi "a noi stessi" dal browser, sulla porta definita potremo vedere il modello del braccio muoversi insieme al braccio fisico.
La connessione Rosbridge viene iniziata sulla porta di default 9090 sulla quale un altro computer può interagire con il sistema ROS pubblicando sui topics e iscrivendosi ad essi.

[**<<Torna all'indice**](#indice)


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


## Parte III: integrazione con altri manipolatori
Per adattare l'archiettura NTBD ad un manipolatore diverso da EEZYBOT, è necessario ridefinire i componenti astratti dell'archiettura.  Inoltre assumiamo che la scheda utilizzata per il controllo dei motori sia una scheda Arduino.
I componenti astratti sono implementati nell'Immagine Docker *ntbd_manipulator* e, come spiegato nel post ["Parte I: una panoramica"](), dipendono dal braccio robotico scelto e vanno quindi modificati conformemente alla struttura scelta.

**Note**:
- Per poter fare il build delle Immagini Docker è necessario [installare Docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#uninstall-old-versions). In questo tutorial assumo che il lettore abbia delle conoscenze su come sviluppare applicazioni in Docker. Per consigli sulla fase development, leggere l'[appendice](#appendice-docker-prod-vs-docker-devel).

### Indice
1. [Modificare i componenti astratti di NTBD](#1-modificare-i-componenti-astratti-di-ntbd)
2. [Fare il build della nuova immagine](#2-fare-il-build-della-nuova-immagine)
3. [Avviare il nuovo contenitore](#3-avviare-il-nuovo-contenitore)
4. [Appendice: Docker Prod vs Docker Devel](#appendice-docker-prod-vs-docker-devel)

### 1. Modificare i componenti astratti di NTBD
Sarà necessario scaricare il [progetto NTBD da github](https://github.com/HotBlackRobotics/ntbd) in modo tale da modificare i seguenti file prima di fare il build della nuova Immagine *ntbd_manipulator*:

- [*joint_names_sibot_urdf.yaml*](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/config/joint_names_sibot_urdf.yaml): in questo file sono definiti i nomi dei joint del braccio robotico, utili per lo scambio di messaggi ROS. Questa definizione è utile, per esempio, nel nodo [physical_2_visual](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/physical_2_visual), che deve quindi essere modificato di conseguenza.
- [*/meshes/*](https://github.com/HotBlackRobotics/ntbd/tree/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/meshes): questa cartella contiene le [meshes](https://it.wikipedia.org/wiki/Mesh_poligonale) per la visualizzazione del braccio robotico scelto, in formato [*STL*](https://it.wikipedia.org/wiki/STL_(formato_di_file)).
 - [siBOT_noEE.urdf](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/urdf/siBOT_noEE.urdf): questo file deve contenere la definizione [URDF](http://sdk.rethinkrobotics.com/wiki/URDF) del nuovo manipolatore; può quindi essere rinominato a piacere con l'unico accorgimento di cambiare il nome anche nel launch file [NTBD_launch.launch](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/launch/NTBD_launch.launch).
 - [index.html](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/web/index.html) e [ntbd_lm.html](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/web/ntbd_lm.html) definiscono le applicazioni Web e devono essere modificati a seconda della nuova configurazione (per esempio, con i nuovi limiti).
 - [IK](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/IK), [FK](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/FK), [motor_values](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/motors_values) e [physical_2_visual](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/physical_2_visual): questi file sono tutti dipendenti dalla scelta del braccio robotico; per ulteriori informazioni riguardo al loro ruolo .
- [*NTBD_launch.launch*](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/launch/NTBD_launch.launch): questo file deve essere modificato per modificare i parametri ROS dei limiti per le [coordinate della posizione](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/launch/NTBD_launch.launch#L16) e per i [motori](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/launch/NTBD_launch.launch#L24).
- [myServoControl_ntbd.ino](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/myServoControl_ntbd.ino): ovviamente, cambiando  il manipolatore,  cambia la configurazione fisica del braccio (numero e tipo di motori) e di conseguenza lo sketch caricato sulla scheda Arduino.

**Nota**:
- il package ROS [*manipulator_urdf*](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf), che contiene tutte le info necessarie per utilizzare la definizione URDF del robot, può essere prodotto automaticamente partendo dall'assembly del robot, utilizzando il tool [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly).
- Nel caso si voglia modificare il nome di un file bisogna tener sempre conto del fatto che questo file potrebbe essere "chiamato" in qualche altro file e quindi quest'ultimo dovrebbe essere modificato di conseguenza. Il consiglio è quindi quello di *evitare di rinominare i file* affinchè non ci siano intoppi.

[**<<Torna all'indice**](#indice)

### 2. Fare il build della nuova immagine
Una volta modificati i file necessari, è ora di "buildare" la nuova immagine. Entrare nella cartella [NTBD_manipulator](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator) in cui è contenuto il file [Dockerfile.intel](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/Dockerfile.intel) ed eseguire il seguente comando.
```
docker build -t ntbd/manipulator:intel .
```

[**<<Torna all'indice**](#indice)

### 3. Avviare il nuovo contenitore
Dopo aver collegato tutti i dispositivi esterni, basterà quindi avviare il contenitore eseguendo:
```
docker-compose -f docker-compose.intel.yml up
```
che sfrutta il tool [Docker Compose](https://docs.docker.com/compose/overview/) con configurazione specificata nel file [docker-compose.intel.yml](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/docker-compose.intel.yml).

L'integrazione di un nuovo braccio finisce qua, per informazioni a proposito dell'utilizzo delle WebApp implementate, vedi il [Post Parte II: un tutorial]().

[**<<Torna all'indice**](#indice)

### Appendice: Docker prod vs Docker devel
Quando si lavora con Docker conviene, per motivi di organizzazione e ordine, usare un'Immagine per lo sviluppo (Development Image) ed una per la produzione (Production Image). Quest'ultima è la versione finale dell'applicazioe Docker, pronta per la distribuzione, mentre la prima è usata durante lo sviluppo dell'applicazione. Viene "buildata" un'immagine comune sulla quale si vanno a sviluppare la versione Prod e la versione Dev e, con qualche accorgimento, si può sfruttare al meglio il sistema di caching per la fase di *build* dell'immagine. Infatti, se i file modificati sono ancora in fase di debug, ad ogni build i layer, anche se già buildati in precedenza, vengono ri-buildati.

![docker](/assets/imgs/2018-01-17-ntbd/4_dockerdev.png)

Per la fase di sviluppo è quindi consigliato avere tutti i file non ancora definitivi in una cartella condivisa tra l'host e il container: tramite uno script bash, eseguito all'avvio del contenitore, i file vengono copiati nel container.
Questo, chiaramente, aumenta il tempo di avvio ma evita che l'immagine venga re-buildata ogni volta che un file viene modificato. L'Immagine Prod, che nel caso di ntbd è quella resa [disponibile su Docker Hub](https://hub.docker.com/r/hbrobotics/ntbd_manipulator/), copia i file definitivi dal *building context* (la cartella in cui sono contenute tutte le risorse necessarie all'esecuzione del container) al container, dal momento che tutti file, a questo punto, dovrebbero essere alla loro versione finale.

Di seguito un esempio di building context:

![buildingcontext](/assets/imgs/2018-01-17-ntbd/building-context.png)

Qui invece riporto il contenuto del file bash NTBD_devel_entrypoint.sh usato per la versione devel:

```bash
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
[**<<Torna all'indice**](#indice)
