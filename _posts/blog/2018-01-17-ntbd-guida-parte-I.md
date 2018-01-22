---
title: "NTBD: guida step by step I"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-parte1.png
headerImage: true
lang: it
otherlanglink: /en/blog/
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
category: blog
author: fiorellazza
description: "Cos'è e come utilizzare NTBD step by step, primo articolo della serie"
---
[> Switch to the English version]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-I %})

[>> Vai a Post Parte II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %})

[>> Vai a Post Parte III: integrazione con altri manipolatori]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-III %})

## Parte I: una panoramica
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

[**<<Torna all'indice**](#indice)

### NTBD: Name To Be Decided
Ebbene sì, ecco il nome tanto ricercato in tutta la sua gloria! Dopo pomeriggi passati a scegliere un nome che fosse accattivante e cool questo è il risultato...
<!-- gif*Not so impressive*-->
![enter image description here](https://media.giphy.com/media/rwedxv8kWXBaU/giphy.gif)

Sono però dell'idea che l'importante sia il contenuto.
A proposito di contenuto, adesso procederò a darvi una veloce panoramica top-down su NTBD.

[**<<Torna all'indice**](#indice)

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

## FINE PARTE I

[>> Vai a Post Parte II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %})

[>> Vai a Post Parte III: integrazione con altri manipolatori]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-III %})
