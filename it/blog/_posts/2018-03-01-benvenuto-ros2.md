---
title: "Benvenuto a ROS 2.0"
layout: post
date: 2018-03-01
image: /assets/imgs/2018-03-01-benvenuto-ros2/main.png
lang: it
headerImage: true
tag:
 - ROS
 - ROS 2
author: ludusrusso
description: "Primo contatto con ROS 2.0"
---

Lo avevo provato qualche tempo fa, quando era ancora alla seconda release alpha, e poi mi sono disinteressato al progetto e l'ho seguito da molto lontano. Ma finalmente, all'inizio di quest'anno, la prima release stabile, [ROS Ardent Apalone](https://github.com/ros2/ros2/wiki/Release-Ardent-Apalone) e io ho finalmente la possibilità di metterci le mani sopra!

## Aspetta un momento, di cosa si parla? Non eravamo a ROS Lunar?

Immagino che molti che non seguono assiduamente il mondo ROS stiano iniziando a farsi un po' di domande, perciò andiamo con ordine.

ROS nasce storicamente con un ben preciso obiettivo: **Fare Ricerca sulla Robotica**, o meglio, abilitare e semplificare la ricerca della robotica di servizio. La prima versione di ROS è nata quindi con l'università in testa, non l'industria, ed il progetto si è poi evoluto a partire da una base che aveva in mente questo obiettivo.

Probabilmente gli stessi ideatori originali di ROS non immaginavano quello che ROS sarebbe diventato e cosa avrebbe rappresentato: un progetto di dimensioni planetarie che esce dal mondo puramente universitario e approda nel mondo industriale, dove standard di funzionamento, robustezza e sicurezza sono necesssari. Contemporaneamente, l'evoluzione della ricerca, il crescente interesse e l'enorme community di **Robot Developers** creatasi intorno a ROS hanno dato vita a nuovi casi d'uso, e ROS ha iniziato a mostrare i primi segni di immaturità. Le critiche su ROS fioccano, e sono più o meno le seguenti:

 - non è Robusto
 - non è Real Time
 - è complesso da apprendere
 - non usa protocolli e pattern adeguati
 - è un casino quando lavori con più di due robot
 - è basato su Python 2
 - ecc.

E sono tutte critiche giustissime, che anche io, da sviluppatore ed entusiasta del progetto, noto ogni giorno.

Nel tempo, la community si è data da fare, e sono uscite varie patch che tentavano di risolvere i vari limiti che ROS ha:

 - Serve creare interfacce web per i robot? [Inventiamo ROSBridge](http://wiki.ros.org/rosbridge_suite).
 - Serve far comuncare ROS con microcontrollori? [Inventiamo ROSSerial](http://wiki.ros.org/rosserial).
 - Serve far lavorare insieme 20 robot ma evitare che se il master perde la connessione si rompa tutto? Inventiamo [Robot In Concert](http://wiki.ros.org/rocon).

Ad un certo punto, ci si è resi conto che era necessario un profondo ripensamento alla base di ROS, e è quindi nato ROS 2.0, che non è altro che una completa reimplementazione di ROS, partendo dalle stesse basi del progetto originale ma includendo le nuove tecnologie, i nuovi casi d'uso e dandogli un taglio più molto più robusto e industriale. Trovate qui un interessante articolo che [spiega nel dettaglio le motivazioni](http://design.ros2.org/articles/why_ros2.html).

Guardate che bella l'architettura :D

![ROS 2.0 Installazione](/assets/imgs/2018-03-01-benvenuto-ros2/ros_stack.png)


## ROS 2.0 Ardent Apalone, primi test. Ha senso migrare?

ROS 2.0 è in fare di sviluppo da circa 2 anni e mezzo, e finalmente, come detto sopra, è stata rilasciata la prima release stabile.

Ed io non ho perso tempo per installarla.

![ROS 2.0 Installazione](/assets/imgs/2018-03-01-benvenuto-ros2/install.png)

Iniziano ad esserci dentro un po' di chicche e carinerie varie:

 - Nativo su Python 3.0 (finally)
 - Supporto a molti linguaggi di programmazione, anche se la base è ancora C++ e Python
 - Linux RT
 - eccetera

Al momento mi sono limitato a leggere la doc e iniziare ad installarlo sul mio PC (nel momento in cui scrivo sto aspettando il download dei vari pacchetti). Ecco alcune cose che ho capito..

Al momento, ROS 2 include solo quello che in ROS è chiamato (core), cioè l'insieme dei funzionamenti base che abilitano la comunicazione e la gestione dei pacchetti. Da ora in avanti seguirà la parte di migrazione e sapremo probabilmente solo tra qualche mese se sarà approvato ed utilizzato dalla community (la fonza di ROS è la community, non ROS in se). 

Ancora quindi non ha troppo senso iniziare a guardarlo, ma certamente io lo farò per interesse e per divertimento. Se il progetto prenderà veramente il posto per cui è nato, cioè una naturale evoluzione del ROS originale, è sicuramente un'ottimo momento per iniziare a contribuirci!

:D

