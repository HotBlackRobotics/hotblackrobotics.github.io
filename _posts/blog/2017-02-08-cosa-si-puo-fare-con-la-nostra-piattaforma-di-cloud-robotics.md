---
title: "Cosa si può fare con la nostra piattaforma di Cloud Robotics"
layout: post
date: 2017-02-08 16:52:09
image: https://www.roboticsbusinessreview.com/wp-content/uploads/2011/07/Roboter-beziehen-Wissen-aus-der-Cloud.jpg
headerImage: false
tag:
 - Domande
 - Cloud
 - Robotics
category: blog
redirect_from: /blog/posts/2017-02-08-cosa-si-puo-fare-con-la-nostra-piattaforma-di-cloud-robotics
author: ludusrusso
description: "Ecco alcune cosa che è possibile fare con la nostra piattaforma di Cloud Robotics"
---

Da quando abbiamo iniziato a mostrare in giro la nostra piattaforma di Cloud Robotics, sempre più gente mi fa questa domanda: **Ma quindi, cosa si può fare con la piattaforma di Cloud robotics?**.

Scrivo questo post per avere un punto di riferimento a cui rimandare le persone che me la fanno (da buon informatico preferisco fare copia incolla che rifare ogni volta le stesse cose)....

Ecco una lista delle cose che attualmente la nostra piattaforma può fare :)

1. Programmazione remota di Robot
2. Controllo remoto
3. Riconoscimento e Sintesi Vocale
4. Applicazioni Multi Robot
5. Streaming Video e Computer Vision
6. Applicazioni di Robotica di Servizio


### Programmazione remota di Robot

La prima cosa che vedete della nostra piattaforma è un editor di testo che permette di sviluppare codice python e ROS in modo semplice e veloce, utilizzando un'interfaccia internet che permette anche di programmare remotamente il robot se raggiungibile tramite IP pubblico.

![Interfaccia Cluod Robotics](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486571402/Schermata_2017-02-08_alle_17.27.39_sl8vdb.png)

È anche possibile utilizzare dei Robot Virtuali in Cloud appositamente sviluppati per inziare a sviluppare in ROS senza avere necessariamente un hardware reale a disposizione, [trovate info qui](http://www.hotblackrobotics.com/blog/posts/2017-02-03-avete-problemi-hardware-ce-il-robot-in-cloud-accessibile-da-remoto-tramite-il-vostro-pc-o)

Perchè abbiamo scelto di utilizzare ROS?
I motivi sono due (più un terzo che però deriva dalla nostra esperienza):

1. È uno standard a livello accademico e industriale (Google/TIM assumono se sai programmare in ROS)
2. È stato sviluppato appositamente per la robotica
3. (opzionale) Io e Gabriele lo abbiamo studiato ed utilizzato per anni!

Conoscere ROS è quindi una *skill* che (secondo noi) sarà importante avere nel mondo lavorativo del futuro prossimo, un po' come adesso saper programmare Android/iOS apre tantissime possibilità nel mondo del lavoro!


### Controllo Remoto

Con controllo remoto, intendo la possibilità di controllare il robot mandado comandi tramite internet, o almeno all'interno di una rete locale. È stata una delle prime applicazioni che abbiamo sviluppato, perchè molte semplice da implementare con ROS ma anche di super effetto!

Abbiamo sviluppato (e stiamo sviluppando) diverse modalità di interazione con il robot, come ad esempio:

- JoyStick Virtuale
- Controllo da Tastiera
- Controllo da Cellulare con sensori di movimento

### Riconoscimento e Sintesi Vocale

Anche in questo caso, applicazioni molte semplice da implementare ma che fa capire benissimo le potenzialità. Seguendo i tutorial qui sotto, potete creare un vero e proprio assistente virtuale che controlla il vostro robot.

- Riconoscimento vocale (TO DO)
- [Sintesi vocale](http://www.hotblackrobotics.com/blog/posts/2017-02-02-hb-cloud-tutorial-speech-bot)

###Hardware Abstraction

A parte il nome che sembra complicato, l'idea dell'Hardware Abstrction è che la Cloud Robotics riesce ad isolare l'hardware dal software in un'applicazione robotica..

Ma cosa vuol dire?

In parole semplici: che voi potete progettare il vostro Hardware e utilizzare tutte le applicazioni già pronte in piattaforma senza dover implementare da zero tutto il software e l'intelligenza..

In parole tecniche: La Cloud Robotics è un protocollo che permette a tutti i dispositivi di parlare la stessa lingua..

Questo è il nostro progettino natalizio.

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/h13exqL9tbw" frameborder="0" allowfullscreen></iframe>



###Applicazioni di Robotica di Servizio

Quando diventerete bravi, potrete sviluppare la vostra applicazioni di [Robotica di Servizio e diventare dei veri e proprio RobotDeveloper](http://www.hotblackrobotics.com/blog/posts/2017-01-25-robotica)

Vi linko alcuni video di applicazioni sviluppate negli anni da noi:

####Robot@CED

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/HlUB0oHuXrc" frameborder="0" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/VH0q-UsDiQY" frameborder="0" allowfullscreen></iframe>

####PARLOMA

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/6MGJb_GqauU" frameborder="0" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/GS6_jwnSgWA" frameborder="0" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/EJ5-uBt7rHs" frameborder="0" allowfullscreen></iframe>
