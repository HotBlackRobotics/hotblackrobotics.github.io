---
title: "Tutorial - Usiamo la piattaforma di Cloud Robotics per sviluppare un semplice assistente personale Robotico"
layout: post
date: 2017-02-10 17:48:31
image: http://www.cups.it/wp-content/uploads/2017/01/robot-assistant.jpg
headerImage: false
tag: 
 - Cloud
 - Speech
 - To
 - Robot
 - Recognition
 - Text
category: blog
redirect_from: /blog/posts/2017-02-10-tutorial-usiamo-la-piattaforma-di-cloud-robotics-per-sviluppare-un-semplice-assistente-personale-robotico
author: sgabello
description: "Usiamo la piattaforma di Cloud Robotics per sviluppare un semplice assistente personale Robotico"
---

Ciao a tutti, iniziamo con questo post una serie di tutorial più completi per lo sviluppo di semplici applicazioni Robotiche. In particolare, in questo tutorial vedremo come sfruttare la nostra piattaforma per sviluppare un'applicazione che permetta di inviare comandi al robot tremite voce e ricevedere informazioni vocali dal robot stesso.

##Cosa serve?

Per sviluppare questo progetto, vi servirà essere iscritti alla nostra piattaforma ed avere a disposizione un robot [reale](http://www.hotblackrobotics.com/blog/posts/2017-02-08-dotbot-tutorial-hardware) o [virtuale (tramite la cloud)](http://www.hotblackrobotics.com/blog/posts/2017-02-03-avete-problemi-hardware-ce-il-robot-in-cloud-accessibile-da-remoto-tramite-il-vostro-pc-o). 

In questo tutorial, per semplicità, useremo un robot virtuale ma ovviamente si può benissimo utilizzare il robot reale!

Se volete utilizzare un robot virtuale scriveteci a info@hotblackrobotics.com!

##Iniziamo: accediamo alla piattaforma e colleghiamoci al Robot

Accediamo al sito [www.hotblackrobotics.com](http://www.hotblackrobotics.com) ed effettuiamo il Login

![sito hotblack robotics](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486657695/Schermata_2017-02-09_alle_17.24.27_o2js8p.png)


Effettuiamo il Login con le nostre credenziali
![Effettuo Login](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486657695/Schermata_2017-02-09_alle_17.24.36_ur1zvl.png)

Accediamo alla piattaforma premendo sul tab *cloud*
![Accesso alla piattaforma cloud](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486657695/Schermata_2017-02-09_alle_17.24.49_crt92p.png)

A questo punto, possiamo collegarci al robot.. Insieriamo il nome o l'indirizzo IP del robot e connettiamoci.

![Inserimento IP Robot](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486662736/Schermata_2017-02-09_alle_18.05.03_gorkzs.png)

Una volta cliccato "Cerca Robot", se tutto va bene, otterremo il seguente messaggio!

![Robot Connesso](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486662843/Schermata_2017-02-09_alle_18.05.08_dwgqnj.png)

##Creiamo il nostro programma

Una volta connesso il robot, siamo pronti ad iniziare a sviluppare il programma! A questo punto andiamo sul tab *sketches*.

![iniziamo a programmare](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486746986/Schermata_2017-02-10_alle_18.15.58_chnyyy.png) 

Creiamo un nuovo programma chiamato "assistente robotico" e successivamente premiamo il bottone "new".

![assistente robbotico](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486747155/Schermata_2017-02-10_alle_18.18.40_elmjy5.png)

Apriamo il file con il tasto "edit" e scriviamo il seguente programma.

```
import dotbot_ros
from std_msgs.msg import String

class Node(dotbot_ros.DotbotNode):
    node_name = 'speech_bot_example'
    
    def setup(self):
        self.pub_speech = dotbot_ros.Publisher('to_speech', String)
        dotbot_ros.Subscriber('speech', String, self.on_speech)

    def on_speech(self, msg):
        if msg.data == 'ciao':
            self.pub_speech.publish("Ciao, come va?")
        elif msg.data == 'mondo':
            self.pub_speech.publish("Vuoi dire Ciao Mondo?")
```

![programma](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486747850/Schermata_2017-02-10_alle_18.28.56_jh1gsi.png)

Nel programma stiamo definendo un *Publisher* di nome ```pub_speech``` che pubblica sul topic ```to_speech``` e invia un messaggio di tipo "String" ed un *Subscriber* che richiama la call back ```on_speech``` chiamata ogni volta che sul topic ```speech``` arriva un messaggio di tipo "String". Ora aprendo la Web App "Speech Rec" e abbilitando il microfono del vostro computer, tramite il tasto centrale, quando pronuncicamo la parola "ciao" il robot ci risponderà "Ciao, come va?" e pronunciando "mondo" il robot risponderà "Vuoi dire ciao mondo?".

NB: ovviamente dovrete inserire nella casella "inserisci" le parole "ciao" e "mondo" altrimenti il robot non riconoscerà le parole!

![](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486748553/Schermata_2017-02-10_alle_18.41.50_ury16w.png) 

Provate ad inserire altre parole e personalizzate il vostro assistente robotico!