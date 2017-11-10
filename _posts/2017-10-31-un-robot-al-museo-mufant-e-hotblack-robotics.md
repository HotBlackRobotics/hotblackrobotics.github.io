---
title: "Un robot al museo - MuFant e HotBlack Robotics"
layout: post
date: 2017-10-31 09:55:34
image: 
headerImage: false
tag: 
category: blog
redirect_from: /blog/post/2017-10-31-un-robot-al-museo-mufant-e-hotblack-robotics
author: sgabello
description: ""
---

Hey! Se sei finito qui è perchè probabilmente ti interessa avere qualche info in più. Se vuoi sapere:

**a) info sul come fare l'applicazione robotica per controllare in remoto un robot tramite un bot in Telegram**

**b) chi sono i "tizi" di HotBlack Robotics e perchè abbiamo concepito tutto ciò**

## Il robot che vedi alla mostra di Cesena

<iframe width="560" height="315" src="https://www.youtube.com/embed/9lyAfzyFcQQ" frameborder="0" gesture="media" allowfullscreen></iframe> 

## Il tutorial per rifare questo robot e l'applicazione a casa tua

Se hai dei problemi con questo tutorial o anche solo vuoi fare 4 chiacchiere scrivimi a ermacora.gabriele@gmail.com !!

<iframe width="560" height="315" src="https://www.youtube.com/embed/E9NX3vx4WSw" frameborder="0" gesture="media" allowfullscreen></iframe>

## RECAP: come fare questa applicazione robotica a casa tua 
Il tutorial per costruire questo robot controllabile da remoto è semplice e consiste in **18** semplici passaggi.

**Cosa ti serve**

Il robot DotBot che uso è open source e puoi costruirtelo a casa. Il tutorial dettagliato per stampare la meccanica in 3D, le schede elettroniche e come farlo lo trovi [qui](http://www.hotblackrobotics.com/blog/posts/2017-02-08-dotbot-tutorial-hardware). 

Se però non hai ancora tempo di costruirti tutto il robot puoi già partire solo con:

* [Raspberry Pi 3](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/)
* [RaspiCam](https://www.raspberrypi.org/products/camera-module-v2/)
* [Una SD Kingstone da 4 GB](https://www.amazon.it/Kingston-SDC4-4GB-MicroSDHC-Adattatore/dp/B000VX6XL6)
* Un alimentatore da cellulare Android per alimentare il Raspberry o un [power bank](https://www.amazon.it/RAVPower-Caricabatterie-Tecnologia-Universale-Smartphone/dp/B00YA01MC6/ref=sr_1_13?ie=UTF8&qid=1509721259&sr=8-13&keywords=ravpower+power+bank)

**NB: potete prendere ovviamente altri componenti purchè le specifiche di funzionamento siano rispettate.**

**NB2: occhio alle batterie scariche! Succede ogni tanto che se non alimentato bene il robot o non si connette o non fa streaming dalla camera o non si muove!**


**Partiamo**

1) vai sul sito [www.hotblackrobotics.com](http://www.hotblackrobotics.com/)

2) Dopo aver visto il mio (fantastico) breve video accedi in piattaforma. In alto a destra premendo ["Registrati"](http://www.hotblackrobotics.com/register) ti registri.

3) Una volta autenticato/a ci sarà un altro (ancora più fantastico) video. Vai sotto e clicca su [**Tutorial**](http://www.hotblackrobotics.com/blog/posts/supporto-tecnico)

4) Clicca sulla scritta arancione **["03-Scaricare HBrain - Immagine SD"](http://www.hotblackrobotics.com/blog/posts/2017-03-24-immagine-sd-per-la-cloud-e-configurazione)** e scarica l'immagine che va masterizzata sulla SD da inserire dentro il Raspberry Pi 3

5) Scarica l'immagine ed il programma per masterizzare [Etcher](https://etcher.io/). Masterizza sulla SD!

6) Collega il tuo raspberry ad un cavo Ethernet e collegalo al tuo router di casa

7) Accendi il Raspberry e dopo un po' (30 secondi - 1 minuto) [cerca il robot dalla piattaforma cloud](http://www.hotblackrobotics.com/cloud/index)

8) Ad un certo punto apparirà un robot che si chiama **hotbot**! Premi "connect". 

9) Ora vai su ["Skecthes"](http://www.hotblackrobotics.com/cloud/sketch/). Vai al fondo della pagina dove c'è scritto "Examples". Troverai un esempio che si chiama "Mufantbot".Premi "clone"!

10) Adesso il programmino è andato in "programs". Ora premi il bottone "Edit". Vedrai che si apre il codice.

11) Crea il tuo bot Telegram seguendo il [tutorial qui](http://www.hotblackrobotics.com/blog/posts/2017-02-16-tutorial-sviluppiamo-un-bot-telegram-in-ros). Segui tutti i passaggi fino a "Creazione del nostro programma" siccome tu il programma lo copi da me ;)

12) Vai nel codice e dove c'è scritto TOKEN a riga 18 sostituisci "il_tuo_token" con il tuo token. 

13) Ora inserisci la telecamera per il Raspberry (RaspiCam). La attivi andando su "Apps" sulla barra in alto e selezionando [RaspiCam](http://www.hotblackrobotics.com/cloud/webgui/camera)

14) Ora da questa pagina premi il pulsante ["Apri Manager Robot"](http://192.168.0.101:9001/).

15) Si apre la schermata con diversi processi e vai a schiacciare "start" su "ros_name". Aspetta 30 secondi.

16) Ora torna su RaspiCam (pagina di prima) e premi start camera. Il LED sulla camera si accende ed inizia lo streaming video.  

17) Ora torna sul tuo programmino e premi "RUN". Cerca su Telegram il tuo bot, con il nome che gli hai dato. 

18)Appena avvii la chat dovrebbe darti segni di vita!


## HOTBLACK ROBOTICS
HotBlack Robotics è una startup innovativa che offre una piattaforma di cloud robotics per facilitare la creazione di servizi e applicazioni robotiche. Il cuore della piattaforma è la community di **robot developers** in grado di inventare, sviluppare e condividere applicazioni e progetti robotici in modo facile e customizzzato.

Gli elementi chiave della piattaforma sono quindi i contenuti ovvero progetti hardware, software, i servizi, i processi, le metodologie e le esperienze da condividere on-line per favorire la condivisione tra gli utilizzatori della piattaforma (robot developers e end-users).

Perchè facciamo questo? Perchè crediamo fermamente che la condivisione open source di contenuti di robotica in una piattaforma cloud possa generare presto nuovi servizi robotici e creare nuovi business. Inoltre crediamo che i nuovi servizi robotici che si andranno a creare andranno a risolvere problemi specifici e customizzati nel mercato (customizzazione di massa).

![](https://static1.squarespace.com/static/5805c3c003596e2f7b9dd8f1/t/586cb9eee6f2e1c533074259/1483520506614/)



