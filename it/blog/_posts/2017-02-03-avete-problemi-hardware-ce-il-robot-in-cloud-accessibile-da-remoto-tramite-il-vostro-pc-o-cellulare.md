---
title: "Avete problemi hardware? C'è il robot in cloud accessibile da remoto tramite il vostro PC o cellulare"
layout: post
date: 2017-02-03 17:58:13
image: http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486136271/istanza_cloud_1_tdt5ho.jpg
headerImage: false
lang: it
tag:
 - Robotics
 - Cloud
 - ROS
 - Iot
 - Amazon
 - Aws

redirect_from: 
 - /2017/02/03/avete-problemi-hardware-ce-il-robot-in-cloud-accessibile-da-remoto-tramite-il-vostro-pc-o-cellulare/
 - /blog/posts/2017-02-03-avete-problemi-hardware-ce-il-robot-in-cloud-accessibile-da-remoto-tramite-il-vostro-pc-o-cellulare
author: sgabello
description: "Avete problemi hardware? C'è il robot in cloud accessibile da remoto tramite il vostro PC o cellulare"
---

Molti di voi vorrebbero iniziare subito a programmare in ROS tramite la piattaforma cloud ma sono bloccati (purtroppo) da rognosi problemi hardware. Probabilmente la scheda motori non funziona correttamente o state aspettando ancora il Raspberri Pi o un componente da qualche fornitore. Perchè quindi non saltate questo passaggio e iniziate a programmare subito in ROS con un robot remotizzato in cloud? Cosa significa esattamente lo affronteremo in questo post, ma in breve significa che potete iniziare ad usare la piattaforma soltanto con il vostro pc e una qualsiasi connessione internet (senza Raspberry Pi e altri dispositivi)! E... tutto ciò che abbiamo visto su come configurare una rete per Dotbot? Beh, ora abbiamo accesso ad un Dotbot in cloud!

In pratica tutto quello che avete visto funzionare su Dotbot è da oggi disponibile in piattaforma cloud e accessibile tramite browser. Vi spiego com'è possibile, sul Raspberry Pi di Dotbot c'è una versione di Linux con installato ROS che è configurato per collegarsi alla nostra piattaforma cloud. Quello che abbiamo fatto è prendere un computer Linux accessibile da Internet ed installarci sopra tutto il software che solitamente installiamo su Dotbot. Per i più tecnici in pratica abbiamo comprato un'istanza cloud da Amazon Web Services ed installato l'immagine Dotbot esattamente come facciamo su Raspberry, questa si che è cloud robotics!

![cloud robotics amazon](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486136271/istanza_cloud_1_tdt5ho.jpg)

Lo schema sopra riassume questi concetti e vi mostra come i due mondi si interfacciano tramite la piattaforma. La cosa che più mi affascina della filosofia cloud è che si confonde la differenza tra mondo "fisico", ovvero ciò che possiamo "toccare", con quello cloud. Infatti a qualcuno verrebbe da pensare che in sostanza abbiamo creato soltanto un sofisticato simulatore.. no, niente di più sbagliato! Il robot in cloud, DotbotCloud, esiste davvero ed è esattamente come Dotbot. L'unica differenza è che il computer Linux di DotbotCloud è una macchina virtuale remotizzata da Amazon e nessuno con certezza può sapere fisicamente dove si trovi. Inoltre ovviamente non può avere nessun tipo di interazione con il mondo fisico. In pratica è come se DotbotCloud vivesse in un mondo parallelo. Quello che possiamo fare è usarlo come fosse un robot normale per poi riportare il nostro software su un robot "concreto" e il funzionamento sarà lo stesso! Vediamo subito cosa ci possiamo fare.

## Primi passi con Dotbot in cloud ##

Entriamo in piattaforma (effettuando il login) e cerchiamo il robot in cloud con il tasto "cerca robot". Nel mio caso ho un indirizzo ip così *54.191.14.121:8080* .

![cloud robotics](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486144089/Schermata_2017-02-03_alle_18.07.33_blhaox.png)

Ora andiamo nella "ROS console" e vediamo cosa succede.

![ros cloud](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1486144458/Schermata_2017-02-03_alle_18.52.51_fags1l.png)
