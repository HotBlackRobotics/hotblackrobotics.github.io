---
title: "Il fondamento della piattaforma di Cloud Robotics: Robotics Operating System "
layout: post
date: 2017-03-14 22:54:50
image:
headerImage: false
tag:
 - ROS

redirect_from: /blog/posts/2017-03-14-il-fondamento-della-piattaforma-di-cloud-robotics-robotics-operating-system
author: ludusrusso
description: ""
lang: it
---

La base tecnologica su cui si basa la piattaforma di cloud robotics è [ROS (Robotic Operating System)] (http://wiki.ros.org/it). ROS è un framework software open source che permette lo sviluppo e la programmazione di robot. Fornisce le stesse funzioni di un sistema operativo come: astrazione dell'hardware, controllo dei dispositivi tramite driver, comunicazione tra processi, gestione delle applicazioni e altre funzioni di uso comune. Si presta particolarmente bene alle nostre esigenze legate all'internet delle cose poichè è un **sistema distribuito**, il che significa che diversi programmi sono distribuiti su robot differenti e comunicano tutti tramite la piattaforma.

Inoltre è particolarmente interessante perchè è utilizzato da tutti i principali sviluppatori software al mondo (sia accademici che industriali) come ad esempio Google, Stanford, ETH, MIT ecc.. Tant'è che è diventato lo *standard de fact0*.
Di conseguenza ha un enorme community molto competente pronta a risolvere problemi e bachi ad ogni momento e lo sviluppo di codice open source rende la comunità molto attiva.

Iniziamo a vedere come funziona!

Un’applicazione ROS è una rete di processi che scambiano dati in una rete di comunicazione composta da macchine diverse (gli oggetti dell'Internet dell cose o i robot).

* Nodo: un singolo processo (programma in esecuzione) all’interno della rete ROS
* Messaggio: struttura dati con cui usata per lo scambio di informazioni. Un messaggio può essere di diversi formati sia standard che custom. Ad esempio un messaggio che contiene un semplice intero sarà fatto così { int32 x }
* Topic: canale all’interno del quale due o più nodi si scambiano messaggi. Immaginate un topic come un canale di comunicazione dove i messaggi vengono trasmessi. Ogni topic usa il carattere *slash* prima del nome del topic (/nome_del_topic) ad esempio /chatter il topic dove due nodi si scambiano un semplice messaggio {String s}. La caratteristica importante e la potenzialità dei topic è che qualunque nodo può ascoltare (subscribe) o inviare messaggi (publish) sullo stesso topic in modo asincrono.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/ROScomm.png)

Nell'immagine sopra un esempio di comunicazione tra nodi attraverso i diversi topic. I nodi sono le figure ovali e i topic le frecce.
Rivediamo ora meglio i vari attori in piattaforma!

## Effettuate l'accesso alla piattaforma
Andate sul sito di Hotblack Robotics (http://www.hotblackrobotics.com/) ed entrate nella piataforma http://www.hotblackrobotics.com/login?next=%2Fcloud%2F . Inserite le vostre credenziali e siete in cloud!

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/cloudplatform.PNG)

La parte relativa la spiegazione di ROS la trovate nel menù in alto a sinistra sotto la voce "ROS".

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/Entratiincloud.PNG)

## Nodi ##
Un nodo è un processo (un programma in esecuzione) all’interno della rete ROS che esegue calcoli.
* Ogni nodo è identificato da un nome unico nella rete
* I nodi si scambiano messaggi per interagire tra loro tramite i topic
* Ad ogni nodo è associato un compito. Due categorie
    - Driver (controlla un sensore o attuatore
    - Elaborazione (esegue calcoli)

In piattaforma trovate sempre sotto la voce "ROS" (http://www.hotblackrobotics.com/cloud/webgui/console) sotto la voce "Nodes List" la lista dei nodi attivi.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/nodi.PNG )

## Topic ##
I topic sono i canali attraverso i quali i nodi comunicano nella rete.
* Ogni topic è identificato da un nome unico
* Un nodo che invia dati su un topic è detto *publisher*
* Un nodo che riceve dati su un topic è detto *subscriber*

Sotto alla sezione "Nodes List" trovate la lista dei topic ("Topics List").

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/topics.PNG)

## I messaggi ##
I messaggi sono i tipi di dati che vengono inviati attraverso il topic
* Informano i nodi su come interpretare i bites scambiati nei topic
* Ad ogni topic è associato un messaggio specifico
* Possono essere tipi semplici (**int, float, bool**) o **strutture dati**

## Namespace ##
Per distinguere nodi e topic con lo stesso nome si usano i namespace.
* Prefisso da applicare ai topic e nodi
* Definiscono una sottorete della rete ROS
* Utili per distinguere lo stesso nodo/topic riferito a macchine diverse ma con la stessa funzione
