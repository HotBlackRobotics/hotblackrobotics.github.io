---
title: "HB Robotics e ITIS Avogadro: resoconto della prima settimana"
layout: post
date: 2017-01-23 14:24:24
image: http://res.cloudinary.com/hbr/image/upload/v1485173582/IMG_20170118_114658-PANO_ye4xbh.jpg
headerImage: false
lang: it
tag:
 - Scuola
 - ROS
 - Avogadro
 - Hbr

redirect_from: 
 - /2017/01/23/hb-robotics-e-itis-avogadro-resoconto-della-prima-settmana/
 - /blog/posts/2017-01-23-hb-robotics-e-itis-avogadro-resoconto-della-prima-settmana
author: ludusrusso
description: "Il resoconto della prima settimana di alternanza Scuola-Lavoro presso la scuola ITIS Avogadro di Torino"
---

![HB Robotcs Spiegazione ROS](http://res.cloudinary.com/hbr/image/upload/v1485173582/IMG_20170118_114658-PANO_ye4xbh.jpg)

Come sapete, settimana scorsa abbiamo iniziato un progetto con l'istituto ITIS Avogadro di Torino, che ci vede coinvolti per almeno 52 con 3 classi quarte del liceo.

Due giorni fa, abbiamo concluso la prima settimana di lavoro con i ragazzi, ed in particolare abbiamo concluso la prima parte del corso che prevedeva la costruzione e il test dei 21 robot **DotBot-ROS** che abbiamo fornito alla scuola. Dopo una domenica di riposo (ci voleva) sono pronto a raccontarvi come sono andati questi giorni.

## Costruzione del Robot

La costruzione del robot è stata certamente la fase che ha più di tutte impegnato i ragazzi in questi giorni. I gruppo di lavoro (composti da 3/4 studenti ciascuono) hanno impiegato circa 4 ore per completare la struttura meccanica del robot. Abbiamo verificato che alcuni passaggi del montaggio possono essere semplificati non poco con alcuni accorgimenti nella meccanica del robot, che andremo ad implementare nei prossimi mesi.
Aspettatevi quindi una versione 2.0 di DotBot!

![Costruzione Robot](http://res.cloudinary.com/hbr/image/upload/v1485174212/collage-2017-01-23_xpguwt.png)

## Inizializzazione del Robot

L'inizializzazione del robot è stata sicuramente la parte meno interessante del percorso fatto fino ad adesso. Ci ha però permesso di venire a capo di alcuni problemi non previsti del nostro progetto.

### Copia della SD
Fortunatamente, dopo un piccolo intoppo che siamo riusciti a risolvere in fretta, la procedura di copia dell'immagine SD sulle SD reali è andata a buon fine su tutti i robot.

### Dare un nome ai Robot
Il dare un nome al robot è stata certamente la parte più divertente di tutto. Per distinguere i 21 robot connessi alla stessa rete, abbiamo deciso di dare la possibilità ai ragazzi di dare un nome ai robot per poterli univocamente distinguere tra loro. Il problema è che di default, i robot hanno tutti lo stesso nome `hotbot`. In fase di configurazione, quindi, è successo un bel po' di volte che i ragazzi si sono messi a lavorare su robot di un altro gruppo senza rendersene subito conto. I risultati sono stati molto divertente, ovviamente al momento stiamo indagando sul modo migliore per riconoscere i robot appena configurati! :D

## Programmazione dei robot

La programmazione in ROS è iniziata subito dopo la fase di configurazione. Per il momento, non abbiamo avuto troppo tempo per far lavorare i ragazzi (il tempo rimasto era molto poco), ma ho trovato il tempo per dargli un'infarinatura generale di ROS e della piattaforma.

I ragazzi erano super interessati, e di questo sono molto contento. Temevo tantissimo che trovassero il tutto troppo complicato per padroneggiarlo nel poco tempo disponibile, però in poco più di mezzora ogni gruppo già smanettava per programmare i robot in modo da accendere led e leggere i valori di pulsanti.

## OpenDay: un evento inaspettato

Durante la giornata di sabato, si è svolto l'OpenDay della scuola. Questo evento serve per aprire le porte a potenziali studenti, per far capire ai nuovi studenti se la scuola fa per loro. La classe con cui lavoriamo è stata coinvolta attivamente in questo evento. Durante la lezione, quindi, molti ragazzi erano preoccupati per la presentazione dei progetti che sarebbe avvenuta nel pomeriggio.
In accordo con loro, quindi, abbiamo deciso di lasciarli un po' più liberi nell'ultimazione dei loro progetti.

Uno di loro, ci ha proposto di montare il nostro Raspberry PI su un Hardware diverso da loro: una macchina telecomandata che stavano cercando di controllare in Arduino.
Il progetto mi sembrava divertente, e nel pochissimo tempo a disposizione siamo riusciti a configurare correttamente il robot e a controllare la macchina via wireless da una webapp!

Mi sono reso conto delle enormi potenzialità della tecnologia che stiamo sviluppando: crediamo che ROS (e la Cloud Robotics) potrà essere quello che adesso è Android (e gli smartphone): una nuova tecnologia che rivoluzionerà la nostra società e creerà una classe di nuovi professionisti chiamati **robot developers** (sviluppatori robotici).

![Macchina Dotbot](http://res.cloudinary.com/hbr/image/upload/v1485181164/IMG_20170121_120500_oxnih7.jpg)

![Open Day Avogadro](http://res.cloudinary.com/hbr/image/upload/v1485182225/IMG_20170121_154621_uixquu.jpg)
