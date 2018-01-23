---
title: "Tutorial - Sviluppiamo un Bot Telegram in ROS"
layout: post
date: 2017-02-16 14:02:45
image: http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487253704/Schermata_2017-02-16_alle_15.01.01_k4kh7s.png
headerImage: false
lang: it
tag:
 - Tutorial
 - Ros
 - Telegram
category: blog
redirect_from: /blog/posts/2017-02-16-tutorial-sviluppiamo-un-bot-telegram-in-ros
author: ludusrusso
description: ""
---

In questo tutorial vedremo come creare un semplice bot telegram in grado di controllare un robot in cloud.

##Cosa serve?

Per sviluppare questo progetto, vi servirà essere iscritti alla nostra piattaforma ed avere a disposizione un robot [reale](http://www.hotblackrobotics.com/blog/posts/2017-02-08-dotbot-tutorial-hardware) o [virtuale](http://www.hotblackrobotics.com/blog/posts/2017-02-03-avete-problemi-hardware-ce-il-robot-in-cloud-accessibile-da-remoto-tramite-il-vostro-pc-o).

Questa volta, useremo un robot reale per dialogare con il robot! Ad ogni modo, potrete comunque richiedere il vostro robot virtuale per fare prove!
Se volete utilizzare un robot virtuale scriveteci a info@hotblackrobotics.com!

Per prima cosa, è importate iscriversi alla piattaforma e collegarsi al robot, potete seguire le istruzioni all'inizio di [questo tutorial](http://www.hotblackrobotics.com/blog/posts/2017-02-10-tutorial-usiamo-la-piattaforma-di-cloud-robotics-per-sviluppare-un-semplice-assistente-personale-robotico).

##Creazione di un Bot Telegram

Per prima cosa, è necessario configurare un bot telegram! Per farlo, telegram mette a disposizione un bot (chiamato **BotFather**) in grado di creare altri bot.

Assicuriamoci di aver installato telegram sul nostro dispositivo e accediamo al [BotFather cliccando qui](https://telegram.me/BotFather). Una volta aperta la chat con **BotFather** lui ci informerà (in inglese) sulle azioni che possiamo fare.

![BotFather primo contatto](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487249723/Schermata_2017-02-16_alle_13.50.11_cnqybw.png)

Per creare un nuovo bot, inviamo il comando `/newbot`, a cui il **BotFather** risponderà chiedendo che nome dare al proprio bot

![Diamo un nome al bot](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487249723/Schermata_2017-02-16_alle_13.50.05_ru42nq.png)

Una volta scelto il nome (**BotFather** ci informa se il nome è già stato preso), verrà creato il nostro bot e ci verranno fornite due informazioni essenziali:

- l'URL del bot, grazia al quale potremmo aprire la chat
- il Token del bot (**da mantenere segreto**) che servirà per sviluppare il codice.

![Bot Creato](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487249723/Schermata_2017-02-16_alle_13.50.19_zvxpdf.png)

Una volta creato il bot, cliccando sull'URL, potremmo aprire la chat! Ovviamente adesso non succederà nulla perchè il bot non è ancora stato implementato.

![Bot Chat](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487249723/Schermata_2017-02-16_alle_13.54.43_xnqcvr.png)

##Creazione del nostro programma

Una volta connessi, creiamo un nuovo programma chiamato *telebot*, seguendo le istruzioni di seguito.

Dopo esserci loggati in piattaforma, apriamo il tab **sketches**.
![aprire sketches](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487253463/Schermata_2017-02-16_alle_13.44.25_zpckl8.png)

Creiamo un nuovo scketch e apriamolo

![nuovo sketch](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487253464/Schermata_2017-02-16_alle_13.44.32_zljmyj.png)
![aprire sketch](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487253460/Schermata_2017-02-16_alle_13.44.37_toe9mx.png)

##Implementiamo un semplice Bot

A questo punto, siamo pronti ad implementare un semplicissimo bot che risponde alla nostra chat.
Nel programma appena crato, implementiamo il seguente codice.

```python
import dotbot_ros
import telepot

import sys

class Node(dotbot_ros.DotbotNode):
    node_name = 'bot'
    TOKEN = "INSERISCI QUI IL TUO TOKEN"

    def setup(self):
        self.bot = telepot.Bot(self.TOKEN)
        self.bot.message_loop(self.handle)

    def handle(self, msg):
        content_type, chat_type, chat_id = telepot.glance(msg)

        if content_type == 'text':
            cmd = msg['text'].split()
            if cmd[0] == '/start':
                self.bot.sendMessage(chat_id, "ciao, benvenuto nella mia chat!")
            elif cmd[0] == '/ciao':
                self.bot.sendMessage(chat_id, "ciao, come stai?")
            else:
                self.bot.sendMessage(chat_id, "scusa, non capisco, conosco solo il comando '/ciao'")
        print msg
        sys.stdout.flush()
```

**IMPORTANTE: ricordate di modificare la linea `TOKEN = "INSERISCI QUI IL TUO TOKEN" ` inserendo il token del vostro robot tra i doppi apici**

![codice bot implementato](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487250887/Schermata_2017-02-16_alle_14.12.49_il2g6h.png)

Una volta implementato il programma, salviamo e lanciamo il codice! A questo punto, se tutto va bene, potremmo aprire la chat telegram col nostro bot e iniziare a dialogare con lui!

![programma lanciato](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487250887/Schermata_2017-02-16_alle_14.12.41_b4jbal.png)

![bot dialogo](http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487250886/Schermata_2017-02-16_alle_14.09.45_dkn7v0.png)

##Analizziamo il codice

Come al solito, il nostro programma è composto da un nodo ROS, la funzione princiapale è la funzione `setup`, che si occupa di inizializzare il bot e creare una callback di gestione.

```python
    def setup(self):
        self.bot = telepot.Bot(self.TOKEN)
        self.bot.message_loop(self.handle)
```

In particolare, la linea `self.bot = telepot.Bot(self.TOKEN)` crea il bot utilizzando il nostro token, mentre la seconda linea `self.bot.message_loop(self.handle)` dice che, ogni volta che un nuovo messaggio viene mandato al bot, bisogna chiamare la funzione `self.handle`.


La funzione `handle`, quindi, viene chiamata quando un nuovo messaggi è mandato alla nostra chat, ed.è implementata come segue:

```python
    def handle(self, msg):
        content_type, chat_type, chat_id = telepot.glance(msg)

        if content_type == 'text':
            cmd = msg['text'].split()
            if cmd[0] == '/start':
                self.bot.sendMessage(chat_id, "ciao, benvenuto nella mia chat!")
            elif cmd[0] == '/ciao':
                self.bot.sendMessage(chat_id, "ciao, come stai?")
            else:
                self.bot.sendMessage(chat_id, "scusa, non capisco, conosco solo il comando '/ciao'")
        print msg
        sys.stdout.flush()
```

Alla funzione viene passato come parametro il messaggio mandato alla chat, contenuto nella variabile `msg`.

Per prima cosa, è necessario estrarre informaizoni utili dalla chat, in particolare il `chat_id` (che identifica univocamente la chat aperta, in modo che il bot possa gestire più chat contemporaneamente), e il `content_type`, cioè il tipo di dati contenuti nel messaggio.
Questa operazione viene fatta dalla riga `content_type, chat_type, chat_id = telepot.glance(msg)`.

A questo punto, dobbiamo verificare che il messaggio sia di tipo testuale (il bot al momento non sa gestire immagini, video o file). Facciamo quindi il check `if content_type == 'text':`.

Se il messaggio è di tipo testuale, possiamo andare ad analizzarlo. Il messaggio sarà contenuto in una stringa `msg['text']` di testo. Possiamo dividere la stringa nelle parole contenute utilizzando il metodo `plit()`, come nella riga `cmd = msg['text'].split()`. In questo modo, se la stringa è `"ciao come stai?"`, la variabile `cmd` sarà uguale a `['ciao', 'come', 'stai?']`.

Ora possiamo analizzare la prima parola, rispondendo in modo diverso in base al valore di questa. Al momento, il robot risponde solamente a due parole: `/ciao` e `/start` (notare lo `/`). Questo viene fatto all'interno del costrutto `if ... elif ... else`.

Per rispondere alla chat, utilizziamo il la riga `self.bot.sendMessage(chat_id, "ciao, come stai?") `, dove il primo parametro è l'id della chat stessa, mentre il secondo parametro è la stringa da mandare.

###Esercizio

Provate ad implemntare altri comandi!

##Controlliamo il robot da chat telegram!

Una volta capito come mandare comandi al robot, possiamo tranquillamente implementare dei comandi per farlo muovere. Per farlo, bisogna modificare il nostro programma come segue.

1. Importare dalla libreria gpiozero l'oggetto Robot
2. Creare un oggetto `Robot` nella funzione di `setup`
3. Implementare i comandi nel costrutto `if ... elif ... else`
4. Nei nuovi comandi, controllare i motori tramite l'oggetto Robot.

Il nuovo codice implementato è il seguete

```python
import dotbot_ros
import telepot

from gpiozero import Robot
import time
import sys

class Node(dotbot_ros.DotbotNode):
    node_name = 'bot'
    TOKEN = "INSERISCI QUI IL TUO TOKEN"

    def setup(self):
        self.bot = telepot.Bot(self.TOKEN)
        self.bot.message_loop(self.handle)
        self.robot = Robot(left=(9, 10), right=(7, 8))


    def handle(self, msg):
        content_type, chat_type, chat_id = telepot.glance(msg)

        if content_type == 'text':
            cmd = msg['text'].split()
            if cmd[0] == '/start':
                self.bot.sendMessage(chat_id, "ciao, benvenuto nella mia chat!")
            elif cmd[0] == '/ciao':
                self.bot.sendMessage(chat_id, "ciao, come stai?")
            elif cmd[0] == '/avanti':
                self.bot.sendMessage(chat_id, "ok, vado avanti")
                self.robot.forward()
                time.sleep(0.5)
                self.robot.stop()
            elif cmd[0] == '/dietro':
                self.bot.sendMessage(chat_id, "ok, vado dietro")
                self.robot.backward()
                time.sleep(0.5)
                self.robot.stop()
            elif cmd[0] == '/destra':
                self.bot.sendMessage(chat_id, "ok, giro a destra")
                self.robot.right()
                time.sleep(0.5)
                self.robot.stop()
            elif cmd[0] == '/sinistra':
                self.bot.sendMessage(chat_id, "ok, giro a sinistra")
                self.robot.left()
                time.sleep(0.5)
                self.robot.stop()
            else:
                self.bot.sendMessage(chat_id, "scusa, non capisco, conosco solo il comando '/ciao'")
        print msg
        sys.stdout.flush()
```

Provate quindi a rilanciare il programma e far muovere un robot. Ecco qui un video che mostra il risultato finale!

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/uoY_GEP8YFw" frameborder="0" allowfullscreen></iframe>
