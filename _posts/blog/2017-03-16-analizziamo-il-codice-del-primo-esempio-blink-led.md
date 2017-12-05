---
title: "Analizziamo il codice del primo esempio - blink LED"
layout: post
date: 2017-03-16 22:01:37
image: 
headerImage: false
tag: 
 - Tutorial
category: blog
redirect_from: /blog/posts/2017-03-16-analizziamo-il-codice-del-primo-esempio-blink-led
author: sgabello
description: ""
---

Analizziamo il codice di `dotbot_led_cnt`.
Se volete copiarlo qui:

```
import dotbot_ros
from dotbot_msgs.msg import Led
import sys

class Node(dotbot_ros.DotbotNode):
    node_name = 'led_cnt'
    def setup(self):
        self.led_pub = dotbot_ros.Publisher('led', Led)
        self.loop_rate = dotbot_ros.Rate(2)
        self.cnt = 0
        print 'setup'

    def loop(self):
        self.cnt += 1
        msg = Led()
        if self.cnt % 2 == 0:
            msg.led1 = True
        else:
            msg.led1 = False
        self.led_pub.publish(msg)
        print 'cnt', self.cnt
        sys.stdout.flush()
```
Iniziamo ad analizzare alcune righe di codice.
Queste prime righe significano che alcune librerie di Python devono essere aggiunte, in particolare il messaggio di tipo **Led** deve esssere importato nel codice.
```
from dotbot_msgs.msg import Led
```
Poi dichiariamo il Nodo, con la dicitura `class Node(dotbot_ros.DotbotNode):` e il nome del nodo con `node_name = 'led_cnt'` .
In seguito abbiamo due funzioni principali una ` def setup(self):` chiamata solo una volta all'inizio dell'esecuzione del programma e un'altra che viene eseguita all'infinito ` def loop(self):`. Quest'ultima viene eseguita ad una frequenza impostata nella funzione setup con il comando `self.loop_rate = dotbot_ros.Rate(2)`.NOTA BENE self è una parola magica che non va mai dimenticata! In questo caso la frequenza è di 2Hz. **NOTA BENE se non impostate la frequenza con questo comando la funzione loop non sarà mai richiamata!**

Con la funzione `self.led_pub = dotbot_ros.Publisher('led', Led)` definiamo un Publisher che pubblica sul topic "led" il messaggio di tipo Led.

Nella funzione loop invece istanziamo il messaggio msg di tipo Led con `msg = Led()`, gli assegniamo un valore che alterna tra True e False. Incrementiamo all'inizio del loop la variabile `self.cnt += 1`. Poi entriamo nel if e calcoliamo il resto di self.cnt diviso per 2 con l'operatore %. Se il resto è uguale a 0 allora riempiamo il messaggio msg con True e il led si accenderà altrimenti il contrario.
Infine pubblichiamo il messaggio con `self.led_pub.publish(msg)` scriviamo a schermo il valore di cnt. La funzione sys.stdout.flush() serve  a scrivere a schermo.

