---
title: "Implementiamo un subscriber con un interruttore"
layout: post
date: 2017-03-24 14:14:36
image:
headerImage: false
lang: it
tag:
 - Tutorial

redirect_from: 
 - /2017/03/24/implementiamo-un-subscriber-con-un-interruttore/
 - /blog/posts/2017-03-24-implementiamo-un-subscriber-con-un-interruttore
author: sgabello
description: ""
---

Ora utilizziamo un po' di più l'hardware! Configuriamo un pin di input per dare il segnale ad un led per accendersi. Il codice a questo punto è molto semplice. Importiamo un messaggio nuovo che è Input. Questo è il messaggio che pubblicherà l'interruttore quando verrà premuto. Quindi abbiam un subscriber che rimane in ascolto, e ogni volta che l'interruttore viene premuto il subscriber richiamerà la funzione `on_input`. Dentro questa funzione in fine c'è un publisher che farà accendere i led  a frequenze diverse. C'è un trick inoltre che è led_msg.led1 = self.cnt % 2 == 1. Questa è la versione compatta del if-then-else che abbiamo visto prima per cambiare continuamente stato al lede da true a false. La sintassi in modo compatto è dividi self.cnt per 2 se da resto 0 allora o (False) se no è 1 (True).

```python
import dotbot_ros
from dotbot_msgs.msg import Input, Led
import sys

class Node(dotbot_ros.DotbotNode):
    node_name = 'input_node'
    def setup(self):
        self.cnt = 0
        dotbot_ros.Subscriber('input', Input, self.on_input)
        self.led_pub = dotbot_ros.Publisher('led', Led)
        print 'Input Node Started'
        sys.stdout.flush()

    def on_input(self, msg):
        if msg.input1 == True:
            self.cnt += 1
            led_msg = Led()
            led_msg.led1 = self.cnt % 2 == 1
            led_msg.led2 = (self.cnt/2) % 2 == 1
            led_msg.led3 = (self.cnt/4) % 2 == 1
            self.led_pub.publish(led_msg)
            print "pressed"
            sys.stdout.flush()
```
