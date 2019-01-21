---
title: "SpiderBot Cloud 2.0 la vendetta - 2 gradi di libertà e controllo vocale"
layout: post
date: 2017-03-16 22:03:56
image:
headerImage: false
lang: it
tag:
 - Hacking

redirect_from:
 - /2017/03/16/spiderbotcloud-20-la-vendetta-2-gradi-di-liberta-e-controllo-vocale/
 - /blog/posts/2017-03-16-spiderbotcloud-20-la-vendetta-2-gradi-di-liberta-e-controllo-vocale
author: sgabello
description: ""
---

Dopo il tutorial "Come collegare un robot comprato da Tiger (Spider Robot) in piattaforma cloud" vi avevo promesso che avrei comprato un secondo ragno robotico da Tiger così da costruire un robot in grado di muoversi in ogni direzione.
Il materiale utilizzato è lo stesso del tutorial precedente ma con una batteria power bank in più.

Lista:

* 2 spider robot comprati da Tiger
* 2 [power bank per cellulare da 5 Volt](http://www.dx.com/p/cylinder-shaped-external-6000mah-emergency-power-battery-charger-for-iphone-cell-phone-silver-206652#.WFpnUrbhB-V)
* un ponte ad H. Io ho usato [questo](http://eud.dx.com/product/hg7881-two-channel-motor-driver-board-dark-blue-2-5-12v-2-pcs-844407060) ma anche in questo caso potete scegliere quello che volete. Qualcuno li costruisce anche a mano mettendo insieme 4 transistor!
* un Raspberry PI 3
* cavetti
* fascette da idraulico
* un pezzo di cartone ;)
* 2 cavetti USB a microUSB (alimentatore per cellulare Android)

Per prima cosa montate i due Spider Robot come dalle istruzioni di Tiger. Poi smontate una parte dei robot e uniteli insieme come in figura.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/spiderbot_pics/1.jpeg)

Unite i cavetti di alimentazione dei motori ai connettori delle batterie. Dopo uniremo ai connettori delle batterie altri cavetti al ponte H.
Ritagliate un pezzo di cartone di questa forma con due linguette da inserire dentro i supporti di plastica dei due robot.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/spiderbot_pics/2.jpeg)

Poi pinzate le linguette in modo da fissarle.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/spiderbot_pics/3.jpeg)

A questo punto fissate con delle fascette da idraulico tutto il sistema composto da Raspberry pi + ponte ad H + le 2 batterie.

Così:
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/spiderbot_pics/4.jpeg)
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/spiderbot_pics/5.jpeg)

Ora effettuiamo i collegamenti.

I pin del Raspberry sono:

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/RP2_Pinout%20(1).png)

Il ponte ad H:

![](https://i.ytimg.com/vi/mKfTi3iD518/maxresdefault.jpg)

Ora collegate ogni alimentazione dei motori (motere A e motore B) ai rispettivi mammut del ponte ad H. Poi collegate il controllo dei motori (A-1A, A-1B e B-1A, B-1B) con i GPIO 9,25 (pin 21,22 o contando 10 dal basso) e GPIO 22,23 (pin 15,16 o contando 13 dal basso)del Raspberry. Infine l'alimentazione la collegate a una delle due batterie (fate attenzione solo che abbia almeno 1 o 2 Ampere altrimenti i motori non hanno abbastanza potenza e non si muovono).

Ora configurate il Raspberry come da [http://hotblackrobotics.github.io/forum/support/13](http://hotblackrobotics.github.io/forum/support/13).

E siete pronti a partire!
Se volete usare il controllo vocale copiate il codice da qui e usatelo tramite la Web App come spiegato [qui]({{ site.baseurl }}{% post_url /it/blog/2017-03-16-spiderbot-cloud-con-controllo-vocale %}).

```python

import dotbot_ros
from dotbot_msgs.msg import Led
from dotbot_msgs.msg import Speed
from std_msgs.msg import String
import sys

class Node(dotbot_ros.DotbotNode):
    def setup(self):
        self.led_pub = dotbot_ros.Publisher('led', Led)
        self.speed_pub = dotbot_ros.Publisher('speed', Speed)
        dotbot_ros.Subscriber('speech', String, self.on_speech)
        print 'setup'

    #@dotbot_ros.on_topic('speech', String)
    def on_speech(self, speech_msg):
        led_msg = Led()
        speed_msg = Speed()
        print speech_msg.data
        sys.stdout.flush()
        if speech_msg.data == 'avanti':
            led_msg.led1 = True
            speed_msg.sx = 100
            speed_msg.dx = 100
            self.led_pub.publish(led_msg)
            self.speed_pub.publish(speed_msg)
        elif speech_msg.data == 'sinistra':
            speed_msg.sx = -100
            speed_msg.dx = 100
            self.speed_pub.publish(speed_msg)
        elif speech_msg.data == 'destra':
            speed_msg.sx = 100
            speed_msg.dx = -100
            self.speed_pub.publish(speed_msg)
        elif speech_msg.data == 'indietro':
            speed_msg.sx = -100
            speed_msg.dx = -100
            self.speed_pub.publish(speed_msg)
        elif speech_msg.data == 'fermo':
            led_msg.led1 = False
            speed_msg.sx = 0
            speed_msg.dx = 0
            self.led_pub.publish(led_msg)
            self.speed_pub.publish(speed_msg)
```

Per informazioni **info@hotblackrobotics.com**. Per la licenza da beta tester gratis registratevi [qui](http://cloud.hotblackrobotics.com/register).
