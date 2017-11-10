---
title: "SpiderBot Cloud con controllo vocale! "
layout: post
date: 2017-03-16 22:05:00
image: 
headerImage: false
tag: 
category: blog
redirect_from: /blog/post/2017-03-16-spiderbot-cloud-con-controllo-vocale
author: Gabriele Ermacora
description: ""
---

##  SpiderBot Cloud con controllo vocale!  ##

Una volta costruito lo SpiderBot come nel tutorial [http://www.hotblackrobotics.com/forum/support/22]( http://www.hotblackrobotics.com/forum/support/22 ) vi spiego come scrivere una semplice app per il controllo vocale! Il nostro SpiderBotCloud andrà avanti o indietro a seconda di ciò che direte!
Leggetevi gli altri tutorial su questo sito per comprendere meglio le righe di codice che vi posterò qui.
Una volta entrati in piattaforma da [http://www.hotblackrobotics.com/cloud](http://www.hotblackrobotics.com/cloud/index) e connesso il robot come nella Lezione 3 [http://www.hotblackrobotics.com/forum/support/4] (http://www.hotblackrobotics.com/forum/support/4) andate in Sketches. Create in nuovo programma con "New" dando il nome del codice che preferite ( o potete anche modificarne uno già esistente) e copiate il seguente codice:

```

import dotbot_ros
from dotbot_msgs.msg import Speed
from std_msgs.msg import String
import sys

class Node(dotbot_ros.DotbotNode):
    def setup(self):
        self.speed_pub = dotbot_ros.Publisher('speed', Speed)
        dotbot_ros.Subscriber('speech', String, self.on_speech)
        print 'setup'

    #@dotbot_ros.on_topic('speech', String)
    def on_speech(self, speech_msg):
        speed_msg = Speed()
        print speech_msg.data
        sys.stdout.flush()
        if speech_msg.data == 'avanti':
            speed_msg.sx = 90
            speed_msg.dx = 0
            self.speed_pub.publish(speed_msg)
        if speech_msg.data == 'dietro':
            speed_msg.sx = -90
            self.speed_pub.publish(speed_msg)


```

Salvate e avviate il programma! Se non ci sono errori andate sul menù in alto dove c'è la voce "Apps" e aprite l'app di controllo vocale [http://www.hotblackrobotics.com/cloud/webgui/speech](http://www.hotblackrobotics.com/cloud/webgui/speech). Arriverrete su una pagina così (vi consigliamo di usare Chrome).

![] (https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/voiceRecognition.png) 

Connettete la web app con il tasto "connect" e premete il tasto centrale a forma di microfono per abilitare il controllo vocale. A questo punto dite ad alta voce " Avanti"  e il robot andrà avanti e "Dietro" e il robot andrà indietro! :) 
Potete provare ad aprire il sito della Web App anche da cellulare e farlo funzionare su mobile!

Per informazioni **info@hotblackrobotics.com** 
Per la licenza da beta tester gratis registratevi qui [http://www.hotblackrobotics.com/register](http://www.hotblackrobotics.com/register).
