---
title: "HB Cloud Tutorial - Speech Bot: come far parlare il vostro robot"
layout: post
date: 2017-02-02 15:59:55
image: https://thumbs.dreamstime.com/z/talking-robot-cartoon-38038488.jpg
headerImage: false
lang: it
tag:
 - Speech
 - Cloud
 - Assistant
 - Robotics

redirect_from: 
 - /2017/02/02/hb-cloud-tutorial-speech-bot/
 - /blog/posts/2017-02-02-hb-cloud-tutorial-speech-bot
author: sgabello
description: Le basi per costruire un "dialogo" con il vostro robot sfruttando le funzionalità di sintesi e riconoscimento vocale.
---

Un'applicazione cloud interessante e molto divertente è far parlare il vostro robot!
Tecnicamente si chiama TTS - [text to speech](https://en.wikipedia.org/wiki/Speech_synthesis), o *sintesi vocale*. I sistemi TextToSpeech (letteralmente da testo a voce) consistono appunto nel convertire un testo e riprodurlo da un sintetizzatore vocale tramite un computer. Ovviamente le applicazioni nella robotica sono tantissime, quello che faremo è dare le basi per costruire un assistente robotico con cui potrete dialogare!

Iniziamo quindi a far "parlare" il computer tramite ROS. Apriamo la Web App "Speech Rec", che trovate su [http://www.hotblackrobotics.com/cloud/webgui/speech](http://www.hotblackrobotics.com/cloud/webgui/speech) oppure nella tendina "Apps". Questa Web App l'abbiamo già vista in precedenza quando usavamo il controllo vocale. Prima con questa Web App potevamo impartire ordini al robot tramite la nostra voce, ora invece usiamo il sintetizzatore vocale per ottenere una risposta o un feedback. In questa Web App abbiamo un modulo in javascript in grado di sintetizzare la voce umana. Premendo sul tasto "Bot" il computer ci accoglierà con un caloroso "Eccomi!". Da questo momento vedrete nella sezione "Console ROS" la creazione di un nuovo topic ```/<nome_del_vostro_robot>/to_speech```.

![speech recognition cloud web app ](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/speech%20bot/web%20app%202.png)

Ora basterà scrivere un nodo ROS che pubblica sul nodo ```/<nome_del_vostro_robot>/to_speech``` una stringa e il vostro computer parlerà! Lo schema concettuale di funzionamento da ROS  alla Web App in Javascript è come in figura.

![ROS speech recognition cloud]( https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/speech%20bot/ROSspeech4444.png )

Il nodo ROS è un semplice publisher di stringhe in tempi diversi. Una volta pubblicate tutte le stringhe terminiamo il nodo in modo da lasciare tutti i processi puliti con `rospy.signal_shutdown("spegniti")`.

```python
import dotbot_ros
from std_msgs.msg import String
import rospy
from time import sleep

class Node(dotbot_ros.DotbotNode):
    node_name = 'text_to_speech_node'

    def setup(self):
        self.pub_voice = dotbot_ros.Publisher('to_speech', String)
        sleep(0.5)
        self.pub_voice.publish("ciao")

        sleep(1)
        self.pub_voice.publish("mi chiamo chat bot")

        sleep(1)
        self.pub_voice.publish("sono un'applicazione di intelligenza artificiale")
        sleep(0.5)

        rospy.signal_shutdown("nodo terminato")
```

Ovviamente la cosa interessante è combinare poi le due funzionalità della Web App. Quindi in input interpretare i comandi vocali della tua voce, processarli e dare in output una risposta. Concettualmente si può schematizzare così.

![bot ROS speech Text to Speech](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/speech%20bot/botSpeechROS2.png )

Potete scrivere un nodo ROS, che nel mio schema ho chiamato "Bot", che quando dite una parola lui risponde di conseguenza! Prossimamente come scrivere un Bot con un po' di intelligenza artificiale!
