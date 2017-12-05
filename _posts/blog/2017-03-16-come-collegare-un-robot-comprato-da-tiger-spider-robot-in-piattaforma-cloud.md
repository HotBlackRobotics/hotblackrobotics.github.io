---
title: "Come collegare un robot comprato da Tiger (Spider Robot) in piattaforma cloud !!"
layout: post
date: 2017-03-16 22:09:28
image: 
headerImage: false
tag: 
 - Hacking
category: blog
redirect_from: /blog/posts/2017-03-16-come-collegare-un-robot-comprato-da-tiger-spider-robot-in-piattaforma-cloud
author: sgabello
description: ""
---

Qui vi spiegherò come modificare un robot comprato da Tiger e collegarlo in piattaforma! Nel mio caso ho comprato uno Spider Robot per ben 7 Euro!:)

![](https://pbs.twimg.com/media/CWmEXs7WUAABSLl.jpg)

Oltre questo avrete anche bisogno di:

* un power bank per cellulare da 5 Volt io ho usato [questo](http://www.dx.com/p/cylinder-shaped-external-6000mah-emergency-power-battery-charger-for-iphone-cell-phone-silver-206652#.WFFUEh9ifCI) ma potete usare anche un power bank da 9 volt
* un ponte ad H. Io ho usato [questo] (http://eud.dx.com/product/hg7881-two-channel-motor-driver-board-dark-blue-2-5-12v-2-pcs-844407060) ma anche in questo caso potete scegliere quello che volete. Qualcuno li costruisce anche a mano mettendo insieme 4 transistor!
* un Raspberry PI 3
* cavetti con connettori femmina-femmina
* fascette da idraulico
* un pezzo di cartone ;) 
* un cavetto USB a microUSB da cellulare Android. 

E otterrete SpiderBot in cloud!

![] (https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/SpiderBotCloud2.jpeg)

Una volta montato lo Spider Bot come dalle istruzioni di Tiger, prendete un pezzo di materiale rigido  (io ho usato un cartone) e ne ritagliate un rettangolo per farci stare il Raspberry. Poi tagliate la parte inferiore in modo da creare una linguetta centrale che andrete a far passare dentro l'appiglio di plastica del robot. Lo ripiegate dentro e lo fissate, io ho usato una spillatrice. 

Poi come si vede in figura fissate il ponte ad H (hg7881) con una fascetta sul davanti del robot.  

![] (https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/Cartone.jpeg) 

Effettuate i collegamenti. Il filo rosso (+) lo inserite nel mammut del Motor A di destra nel ponte ad H e il nero (terra) nel mammut di sinistra. Con un cacciavite chiudete i mammut per far bene contatto. A questo punto prendete dei cavetti femmina-femmina e, due di questi (il mio rosso e marrone) li usate come alimentazione del ponte ad H e altri due li mettete nei pin di controllo del Motor A.

![] (https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/ponteH.jpeg)

Scaricate l'immagine per il Raspberry da [qui] (https://www.dropbox.com/s/zop0xgrcklc0951/dotbot_v0_2.rar?dl=1) e copiate sull'SD del Raspberry. Configurate il Raspberry che si possa connettere in cloud come nei tutorial precedenti. Ora montate con delle fascette il Raspberry sul supporto verticale insieme alla batteria!

![] (https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/Rasp.jpeg)

Ora facciamo i collegamenti. Secondo questo scema dei pin del raspberry collegate i due cavetti di alimentazione del ponte H ai pin 4 (+ 5V) e 6 (ground). Poi collegate i pin di controllo del motore ai pin 21 e 22. A questo punto basterà collegare il cavo USB-microUSB al Raspberry e funzionerà!

![] (https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/RP2_Pinout%20(1).png)

Per farlo muovere collegatevi in piattaforma e usate l'app Test Hardware ( http://www.hotblackrobotics.com/cloud/webgui/hwtest ). Impostate il valore 100, 100 sui motori (anche se di fatto ne controllate solo uno) e premete set motor. Vedrete che lo spider bot inizierà a muoversi! Ovviamente mettendo valori diversi si muoverà più o meno veloce e cambiando segno al valore impostato si muoverà al contrario. Prossimo tutorial con due Spiderbot costruiremo uno Spider Bot in grado di andare avanti, indietro e pure girare! 

Per informazioni **info@hotblackrobotics.com** 
Per la licenza da beta tester gratis registratevi qui [http://www.hotblackrobotics.com/register](http://www.hotblackrobotics.com/register).