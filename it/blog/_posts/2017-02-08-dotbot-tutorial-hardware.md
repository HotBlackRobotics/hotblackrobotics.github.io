---
title: "Tutorial hardware - Come costruire il robot Dotbot"
layout: post
date: 2017-02-08 13:28:57
image: https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.21.55.jpeg
headerImage: false
lang: it
tag:
 - Tutorial
 - Meccanica
category: blog
redirect_from: 
 - /2017/02/08/dotbot-tutorial-hardware/
 - /blog/posts/2017-02-08-dotbot-tutorial-hardware
author: sgabello
description: ""
---

Qui di seguito un breve tutorial su come assemblare il robot Dotbot. In questo tutorial ci dedichiamo ad assemblare il  robot. Inoltre sono linkate le parti da stampare in 3D. **Un grazie speciale a Michele Maffucci da cui ho preso gran parte (o quasi tutto) per questo tutorial!** Vedi gli altri tutorial per la configurazione e la programmazione dei dotbot.

## La lista delle parti meccaniche  ##
Le parti si possono stampare con una stampante 3D (noi abbiamo usato una ShareBot e stampiamo in PLA) e i link ai file STL sono in blu.

* 2 X [db-ball_caster.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-ball_caster.stl)

* 1 X [db-bottom.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-bottom.stl)
* 1 X [db-breadboard.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-breadboard.stl)

* 1 X [db-supports.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-supports.stl ) [ **il file è unico ma i supporti per il motori da stampare sono 4**]

* 1 X [db-top.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-top.stl)

* 8 x supporti rettangolari [db-rect.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/v04-db-dist-25-mm.stl)

* 1 X [db-supports.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-supports.stl ) [ **il file è unico ma i supporti per il motori da stampare sono 4**]

* 1 X [db-top.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/db-top.stl)

* 8 x supporti rettangolari [db-rect.stl](https://github.com/sgabello1/Dotbot-Kit-e-Tutorial/blob/master/v04-db-dist-25-mm.stl)

* 4 x bulloncini di supporto al Raspberry

## La lista dei componenti ##

*  4 viti M3 da  30 mm

*  4 viti M2 da  16 mm

*  14 viti M3 da  16 mm

* 2 viti M3 da 10 mm

*  18 bulloni M3

*  4 bulloni M2

*  2 biglie di vetro

*  2 motorini CC e  2 ruote [link per acquistarli](http://www.volumerate.com/product/3-7-2v-dual-axis-tt-gear-motor-65mm-blue-rubber-wheel-for-smart-car-844443000)

*  1 scheda driver motori [link per acquistarli]( http://www.volumerate.com/product/hg7881-two-channel-motor-driver-board-dark-blue-2-5-12v-2-pcs-844407060 )

*  1 batteria power bank [link per acquistarli](https://www.amazon.it/RAVPower-Caricabatterie-Tecnologia-Universale-Smartphone/dp/B00YA01MC6/ref=sr_1_22?ie=UTF8&qid=1479834997&sr=8-22&keywords=batteria+esterna)

*  1 breadboard da 400 fori

*  1 una batteria alcalina da 9 Volt (opzionale - per alimentare i motori in parallelo al Raspberry)

*   LED, interruttori, resistenze, fili per i collegamenti sulla breadboard

Il kit smontato e con tutti i suoi componenti sarà più o meno così:

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.21.55.jpeg)

Iniziamo a montare!

## 1 - Montaggio dei motori


![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.27.45.jpeg)


![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.27.45_2.jpeg)


![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.27.45_1.jpeg)


![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.47.18.jpeg)


Con le viti M3X30.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2010.47.18_1.jpeg)

## 2 - Montaggio delle ruote ominidirezionali realizzate con delle biglie di vetro##

Iniziamo a montare le ruote omnidirezionali.


![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05.jpeg)

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05_1.jpeg )

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05.jpeg)

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05_1.jpeg )

Poi montiamo i supporti con le viti M3X16, dalla parte della ruota omnidirezionale e M3X10 dalla parte della flangia.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05_4.jpeg)
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05_3.jpeg )
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.12.05_2.jpeg )

E infine otteremo questo.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.37.35.jpeg )

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.37.35.jpeg )


## 3 - Montaggio batterie - Inserimento driver motori - Inserimento giunti stampati in 3d per secondo livello##

Montate il driver motori inserendo i 4 cavi dei motori dentro i mammut della scheda driver. L'ordine non è importante perchè determina il verso che aggiusterete in fase di test. NB: testate invece con la batteria che i motori funzionino correttamente e non abbiano problemi.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.50.00.jpeg)

Montate la batteria con due fascette.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2011.57.56.jpeg )

E infine i supporti verticali per il piano superiore.

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2012.15.27.jpeg)




## 4 - Fissaggio breadboard ##


![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2012.17.04.jpeg )
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-11%20at%2012.23.47.jpeg )

Ora avvitiamo il Raspberry e la parte superiore! E' più comodo se sotto il Raspberry mettete dello spessore, nel mio caso ho messo dei distanziali di plastica. **ATTENZIONE state attenti a montare il Raspberry dalla parte giusta, ovvero con l'alimentazione che guarda verso l'esterno e non verso la bread board e mettete i supporti circolari (bulloni di supporto) sotto il Raspberry come in figura.**

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-16%20at%2018.06.11.png)


# 6 -Montate il circuito e fate i collegamenti ##

Allacciate con delle fascette la batteria alcalina da 9v e con questa alimentate la scheda motori. I fili di alimentazione vanno collegati nei due pin centrali della scheda GND e VCC.

Ora inserite i fili di comando dei motori. Sono due per motore, tenendo conto che i pin del driver (c'è scritto comunque sopra) sono disposti cosi:

* pin 1 e 2 - controllo motore 1
* pin 3 e 4 - ground e alimentazione
* pin 5 e 6 - controllo motore 2

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/maxresdefault.png )

Il raspberry ha i pin configurati così:

![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/RP2_Pinout.png )

Nel nostro caso abbiamo i pin 3,5 per gli input e 4,6 Power e Ground per alimentare la breadboard. Collegate ogni alimentazione dei motori (motore A e motore B) ai rispettivi mammut del ponte ad H. Poi collegate il controllo dei motori (A-1A, A-1B e B-1A, B-1B) con i GPIO 9,25 (pin 21,22 o contando 10 dal basso) e GPIO 22,23 (pin 15,16 o contando 13 dal basso)del Raspberry. LED 1 GPIO 4 (pin 7).
I pin 7,11,12 per i LED.

E otterete finalmente DotBot!
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-18%20at%2012.34.06%20(1).jpeg)
![](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/tut/dotbot_git2/WhatsApp%20Image%202017-01-18%20at%2012.34.06%20(4).jpeg)
![](https://raw.githubusercontent.com/ludusrusso/images/master/avogadro/avogadro3.jpeg)
