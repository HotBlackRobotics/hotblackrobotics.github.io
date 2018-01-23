---
title: "Annunciamo il programma di Beta Testing di HBR Cloud API"
layout: post
date: 2017-10-07 08:13:27
image: https://raw.githubusercontent.com/ludusrusso/images/master/hbropenapi/betasting.png
headerImage: false
tag:
 - Api
 - Hbr
 - Cloud
category: blog
redirect_from: /blog/posts/2017-10-07-annunciamo-il-programma-di-beta-testing-di-hbr-cloud-api
author: ludusrusso
lang: it
description: ""
---

![Time Plot](https://raw.githubusercontent.com/ludusrusso/images/master/hbropenapi/betasting.png)

<a href="https://goo.gl/forms/p7bFtHkbPwUAAydY2?utm_source=openapi&utm_medium=form&utm_campaign=api&utm_content=dc" class="btn btn-lg btn-info"> Accedi al Beta Testing delle HBR Cloud API</a>

Tra il 2014 e il 2015, abbiamo sviluppato un progetto chiamato Robot@CED, un sistema robotico in Cluod che permetteva di monitorare automaticamente un ambiente CED (Data Center) utilizzando un robot in grado di muoversi autonomamente nell'ambiente e dotato di sensori ambientali (temperatura e umidità).

Robot@CED è stata l'idea iniziale da cui è nato HotBlack Robotics attorno alla quale abbiamo sviluppato le nostre competenze sulla Cloud Robotics e la nostra tecnologia.

Oggi annunciamo ufficialmente che un altro tassello di Robot@CED rientra ufficialmente a far parte di HRB, e sarà messo a disposizione dei nostri utenti per permettere lo sviluppo di applicazioni di Cloud Robotics: stiamo sviluppando le **HBR Cloud API**.

### HBR Cloud API

Le nostre API sono le stesse utilizzate all'interno di Robot@CED, migliorate dopo due anni di esperienza di Cloud e messe a disposizione degli utenti. Essenzialmente, esse erano alla base del sistema di raccolta dati del robot: infatti permettono di salvare in Cloud e organizzare una mole di dati ambientali in base alla loro posizione ed al loro instante di cattura. In futuro, queste **API** si evolveranno includendo sempre di più funzionalità legate al mondo della navigazione autonoma, come il **calcolo automatico della traiettoria**, il **mapping** ecc.

### Accedi al programma di Beta Testing

Sei interessato ad aiutarci a sviluppare e migliorare queste API? Accedi al programma di betatesting cliccando qui

<a href="https://goo.gl/forms/p7bFtHkbPwUAAydY2?utm_source=openapi&utm_medium=form&utm_campaign=api&utm_content=dc" class="btn btn-lg btn-info"> Accedi al Beta Testing delle HBR Cloud API</a>

Non appena saremo pronti, ti forniremo un accesso da Beta Tester, un tutorial di utilizzo e tutte le informazioni necessarie per utilizzarle al meglio!

## Esempio: Robot@CED

Vediamo come noi le abbiamo utilizzate all'interno del Data Center del Politecnico di Torino per effettuare monitoraggio ambientale.

Come detto, il robot da noi sviluppato è in grado di muoversi autonomamente nell'ambiente, e di raggiungere qualsiasi punto del data center in modo sicuro (cioè evitando eventuali ostacoli).

Il data center del Politecnico di Torino è un piccolo data center composto da 16 rec (armadi di server) disposti su due file, che creano 3 corridoi. Abbiamo programmato il robot in modo da raggiungere 9 Way Point (WP) disposti come in figura sottostante (i cerchi).

![Data Center Layout](https://raw.githubusercontent.com/ludusrusso/images/master/hbropenapi/dclayout.png)

Al raggiungimento di ogni Way Point, il robot esegue le misure di temperatura e umidità e le salva nella piattaforma di Cloud Robotics sfruttando le HBR Cloud API.

Il risultato è una serie di misure geolocalizzate, o una serie storica di misure per ogni way point. In figura, i plot di temperature e umidità su 3 giorni dei dati raccolti per due waypoint specifici.

![Time Plot](https://raw.githubusercontent.com/ludusrusso/images/master/hbropenapi/plot.png)
