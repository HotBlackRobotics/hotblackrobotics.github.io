---
title: "Introduzione e visione tecnologica, Cloud Robotics e Internet delle Cose: l'Internet dei Robot"
layout: post
date: 2017-01-12 14:26:29
image: https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/cloudcomputing-01.jpg
headerImage: false
lang: it
tag:
 - Robotics
 - Cloud
 - Vision
 - Iot
 - Trend
category: blog
redirect_from:
 - /2017/01/12/introduzione-e-visione-tecnologica-cloud-robotics-e-internet-delle-cose-l-internet-dei-robot/
 - /blog/posts/2017-01-12-introduzione-e-visione-tecnologica-cloud-robotics-e-internet-delle-cose-l-internet-dei-robot
author: sgabello
description: "Introduzione e visione tecnologica, Cloud Robotics e Internet delle Cose: l' Internet dei Robot"
---

 L'internet delle cose è quella visione tecnologica che vede "oggetti" interconnessi comunicare e processare dati tramite Internet. Fino a poco tempo fa gli unici oggetti collegati alla rete erano i computer, le stampanti e gli smartphone. Oggi iniziano a vedersi i primi oggetti "smart" collegati alla rete: ad esempio la smartTV o le console di videogame moderne.
In futuro avremo tutti gli oggetti di casa nostra "smart": ci sarà il frigorifero connesso alla rete domotica della casa insieme a tutti gli elettrodomestici. Tutti questi dispositivi in casa integrati in una sola rete ottimizzeranno le risorse per sprecare meno elettricità e gas a seconda delle nostre necessità.

Fuori dall' ambiente domestico troveremo, ad esempio, reti di sensori nella città per monitorare lo stato dei parcheggi che ti informeranno su dove è meglio parcheggiare; telecamere smart che sorveglieranno la città; sensori per il monitoraggio della temperatura e umidità nelle coltivazioni agricole.

Un esempio concreto di IoT domestico (già in vendita) è [Nest](https://nest.com/): un termostato per la casa intelligente che si collega ad Internet e tramite il cellulare studia l'utilizzatore e si adatta alle abitudini delle persone regolando così la temperatura a seconda che la persona sia in casa o meno.
Internet delle cose quindi, perchè stiamo vivendo in questo periodo tecnologico il passaggio da *Internet "digitale"*, ovvero le pagine web, i server, i computer, gli smartphone o le stampanti, a *Internet degli oggetti*, ovvero la diffusione di Internet a rendere "smart" tutti gli oggetti che ci circondano.

![Smart Home IoT](http://itersnews.com/wp-content/uploads/2014/08/Smartthings-Living-Room.png)


La **Cloud Robotics** si pone all'interno della visione dell'Internet delle cose ma estesa anche ad "oggetti" che operano nell'ambiente fisico oltre che quello digitale: i robot. Nel futuro imminente avremo robot interconnessi tra loro e agli oggetti "smart" che oltre a comunicare tramite Internet, effettueranno anche delle azioni nel mondo fisico. Immaginiamo ad esempio la self driving car di Google interconnessa a tutte le altre autonomibili autonome ottimizzare il traffico e risolvere tutte le nostre necessità legate alla mobilità.
Un esempio già molto attuale è [Roomba](http://www.irobot.it/roomba/): robot aspirapolvere connesso con gli altri dispositivi smart della casa. Altri esempi più fantascientifici sono già in mostra alle fiere di robotica, come il braccio robotico che cucina in casa apprendendo dalla rete dai migliori chef al mondo, flotte di droni per il trasporto di merci o per il monitoraggio intelligente della citta senza il bisogno di infrastrutture.

![Visione Cloud Robotics](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/cloudcomputing-01.jpg)

Questa visione tecnologica è strettamente dipendente da una piattaforma di cloud robotics, che è il layer tecnologico che permette la comunicazione tra gli oggetti e lo storage delle informazioni. In generale la cloud robotics ha tre caratteristiche importanti:

* **L'astrazione dell'hardware:** ovvero gli oggetti interconnessi alla rete (robot, sensori o dispositivi) vengono astratti e l'interfaccia di comunicazione è semplificata e uguale per tutti. Agli occhi della piattaforma tutti i dispositivi sono uguali e non ci interessa che tipo di hardware sia e il linguaggio di comunicazione proprietario, la piattaforma si pone ad un livello più alto e tralascia i protocolli a più basso livello. Ad esempio per una telecamera che comunica in cloud non ci interessa il protocollo di comunicazione, i driver che supporta, la marca, l'hardware, le API proprietarie ma tutto è astratto a livello più alto e la telecamera figurerà solo come un entità che genera uno stream video. Il nostro robottino Dotbot è un altro esempio che vedremo qui in particolare. Dotbot utilizza un Raspberry PI 3, diversi sensori e driver per i motori ma tutto è astratto dalla piattaforma e non c'è addirittura bisogno che chi utilizza Dotbot sappia che c'è un Raspberry e sappia come configurarlo!Il tutto è accessibile ed utilizzabile in modo semplice grazie alla piattaforma di cloud robotics.

* **La condivisione delle informazioni:** gli oggetti non comunicano solo tra di loro ma scambiando informazioni salvano dati sui data base in piattaforma. Questi dati processati in modo opportuno diventano informazione condivisa e si trasforma in "esperienza" per gli oggetti. Immaginate la macchina automatica di Google che deve calcolare il percorso ottimale per portarvi a casa nella vostra routine quotidiana, facendo lo stesso percorso tutti i giorni imparerà quali sono le "scorciatoie", saprà che quel determinato tratto è più trafficato, saprà fare una stima dei tempi corretta di quanto tempo ci mette e cosi via. Non solo nella routine, ma nel caso voi vogliate andare in un posto completamente nuovo e sconosciuto la macchina scaricherà dalla rete l'esperienza di altri e avrete sicuramente il percorso ottimale! Questo funziona molto bene con l'intelligenza artificiale. Ad esempio se il vostro robot deve riconoscere un oggetto a lui sconosciuto, gli basterà scattare una foto e mandarlo in rete per vedere se l'oggetto in questione è stato già riconosciuto da un altro robot e scaricare quindi tutti i dati relativi all'operazione che dovrà effettuare. In realtà la condivisione delle informazioni non è solo molto utile ai robot ma anche agli umani. Infatti tramite la piattaforma potete scaricare il codice scritto da altri e utilizzarlo o modificarlo a piacimento oltre che condividere suggerimenti e commenti come sui tipici forum.

* **Il remote processing:** Questa è la caratteristica tipica delle soluzioni cloud, non solo applicate alla robotica. Ovviamente relegando i processi alla rete possiamo disporre di processori in locale sempre meno potenti siccome i calcoli "pesanti", a livello computazionale, sono effettuati da server in remoto. Questo ci permette di avere hardware meno performanti e banalmente si può tradurre nella riduzione dei costi dei robot e dispositivi, cosa molto importante!L'esempio legato al riconoscimento oggetti, di cui scrivevo prima, ad esempio ha bisogno di hardware molto meno performanti se si sceglie di utilizzare la cloud. Senza cloud il robot ha bisogno di tera e tera di memoria per memorizzare tutti i possibili oggetti che dovrà riconoscere il software, gli algoritmi in locale, un computer in grado di sostenere tutto il carico computazionale oltre che i sensori per il riconoscimento dell'oggetto. Un altro esempio che fa vedere la potenza della robotica in cloud è sempre Dotbot il quale è dotato solamente di un Raspberry Pi, dei led e driver motori e possiede un intelligenza minima. Nell' applicazione che vedremo più avanti controlleremo il robot con la voce, e ovviamente tutta la parte di intelligenza artificiale è in remoto.
