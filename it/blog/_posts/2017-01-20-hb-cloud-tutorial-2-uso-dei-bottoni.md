---
title: "HB Cloud Tutorial #2 - Uso dei Bottoni"
layout: post
date: 2017-01-20 18:26:22
image: http://res.cloudinary.com/hbr/image/upload/v1484936611/IMG_20170120_172631_aupgbk.jpg
headerImage: false
lang: it
tag:
 - Tutorial
 - Hbr
 - Hbrain
 - Gpiozero
 - Python
 - ROS

redirect_from: /blog/posts/2017-01-20-hb-cloud-tutorial-2-uso-dei-bottoni
author: ludusrusso
description: "Rieccomi con il secondo tutorial legato all'uso dei bottoni per il robot **DotBot-ROS**. In questo tutorial, vedremo come configurare ed utilizzare in Python un bottone attaccato ad un pin GPIO del Raspberry Pi 3."
---

Rieccomi con il secondo tutorial legato all'uso dei bottoni per il robot **DotBot-ROS**. In questo tutorial, vedremo come configurare ed utilizzare in Python un bottone attaccato ad un pin GPIO del Raspberry Pi 3.

![DotBot bottone tutorial ROS Raspberry](http://res.cloudinary.com/hbr/image/upload/v1484936611/IMG_20170120_172631_aupgbk.jpg)

Come al solito, scrivo questo tutorial come materiale di supporto per il corso che stiamo facendo presso l'ITIS Avogadro di Torino.

##Scopo del Tutorial
Alla fine di questo tutorial, sapremo configurare ed utilizzare un bottone utilizzando le librerie **gpiozero** e **dotbot_ros**. In particolare, affronteremo i seguenti argomenti:

- Uso della libreria **gpiozero** per interfacciarsi con un bottone e leggerne lo stato
- Come stampare a video sulla shell di HBR Cloud
- Uso della libreria **dotbot_ros** per pubblicare su un topic
- Uso delle Callback

##Il circuito elettronico

Come al solito, prima di iniziare a sviluppare il codice è importante configurare il circuito. Per il momento, utilizzeremo un semplice circuito, molto semplice, basato su un Led e un Bottone. Il led collegato al pin **GPIO05**, mentre il bottone, al pin **GPIO02**. Trovate un'immagine che mostra la numerazione dei pin in questa figura:

![Raspberry Pi Configurazione PIN](https://raw.githubusercontent.com/sgabello1/Dotbot-Kit-e-Tutorial/master/RP2_Pinout.png)

> Importante: Ricordate che la libreria **gpiozero** utilizza la numerazione dei pin colorata in Arancione.

I componenti necessari sono i seguenti:

- Led Colorato
- Resistenza da 270Ohm
- Pulsante
- Cavetti per Breadboard

Colleghiamo il led e la resistenza in serie, attaccando l'anodo del Led al pin **GPIO05** e il catodo (attraverso la resistenza) a GND.

Colleghiamo una delle due fasi dell'interruttore al Pin **GPIO02** e l'altra fase a massa.

![Circuito elettronico](http://res.cloudinary.com/hbr/image/upload/v1484936413/tutorial2_bb_rsp6yo.png)


##Scriviamo il codice!

Come al solito, partiamo dallo scheletro di un'applicazione, in cui impostiamo il nome del nodo come `button_example`

```python
import dotbot_ros

class Node(dotbot_ros.DotbotNode):
    node_name = 'button_example'

    def setup(self):
        pass

    def loop(self):
        pass
```

Importiamo i due oggetti che andremo ad utilizzare dalla libreria **gpiozero**: `Button` e `LED`, utilizzando la seguente stringa prima della dichiarazione del nodo

```python
from gpiozero import Button, LED
```

###Utilizzo del bottone

Per il momento, utilizziamo solamente il bottone per visualizzare il suo stato (se chiuso o aperto) sfruttando la shell di **DotBot-ROS** e la funzione `print` di Python.

Per prima cosa, dobbiamo inilizzare il bottone e settare la frequenza di iterazione della funzione `loop` (per ora settiamola a 10Hz). Andiamo quindi ad implementare la funzione `setup` come segue

```python
    def setup(self):
        self.btn = Button(2) #GPIO 2
        self.loop_rate = dotbot_ros.Rate(10)
```

Con la riga `self.btn = Button(3)` abbiamo creato un attributo chiamato `btn` al nostro nodo che gestisce un bottone collegato al pin `GPIO02` del raspberry.

Invece, come già spiegato nel tutorial precedente, la riga `self.loop_rate = dotbot_ros.Rate(10)` setta a 10Hz la frequenza di iterazione di `loop`.

A questo punto, possiamo implementare la funzione `loop` in modo che stampi a video lo stato del bottone. Per farlo, utilizziamo l'attributo `is_pressed` dell'oggetto `Button`, che restituisce `True` se il bottone è premuto, `False` altrimenti.

Possiamo quindi utilizzare il costrutto `if`-`else` nella funzione `loop`:

```python
    def loop(self):
        if self.btn.is_pressed:
            print 'interruttore chiuso'
        else:
            print 'interruttore aperto'
        sys.stdout.flush()
```

> Importante: Ogni qual volta viene utilizzata la funzione `print`, bisogna aggiungere la linea di codice `sys.stdout.flush()` (e importare il modulo `sys` con la stringa `import sys`. Questo serve a forzare la stampa effettiva sulla shell di **DotBot-ROS**.


Proviamo ad eseguire il codice e vedere cosa succede. Dovreste vedere un output sulla shell di questo tipo:


![Shell Bottone Status](http://res.cloudinary.com/hbr/image/upload/v1484936416/Schermata_2017-01-20_alle_18.08.05_sshqv4.png)


###Pubblichiamo lo stato del bottone su un topic ROS

Se il programma precedente funziona, siamo pronti ad utilizzare una delle funzione principali di ROS: i **Topic**.

Un topic è un canale di comunicazione che permette ai vari nodi di una rete ROS di scambiare informazioni. Un nodo può pubblicare o iscriversi ad un topic, in modo da mandare, o ricevere informazioni. Per ora ci focalizziamo sul pubblicare i dati.

Per farlo, dobbiamo creare un oggetto `Publisher`, utilizzando il costruttore `dotbot_ros.Publisher(<Name>, <Type>)`. Dove il parametro *Name* è una stringa che indica il nome del topic, mentre il parametro `Type` è un oggetto che indica il tipo di messaggio che viene scambiato all'interno del topic.

In questo semplice esempio, chiameremo il topic `button_status` e manderemo messaggi di tipo booleano. Per utilizzare il tipo `Bool` dei messaggi, prima di tutto dobbiamo importarlo dalla libreria `std_msgs.msg`, aggiungendo questa stringa all'inizio del programma

```python
from std_msgs.msg import Bool
```

Possiamo quindi creare l'oggetto pubblisher nella funzione setup

```python
    def setup(self):
        #...
        self.pub_btn_status = dotbot_ros.Publisher('button_status', Bool)
        #...
```

A questo punto, possiamo utilizzarlo nella funzione `loop`. Modifichiamola nel modo seguente

```python
    def loop(self):
        btn_status = self.btn.is_pressed
        self.pub_btn_status.publish(btn_status)
        if btn_status == True:
            print 'interruttore chiuso'
        else:
            print 'interruttore aperto'
        sys.stdout.flush()
```

In particolare, abbiamo creato una nuova variabile chiamata `btn_status`, che contiene il valore dello stato del bottone. Con la riga `self.pub_btn_status.publish(btn_status)` diciamo all'oggetto pubblicatore di mandare un messaggio sul topic a cui si riferisce contenente il valore della variabile `btn_status`. Il resto della funzione non è stato modificato.

A questo punto possiamo nuovamente testare il programma, per vedere se il topic ROS viene effettivamente utilizzato e i messaggi vengono mandati. Una volta lanciato il programma, accediamo al tab `ROS` della piattaforma online. Nella lista di topic, dovreste vedere un topic chiamato `/<nome del robot>/button_status`. Questo è il topic che abbiamo appena creato. Si noti che la piattaforma aggiunge automaticamente il *namespace* del vostro robot ai topic che creeremo. Nel mio caso specifico, il topic si chiama `/polibot/button_status`. Premiamo quindi sul pulsante **Echo** riferito al topic in questione. Si aprirà un nuovo pannello che mostrerà in tempo reale i dati scambiati all'interno del topic.

![Echo Topic Status](http://res.cloudinary.com/hbr/image/upload/v1484936413/Schermata_2017-01-20_alle_18.27.11_meep9b.png)

Proviamo a premere il bottone e vediamo se i valori inviati cambiano di conseguenza!!


###Controllo del LED

A questo punto, siamo pronti a completare la nostra applicazione utilizzando un Led. Voglio, in particolare, far si che ogni volta che il bottone venga premuto, il led cambi stato (utilizzando la funzione `toggle` vista nel tutorial precedente).

Per farlo, utilizzeremo un concetto di programmazione chiamato `callback`. La `callback` è una funzione che non viene chiamata in modo esplicito dal programma, ma che viene chiamata al verificarsi di un evento. In particolare, quello che faremo è creare una callback che chiamerà la funzione `Led.toggle` quando si verifica l'evento di pressione del bottone.

Prima di tutto, creiamo il nostro oggetto `LED` nella funzione `setup` utilizzando la linea di codice `self.led = LED(5)`. A questo punto, siamo pronti a settare la callback: per associare una funzione all'evento *pressione del pulsante*, dobbiamo settare il nome della funzione da richiamare all’attributo `Button.when_pressed`: `self.btn.when_pressed = self.led.toggle`. Sembra semplice vero? Eppure questo basta per far funzionare il tutto.

```python
    def setup(self):
        #...        
        self.led = LED(5)
        self.btn.when_pressed = self.led.toggle
```

Si noti che noi non andiamo a chiamare esplicitamente la funzione `self.led.toggle`, ma informiamo solo il programma di chiamarla quando l'evento  `self.btn.when_pressed` si verifica.

Possiamo a questo punto lanciare il programma e testare che tutto funzioni!

###Codice completo

Qui trovate il codice completo appena realizzato

```python
import dotbot_ros
from gpiozero import LED, Button
from std_msgs.msg import Bool

import sys

class Node(dotbot_ros.DotbotNode):
    node_name = 'button_example'

    def setup(self):
        self.btn = Button(2)  #GPIO 2
        self.loop_rate = dotbot_ros.Rate(10)
        self.pub_btn_status = dotbot_ros.Publisher('button_status', Bool)

        self.led = LED(5)
        self.btn.when_pressed = self.led.toggle

    def loop(self):
        btn_status = self.btn.is_pressed
        self.pub_btn_status.publish(btn_status)
        if btn_status == True:
            print 'interruttore chiuso'
        else:
            print 'interruttore aperto'
        sys.stdout.flush()
```

##Esercizi

Provate a migliorare il programma come segue.

####1. Aggiungere un secondo pulsante
Aggiungete un nuovo pulsante su un pin GPIO a piacere e stampate a video gli stati di entrambi i pulsanti ad ogni iterazione.

####2. Controllo del led con entrambi i pulsanti
Controllate il led utilizzando i due pulsanti. In particolare, fate in modo che il led si spenga premendo il primo pulsante, e si accenda premendo il secondo pulsante. Utilizzate le funzioni `led.on` e `led.off`.

####3. Topic `btn1_and_btn2`
Create un secondo topic di tipo `Bool` chiamato `btn1_and_btn2`. Su questo topic, inviate l'informazione ottenuta dall'`AND` logico del valore dei due pulsanti.

Ricordo che in Pyton, l'`AND` logico tra due variabili `a` e `b` booleane si implementa con la seguente sintassi

```python
a_e_b = a and b
```

Fate la stessa cosa con l'`OR` logico.
