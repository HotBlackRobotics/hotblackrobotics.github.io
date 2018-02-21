---
title: "HB Cloud Tutorial #3 - I Motori"
layout: post
date: 2017-01-20 20:27:06
image: http://res.cloudinary.com/hbr/image/upload/v1484943808/IMG_20170120_172801_ufb9qy.jpg
headerImage: false
lang: it
tag:
 - Tutorial
 - Python
 - Hbrain
 - Gpiozero
 - Hbr
category: blog
redirect_from: 
 - /2017/01/20/hb-cloud-tutorial-3-i-motori/
 - /blog/posts/2017-01-20-hb-cloud-tutorial-3-i-motori
author: ludusrusso
description: "I Motori sono una delle parti essenziali dei robot. In questo tutorial, vederemo come è possibile in modo semplice ed intuitivo implementare un programma in Python che controlla i motori in base a comandi inviati via Wifi al Robot."
---

I Motori sono una delle parti essenziali dei robot. In questo tutorial, vedremo come è possibile in modo semplice ed intuitivo implementare un programma in Python che controlla i motori in base a comandi inviati via Wifi al Robot.

![](http://res.cloudinary.com/hbr/image/upload/v1484943808/IMG_20170120_172801_ufb9qy.jpg)

Ecco nel dettaglio cosa vedremo:

- Come gestire una coppia di motori in **gpiozero**.
- Come sottoscriversi ad un topic ROS sfruttando le **callback**.

## Circuito

Colleghiamo i motori al driver. L'alimentazione del driver va collegata alla batteria da 9V, ricordandoci di mettere in comune la massa della batteria con quella del raspberry.

Le due fasi dei motori, vanno collegate, rispettivamente, ai GPIO 16,19 (sinistra) e 20,26 (destra).

> **Importante: Ricordate di mettere in comune la massa (GND) dell'alimentazione dei motori con la massa del Raspberry Pi.**

![schema motori raspberry](http://res.cloudinary.com/hbr/image/upload/v1485196535/motori_bb_fdxui2.png)

## Scriviamo il Codice

Per utilizzare i motori, useremo l'oggetto `Robot` della libreria **gpiozero**, che è in grado di gestire il movimento di un semplice robot a due ruote.

## Programma di test
Per prima cosa, testiamo che i motori funzionino lanciando un semplice programma di test. Questo programma sfrutta solo la funzione Setup
Scriviamo un brevissimo programma che controlla il robot facendogli fare semplici movimenti.

Per creare l'oggetto `Robot` dobbiamo passare al costruttore le coppie di PIN GPIO a cui sono collegati i due motori, sfruttando i parametri `left` e `right`:

```python
self.robot = Robot(left=(16, 19), right=(20, 26))
```

Una volta inizializzato, possiamo sfruttare le funzioni `Robot.left`, `Robot.right`, `Robot.forward` e `Robot.backward` per farlo muovere, rispettivamente, a destra, sinistra, avanti e indietro. E la funzione `Robot.stop`  per farlo fermare. Utilizziamo inoltre la funzione `time.sleep` della libreria `time` per far prolungare una certa azione nel tempo al robot.

Il tutto, può essere utilizzato come nel seguente programma per testare che i motori si muovino correttamente. Si noti che `time.sleep` chiede come unico parametro il tempo di attesa in secondi.

```python
import dotbot_ros
from gpiozero import Robot

import time

class Node(dotbot_ros.DotbotNode):
    node_name = 'test_motor'

    def setup(self):
        self.robot = Robot(left=(16, 19), right=(20, 26))

        self.robot.forward()
        time.sleep(1)

        self.robot.backward()
        time.sleep(1)

        self.robot.left()
        time.sleep(1)

        self.robot.right()
        time.sleep(1)

        self.robot.stop()
```

Una volta lanciato questo programma, il robot dovrebbe iniziare a muoversi prima avanti e poi indietro, per poi girare a destra e a sinistra.
Se qualcosa non funziona, controllate che i motori siano alimentati e che le masse siano messe in comune.

## Sottoscriviamo ad un Topic ROS e usiamo le Callback

Controllare il robot in questo modo non gli permette di essere nè più nè meno di un semplice giocattolo. Proviamo quindi a fare qualcosa di più interessante: controllare il robot attraverso un topic.

Per farlo, sottoscriviamoci ad un topic chiamato `speed` di tipo `std_msgs/Int16MultiArray`. Per farlo, per prima cosa, dobbiamo capire cos’è e come si usa una funzione di callback.

### Funzioni di Callback
Come già spiegato in precedenza, una funzione di **callback** è una funzione che non viene esplicitamente chiamata dal nostro programma, ma è automaticamente eseguita al verificarsi di un certo evento *asincrono* (cioè un evento che è generato al di fuori del nostro programma).

ROS sfrutta la callback come meccanismo per intercettare i messaggi inviati da su topic al quale il nodo è sottoscritto, e per processare i dati in modo immediato e istantaneo. In particolare, ROS chiede al programmatore di implementare una funzione di callback per ogni topic a cui il nodo è sottoscritto, e chiama automaticamente questa funzione ogni qualvolta un messaggio è invitato sul topic.

### Implementiamo la funzione di callback per il topic `speed`
Ricapitolando: il nostro nodo si deve sottoscrivere al topic Speed, in cui vengono mandati comandi di velocità per il robot. Ogni volta che un messaggio viene inviato sul topic, il nodo deve processare il messaggio e controllare i motori di conseguenza.

Andiamo quindi ad implementare una funzione di callback, chiamata `on_speed`. Questa funzione (come tutte le funzioni di callback) avrà la seguente forma:

```python
def on_speed(self, msg):
    pass
```

I due parametri che la funzione prende sono `self` (che rappresenta il nodo) e `msg`, che conterrà il messaggio scambiato dal topic.

Il messaggio `std_msgs/Int16MultiArray` contiene due valori (`data[0]` e `data[1]` rispettivamente per il motore destro e sinistro) che possono variare tra `-255` e `255`. La convenzione è che `255` è il massimo valore di velocità in avanti, `-255` è il massimo all'indietro, `0` significa velocità nulla e ogni altro valore è un valore intermedio tra queste velocità.

Anche la classe `Robot` funziona in modo simile, ma i valori di velocità delle ruote possono variare tra `-1.0` (massima velocità all'indietro) e `1.0` (massima velocità in avanti). La prima cosa che dovrò fare la funzione, quindi, è convertire questi valori e controllare che i valori finali siano nell'intervallo `[-1, 1]`.

```python
def on_speed(self, msg):
    v_dx = msg.data[0]/255.0
    v_sx = msg.data[1]/255.0

    #controllo che v_dx sia nel range [-1,1]
    if v_dx > 1.0:
        v_dx = 1.0
    elif v_dx < -1.0:
        v_dx = -1.0

    #controllo che v_sx sia nel range [-1,1]   
    if v_sx > 1.0:
        v_sx = 1.0
    elif v_sx < -1.0:
        v_sx = -1.0
```

Una volta generati i due comandi di velocità (`v_dx` e `v_sx`), aggiungiamo una stringa per stamparne a video i valori finali utilizzando la funzione `print`:

```python
def on_speed(self, msg):
       #...
       print 'v_dx', v_dx
       print 'v_sx', v_sx
       sys.stdout.flush()
       #...
```

Ricordate di aggiungere sempre la linea di codice `sys.stdout.flush()` (e importare il modulo `sys` con la stringa `import sys` per forzare la stampa effettiva sulla shell di DotBot-ROS.
Ora non ci resta che settare questi valori per far muovere le ruote. Per farlo, ci viene incontro un utilissimo parametro dell'oggetto `Robot`: `Robot.value`, che si usa nel seguente modo:

```python
self.robot.value = (v_sx, v_dx)
```

In particolare, questo parametro vuole entrambi i comandi di velocità contemporaneamente tra parentesi tonde (in python questa struttura si chiama `Tupla`) e, appena settato, automaticamente controlla le ruote con i valori richiesti!

La funzione `on_speed`, quindi, verrà completata in questo modo:

```python
def on_speed(self, msg):
    v_dx = msg.data[0]/255.0
    v_sx = msg.data[1]/255.0

    #stampo a video i valori di v_dx e v_sx
    print 'v_dx', v_dx
    print 'v_sx', v_sx
    sys.stdout.flush()

    #controllo che v_dx sia nel range [-1,1]
    if v_dx > 1.0:
        v_dx = 1.0
    elif v_dx < -1.0:
        v_dx = -1.0

    #controllo che v_sx sia nel range [-1,1]   
    if v_sx > 1.0:
        v_sx = 1.0
    elif v_sx < -1.0:
        v_sx = -1.0

    #controllo del robot  
    self.robot.value = (v_dx, v_sx)
```

### Sottoscrizione al topic

Una volta implementata la funzione di callback, non ci resta che sottoscriverci al topic `speed` per poterla correttamente utilizzare. Per farlo, nella funzione `setup`, aggiungiamo la seguente linea di codice:

```python
dotbot_ros.Subscriber("speed", Int16MultiArray, self.on_speed)
```

ricordandoci di importare l'oggetto `Int16MultiArray` da `std_msgs.msg`
```python
from std_msgs.msg import Int16MultiArray
```

### Codice completo

Ecco il codice completo del nostro programma

```python
import dotbot_ros
import sys
from gpiozero import Robot
from std_msgs.msg import Int16MultiArray

class Node(dotbot_ros.DotbotNode):
    node_name = 'example_motor'

    def setup(self):
        self.robot = Robot(left=(16, 19), right=(20, 26))
        dotbot_ros.Subscriber("speed", Int16MultiArray, self.on_speed)

    def on_speed(self, msg):
        v_dx = msg.data[0]/255.0
        v_sx = msg.data[1]/255.0

        #controllo che v_dx sia nel range [-1,1]
        if v_dx > 1.0:
            v_dx = 1.0
        elif v_dx < -1.0:
            v_dx = -1.0

        #controllo che v_sx sia nel range [-1,1]   
        if v_sx > 1.0:
            v_sx = 1.0
        elif v_sx < -1.0:
            v_sx = -1.0

        #controllo del robot  
        self.robot.value = (v_dx, v_sx)
```

Per inviare comandi di velocità, possiamo utilizzare l'app di test della nostra piattaforma.

## Esercizi

Vi suggerisco alcuni esercizi per migliorare il codice.

### Esercizio 1: Robot Joystick

La webapp joystic manda messaggi di tipo `geometry_msgs/Vector3` sul topic `joy` contenenti due variabili `msg.x` e `msg.y` che contengolo la posizione del joystick in coordinate cartesiane. Si modifichi il programma in modo da intercettare questo messaggio e far muovere il robot in base alla posizione del joystick.

### Esercizio 2: Stop del robot dopo N secondi

Un problema che (in base alle preferenze) potrebbe essere risolto o no, rigurda il fatto che se viene impostato un comando di velocità, il robot continuerà a muoversi finchè non gli viene impostato il comando di velocità `(0,0)`.

Si provi a migliorare il programma in modo che il robot si fermi dopo *N* secondi (sceliti da programmatore) dall'ultimo comando di velocità ricevuto.

Per farlo suggerisco di utilizzare i seguenti accorgimenti:

1. Creare una variabile che contiene il tempo di esecuzione dell'ultima chiamata della funzione di callback. Usando la funzione `datetime.datetime.now`.
2. Settare l'esecuzione della funzione `loop` a frequenza alta (ad esempio, 20Hz).
3. Nella funzione loop, controllare che la differenza in secondi tra l'ultima volta che è stata eseguita la funzione `on_speed` e il tempo attuale sia maggiore di N
4. In caso affermativo, utilizzare la funzione `Robot.stop` per stoppare il robot.

Vi riporto, sotto, un esempio per ottenere la differenza in secondi tra due tempi

```python
from datetime import datetime
import time

x = datetime.now()
time.sleep(2)
y = datetime.now()
seconds = (y-x).total_seconds()
# 2.003452
```
