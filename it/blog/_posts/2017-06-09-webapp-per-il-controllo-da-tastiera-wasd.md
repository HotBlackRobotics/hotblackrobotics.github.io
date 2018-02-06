---
title: "Webapp per il controllo da tastiera wasd"
layout: post
date: 2017-06-09 09:57:40
image: https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRdu75D9NQxLgTcN9Iv2UkHenAJrbK2aQjovA37vbijPucqILYhBDVr5SC2
headerImage: false
lang: it
tag:
 - Robot
 - Controllo
 - Remoto

redirect_from: 
 - /2017/06/09/webapp-per-il-controllo-da-tastiera-wasd/
 - /blog/posts/2017-06-09-webapp-per-il-controllo-da-tastiera-wasd
author: Pietro Chirio
description: "In questo tutorial vedremo come controllare un robot attraverso i tasti **wasd** per farlo muovere nelle 4 direzioni."
---

In questo tutorial vedremo come controllare un robot attraverso i tasti **wasd** per farlo muovere nelle 4 direzioni. In particolare vedremo:

- come sviluppare una webapp in grado di controllare il robot attraverso la tastiera

- come sviluppare lo **sketch** in ROS per sottoscriversi al topic della webapp

La webapp
-----
Come prima cosa andiamo a sviluppare l'applicazione web che ci permetterà di controllare il robot tramite la tastiera. [Da qui](https://github.com/sgabello1/WebApp) scaricate (pemendo sul pulsante **clone or download**) la cartella ed estraetela in un luogo facilmente raggiungibile.

![](https://user-images.githubusercontent.com/29255795/26967384-5320d1c2-4cfe-11e7-8e7f-1ca7de8dcf1b.png)

Aprite la cartella **keyboardteleop_files** e aprite il file all'interno della cartella (la "struttura" in javascript della nostra webapp) con l'applicazione blocco note. A questo punto cercate le righe dove sono indicate le direzioni (nella forma `//command` e aggiungete sotto ognuno di essi la stringa `console.log("direzione")`, sostituendo la parola "direzione" in base a dove si dirigerà il robot.

![](https://user-images.githubusercontent.com/29255795/26967302-ff0a2fde-4cfd-11e7-8a97-693cca91596a.png)

Queste aggiunte serviranno a verificare l'effettiva funzionalità della nostra webapp. Il lavoro non è però ancora completo: è necessario far sì che la nostra app sia in grado di dialogare con il nostro robot. Aprite il file html **keyboardteleop** con il blocco note e cercate la stringa `start_ros('192.168.0.112', 'silverbot', '192.168.0.112', '192.168.0.112/bridge/');`

![](https://user-images.githubusercontent.com/29255795/26968358-0a5e8afc-4d02-11e7-983e-038aeaf409b3.png)

Dovete ora sostituire questa stringa con quella che identifica il vostro robot. Per trovarla collegate il vostro robot al sito di Hotblack, aprite una qualunque webapp e aprite la sorgente della pagina (tasto destro)

![](https://user-images.githubusercontent.com/29255795/26968706-438104f8-4d03-11e7-97f7-96e6deb0a765.png)

All'interno del codice sorgente cercate la stringa `start_ros('#...');`, copiatela al posto di quella presente nel file html (aperto con il blocco note) **keyboardteleop** e salvate il tutto.

Possiamo ora testare la nostra app: aprite il file **keyboardteleop** nel browser e aprite la console cliccando col tasto destro del mouse e selezionando seleziona. Refreshate la pagina (F5) e ogni volta che premete uno dei pulsanti **wasd** la console dovrebbe restituirvi, stampandola a video, l'azione connessa al pulsante premuto.

![](https://user-images.githubusercontent.com/29255795/26967298-fc17bf12-4cfd-11e7-824d-a1f437043c57.png)

Sketch ROS
---------

Scriviamo ora lo sketch in Ros che farà comunicare il nostro robot con la webapp.  Importiamo l'oggetto `Robot` dalla libreria `gpiozero` per gestirne il movimento e costruiamo le coppie di PIN GPIO a cui sono collegate i due motori, sfruttando i parametri left e right:

```python
self.robot = Robot(left=(16, 19), right=(20, 26))
```

##Sottoscriviamoci ad un Topic ROS e usiamo le Callback

 Andiamo ad implementare una funzione di callback, chiamata `keyb_wasd`. Questa funzione (come tutte le funzioni di callback) avrà la seguente forma:

```python
def keyb_wasd(self, msg):
    pass
```

I due parametri che la funzione sfrutta sono `self` (che rappresenta il nodo) e `msg`, che conterrà il messaggio scambiato dal topic.

Il messaggio `geometry_msgs/Twist` contiene 2 valori a noi utili:

 - `linear.x` che varia nell'intervallo `+20` e `-20`
 -  `linear.z` che varia nell'intervallo `+1` e `-1`

La classe `Robot` funziona in modo simile, ma i valori di velocità delle ruote possono variare tra `-1.0` (massima velocit‡ all'indietro) e `1.0` (massima velocità in avanti). La prima cosa che dovrà fare la funzione, quindi, è convertire questi valori e controllare che i valori finali siano nell'intervallo `[-1, 1]`.

```python
def on_speed(self, msg):
    v_dx = (msg.linear.x/20) - msg.linear.z
    v_sx = (msg.linear.x/20) + msg.linear.z

```

Una volta generati i due comandi di velocità (`v_dx` e `v_sx`), aggiungiamo una stringa per stamparne a video i valori finali utilizzando la funzione `print`:

```python
def on_speed(self, msg):
       #...
       print 'v_dx', v_dx
       print 'v_sx', v_sx
       stdout.flush()
       #...
```

Ricordate di aggiungere sempre la linea di codice `stdout.flush()` (ed importare il modulo `flush` con la stringa `from sys import flush` per forzare la stampa effettiva sulla shell.
Ora non ci resta che settare questi valori per far muovere le ruote. Per farlo, utilizziamo il prametro `Robot.value`dell'oggetto `Robot`

```python
self.robot.value = (v_sx, v_dx)
```


La funzione `keyb_wasd`, quindi, verrà completata in questo modo:

```python
def keyb_wasd(self, msg):
   v_dx = (msg.linear.x/20) - msg.linear.z
   v_sx = (msg.linear.x/20) + msg.linear.z

    #stampo a video i valori di v_dx e v_sx
    print 'v_dx', v_dx
    print 'v_sx', v_sx
    stdout.flush()


    #controllo del robot  
    self.robot.value = (v_dx, v_sx)
```


###Sottoscrizione al topic

Una volta implementata la funzione di callback, non ci resta che sottoscriverci al topic `keyboard` per poterla correttamente utilizzare. Per farlo, nella funzione `setup`, aggiungiamo la seguente linea di codice:

```python
 dotbot_ros.Subscriber('keyboard', Twist, self.keyb_wasd)
```

ricordandoci di importare l'oggetto `Twist` da `geometry_msgs.msg`
```python
from geometry_msgs.msg import Twist
```

###Codice completo

Ecco il codice completo del nostro programma

```python
import dotbot_ros
from sys import stdout
from geometry_msgs.msg import Twist
from gpiozero import Robot


class Node(dotbot_ros.DotbotNode):
    node_name = 'keyboard'

    def setup(self):  
        self.robot = Robot(left=(16, 19), right=(20, 26))
        dotbot_ros.Subscriber('keyboard', Twist, self.keyb_wasd)

    def keyb_wasd(self, msg):
        print msg.linear.x
        stdout.flush()

        v_dx = (msg.linear.x/20) - msg.linear.z
        v_sx = (msg.linear.x/20) + msg.linear.z

        print v_dx
        print v_sx
        stdout.flush()


        self.robot.value = (v_dx, v_sx)
        stdout.flush()
```

Per inviare i comandi dobbiamo aprire la nostra webapp, la console ROS e lanciare il nostro nodo. A questo punto refreshando la webapp dovrebbe comparire sulla console il topic  `nomerobot/keyboard`. Premendo sul pulsante `echo` ci comparirà una schermata che mostrerà il variare dei valori del messaggio pubblicato come topic dalla webapp.

![](https://user-images.githubusercontent.com/29255795/26968915-1d9efcee-4d04-11e7-9b28-75660dccf4f5.png)

Refreshiamo un'ultima volta la pagina della webapp e siamo pronti a comandare il nostro robot dalla tastiera!
