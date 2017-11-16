---
title: "Collegare in cloud i LEGO NXT ed estenderne le funzionalità"
layout: post
date: 2017-06-16 13:32:57
image: 
headerImage: false
tag: 
 - Lego
category: blog
redirect_from: /blog/post/2017-06-16-collegare-in-cloud-i-lego-nxt-ed-estenderne-le-funzionalita
author: Pietro Chirio
description: ""
---

In questo tutorial vedremo come collegare un braccio a 3 gradi di libertà costruito con la piattaforma Lego NXT al cloud utilizzando il Raspberry pi3 e l'interfaccia Hbrain. Successivamente vedremo come scrivere una webapp che ci permetta di sfruttare le potenzialità del cloud per controllarlo. 

![](https://user-images.githubusercontent.com/29255795/27220202-9fdf2db6-5284-11e7-9fd3-85c078f423ce.jpg)

Collegamenti
---
Come prima cosa prendete la scheda Raspberry e accendetela collegandola alla corrente. Collegatela al sito di Hotback (come imparato dal manuale). Prendete il lego NXT, accendetelo e collegate il suo cavo usb ad un ingresso qualunque della scheda Raspberry. Avete così collegato il lego NXT al cloud!

La webapp
----
Ora che abbiamo connesso l'NXT al cloud procediamo a creare una webapp che ci permetterà di controllarlo attraverso la tastiera del nostro computer. Come prima cosa scaricate, premendo sul pulsante **clone or download** i file che potete trovare [qui](https://github.com/cynicalzero4/raspnxt). 

![](https://user-images.githubusercontent.com/29255795/27223586-5bc4c21e-5291-11e7-8767-43ec9775e773.png)

Ora è necessario far sì che la nostra webapp sia in grado di dialogare con il nostro robot. Aprite il file html **keyboard_robotarm** con il blocco note e cercate la stringa `start_ros('192.168.0.112', 'silverbot', '192.168.0.112', '192.168.0.112/bridge/');`

![](https://user-images.githubusercontent.com/29255795/26968358-0a5e8afc-4d02-11e7-983e-038aeaf409b3.png)

Dovete ora sostituire questa stringa con quella che identifica il vostro robot. Per trovarla dirigetevi sul sito di Hotblack al quale avete prima connesso la vostra scheda Raspberry, aprite una qualunque webapp e aprite la sorgente della pagina premendo il tasto destro del mouse

![](https://user-images.githubusercontent.com/29255795/26968706-438104f8-4d03-11e7-97f7-96e6deb0a765.png)

All'interno del codice sorgente cercate la stringa `start_ros('#...');`, copiatela al posto di quella presente nel file html (aperto con il blocco note) **keyboard_robotarm** e salvate il tutto.

Possiamo ora testare la nostra app: aprite il file **keyboard_robotarm** nel browser e aprite la console cliccando col tasto destro del mouse e selezionando ispeziona. Refreshate la pagina (F5) e ogni volta che premete uno dei pulsanti **wa, sd, ik** la console dovrebbe restituirvi, stampandola a video, l'azione connessa al pulsante premuto.

![](https://user-images.githubusercontent.com/29255795/27224569-1c161e74-5296-11e7-824c-125534439931.png)

Sketch ROS
---
Scriviamo ora lo sketch in ROS che farà comunicare il nostro robot con la webapp.  Importiamo subito le librerie che utilizzeremo nel programma:
```python
import dotbot_ros
import sys
import nxt
from nxt.motor import *
from geometry_msgs.msg import Twist
from time import sleep
```
Ora nel `setup` andiamo a includere la stringa `self.NXT = nxt.locator.find_one_brick()` per far sì che il programma cerchi il lego NXT attraverso la connessione usb e inizializziamolo:

```python
print 'starting'   #stampa starting al lancio del programma 
sys.stdout.flush() #forza la stampa sulla shell ROS
       
self.NXT = nxt.locator.find_one_brick()
self.m1 = Motor(self.NXT, PORT_A) #motore della porta A
self.m2 = Motor(self.NXT, PORT_B) #motore dell porta B
self.m3 = Motor(self.NXT, PORT_C) #motore della porta C
self.cnt = -1  
```
Sempre nel **setup** creiamo un **subscriber** che sottoscriva il programma al messaggio della webapp da noi creata. Il nostro subscriber si sottoscriverà ad un topic chiamato`/keyboard` che scambia messaggi di tipo `std_msgs/Twist` inserendoli in una funzione callback chiamata `xyz` definita dai parmetri `self` e `msg` :

```python
def setup(self):

     #...
     dotbot_ros.Subscriber("/keyboard", Twist, self.xyz)

def xyz(self, msg):

```

Procediamo ora a sfruttare i messaggi che giungono dalla nostra webapp nella nostra funzione `xyz` per definire i movimenti che eseguirà il nostro braccio. Il messaggio di tipo `Twist` spedisce, nel nostro caso, 3 messaggi diversi:

- **msg.linear.x** che useremo per muovere il primo giunto

- **msg.linear.y** che useremo per muovere il secondo giunto

- **msg.linear.z** che useremo per muovere il terzo giunto

Per far sì che il nostro braccio si muova sfrutteremo l'attributo **turn** della classe **Motor** definito da 5 parametri:

- **power**: la forza, nel range **-127 /+128**, con cui si attivano i motori. Valori inferiori a 64 sono altamente sconsigliati

- **tacho_units**: il numero di gradi di cui si muoverà il motore. Il valore minimo che i motori sono in grado di leggere, già con difficoltà, è **5**

- **brake**: definisce se il motore si stoppa (**True**) dopo aver compiuto il movimento o meno (**False**)

- **timeout**: numero di secondi dopo il quale viene mostrato un messaggio di errore nel caso il motore non si muova

- **emulate**: da definire sempre come False 

Per far sì che il giunto del nostro braccio si muova solamente nel caso in cui il messaggio proveniente dal topic sia diverso da 0 utilizziamo il costrutto `if-else`. in particolare il simbolo **!=** significa **diverso** e la stringa `self .m1.idle()` dice semplicemente al motore definito dalla funzione **m1** di non fare alcunchè:

```python
def xyz(self, msg):

    if msg.linear.x != 0: 
        self.m.turn(msg.linear.x*100, 5, True, 1, False)
            
    else: 
        self.m1.idle()
```
A questo punto abbiamo creato la struttura per far muovere il primo giunto quando riceve un messaggio dal topic. Non ci resta che aggiungere il codice per fa muovere gli altri due, copiando il codice già scritto modificando semplicemente il tipo di messaggio del quale verificare la differenza da 0 e il motore da muovere:

```python
def xyz(self, msg):

    if msg.linear.x != 0: 
        self.m1.turn(msg.linear.x*100, 5, True, 1, False)
            
    else: 
        self.m1.idle()


    if msg.linear.y != 0: 
        self.m2.turn(msg.linear.x*100, 5, True, 1, False)
            
    else: 
        self.m2.idle()


    if msg.linear.z != 0: 
        self.m3.turn(msg.linear.x*100, 5, True, 1, False)
            
    else: 
        self.m3.idle()
```

Codice completo
---

```python
import dotbot_ros
import sys
import nxt
import nxt.locator
from nxt.motor import *
from geometry_msgs.msg import Twist
from time import sleep

class Node(dotbot_ros.DotbotNode):
    node_name = 'nxt_node_keyboard'

    def setup(self):
        print 'starting'
        sys.stdout.flush()
        
        self.NXT = nxt.locator.find_one_brick()
        self.m1 = Motor(self.NXT, PORT_A) #motore della porta A
        self.m2 = Motor(self.NXT, PORT_B) #motore della porta B
        self.m3 = Motor(self.NXT, PORT_C) #motore della porta C
        self.cnt = -1  
        
        dotbot_ros.Subscriber("/keyboard", Twist, self.xyz)

    def xyz(self, msg):
        
        if msg.linear.x != 0: 
            self.m1.turn(msg.linear.x*100, 5, True, 1, False)
            
        else: 
            self.m1.idle()
        
        
        if msg.linear.y != 0: 
            self.m2.turn(msg.linear.y*100, 5, True, 1, False)
            
        else: 
            self.m2.idle()
            
        
        if msg.linear.z != 0:
            self.m3.turn(msg.linear.z*100, 5, True, 1, False)
            
        else:
            self.m3.idle()
        
```