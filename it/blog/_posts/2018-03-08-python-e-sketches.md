---
title: "Python e sketch HBR"
layout: post
date: 2018-03-08
image: /assets/imgs/2018-03-08-py/py.png
lang: it
tag:
 - python
 - HBR platform
author: fiorellazza
description: "Sintassi base di Python e sketch HBR"
---
Questo post serve a richiamare qualche costrutto base del linguaggio Python ed evidenziarne la sintassi che lo differenzia da altri linguaggi di programmazione. Procederò poi a descrivere quello che troverete nell'editor online della Piattaforma Cloud HBR e che vi sarà utile ricordare per evitare errori.

### Indice
* TOC
{:toc}
 
# 1. Python: sintassi base
Per avere un tutorial strutturato, da seguire qualora voleste imparare il linguaggio indipendentemente dalla piattaforma, [questo](http://www.html.it/guide/guida-python/) è un valido esempio. Per sperimentare, inoltre può tornare utile "giocare" con il nuovo linguaggio su un [compilatore online](https://repl.it/repls/ThreadbareBusyArrays), così da evitare la parte di installazione nel caso non foste poi interessati ad utilizzarlo sul vostro pc.
 
## 1.1. Indentazione
In Python non utilizziamo parentesi per definire un blocco ma piuttosto il blocco è definito da opportuna indentazione: **il contenuto di ogni blocco deve rientrare di 4 spazi**.

Per esempio:

```python
def funzione1():
    #contenuto funzione
    if #condizone:
        #fai qualcosa

def funzione2():
    #contenuto funzione
    while #condizone:
        #fai qualcosa
    #altro contenuto funzione
```

## 1.2. Apice singolo e virgolette 
In Python le stringhe possono essere comprese sia tra apici singoli che tra virgolette:
```python
stringa = 'stringa'
```
è uguale a
```python
stringa = "stringa"
```

## 1.3. Istruzione **if**
L'istruzione condizionale `if` non richiede l'uso di parentesi per definire la condizione da verificare. Se la condizione viene verificata, il contenuto del blocco viene eseguito:
```python
if x < 0:
    #fai qualcosa
```
## 1.4. Ciclo **while**
Finchè la condizione viene verificata, il contenuto del blocco viene eseguito ciclicamente; quando la condizione non si verifica, il programma esce dal ciclo.
```python
while x < 0:
    #fai qualcosa
```
## 1.5. Ciclo **for**
Il `for` di Python viene utilizzato in modo un po' diverso rispetto agli altri linguaggi di programmazione: infatti si può sfruttare la seguente forma per "ciclare" sugli elementi di una lista.

```python
elements = [1,3,5,7]
# "element" assume, ad ogni ciclo, un valore all'interno della lista "elements":
for element in elements:
    #fai qualcosa con element
    #fai qualcos'altro
```
Al ciclo numero 1 quindi *element* assumerà il valore 1, al ciclo 2 assumerà il valore 3 e così via.

##  1.6. Funzioni
Come avete potuto vedere nell'esempio per l'indentazione, una funzione python viene definita usando la parola chiave `def`:
```python
def funzione(possible_argomento1, possibile_argomento2): 
    #contenuto della funzione
```
## 1.7. Istruzione **import**
Questa istruzione serve ad importare le librerie necessarie per chiamare specifiche funzioni o usare particolari oggetti. Per esempio:
```python
import sys
from gpiozero import LED
...
```

# 2. Editor Piattaforma HBR
L'editor per scrivere programmini da far girare sulla Raspberry tramite la piattaforma Cloud HBR, ideata da Ludovico e Gabriele, presenta la forma che dovrebbe risultare famigliare all'utente Arduino, ovvero la presenza delle funzioni `setup()` e `loop()`, che consente all'utente di definire un blocco di inizializzazione ed uno di esecuzione ciclica esattamente come nell'IDE Arduino. Anche in questo caso i programmi scritti nell'editor HBR vengono chiamati *sketch*.
## 2.1. Classi e **self** 
La parola chiave `class` definisce una classe e deve essere seguita dal nome che vogliamo dare alla classe stessa (con iniziale maiuscola). Il concetto di classi non sarà da capire a fondo per programmare sull'editor della piattaforma: vi basti sapere che il programma nello sketch deve essere definito come classe `Node` la quale riceve sempre l'argomento `dotbot_ros.DotbotNode` (la struttura, scheletro, di uno sketch viene in ogni caso automaticamente fornita alla creazione dello sketch stesso).
Per evitare errori, ricordatevi di *anteporre `self.` a qualsiasi variabile definita in `setup()`, in modo tale da poter essere usata/processata in `loop()`* o qualsiasi altra funzione vogliate creare.

Ecco un esempio di sketch:

```python
import dotbot_ros
from gpiozero import LED

class Node(dotbot_ros.DotbotNode):
    node_name = 'blink'

    def setup(self):
        self.led = LED(5)
        self.loop_rate = dotbot_ros.Rate(2)

    def loop(self):
        self.led.toggle()
```
Per maggiori informazioni su *self* e classi Python date un occhio [qui](http://www.html.it/pag/15622/classi-in-python/).


**Ciao!** :hibiscus: