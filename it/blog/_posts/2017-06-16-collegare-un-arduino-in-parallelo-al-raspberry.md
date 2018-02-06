---
title: "Collegare un Arduino in parallelo al raspberry"
layout: post
date: 2017-06-16 09:54:19
image:
headerImage: false
lang: it
tag:
 - Arduino
 - Raspberry

redirect_from: /blog/posts/2017-06-16-collegare-un-arduino-in-parallelo-al-raspberry
author: Pietro Chirio
description: ""
---

In questo tutorial vedremo come come collegare un Arduino UNO al nostro Raspberry con ROS. In questo modo potremmo sfruttare l'alto livello del codice di programmazione di ROS in parallelo alla versatilità nel controllo di sensori/motori/periferiche di Arduino.

# Nota Bene

Per far funzionare questo tutorial, prima di tutto è necessario accedere al terminale, utilizzando [questa guida](http://www.hotblackrobotics.com/blog/posts/2017-05-23-accedere-al-terminale-linux-di-hbrain-da-browser), ed eseguire il seguente comando (basta fare copia incolla). Una volta fatto questo, riavviare il robot!

```bash
curl https://gist.githubusercontent.com/ludusrusso/a3533daae7a03c07ce55b90019f2a0ba/raw/c20b544de544f0c13577c31a3bc0322718c884d8/arduino_patch_hbrain | bash
```

Includiamo le librerie di ROS su Arduino
-------
Utilizzando il pacchetto **rosserial_arduino** si "trasforma" Arduino in un nodo ROS  a tutti gli effetti, che può pubblicare o sottoscriversi a topic ROS.  La comunicazione con ROS è resa possibile dalla libreria `ros_lib`, quindi basta implementarla nella cartella delle librerie di Arduino in questo modo:

- scaricate [da qui](https://github.com/HotBlackRobotics/ros_lib_arduino) (premendo su **clone or download**) le librerie presenti
![](https://user-images.githubusercontent.com/29255795/27024733-1708cddc-4f58-11e7-9427-c3b4e0770ae6.png)

- aprite l'Arduino IDE e andate su **sketch/include library/manage libraries**, cercate **Rosserial Arduino Library** e cliccate su **install**
![](https://user-images.githubusercontent.com/29255795/27024877-b5702b28-4f58-11e7-87cb-16065a54e8d9.png)

A questo punto Arduino è in grado di comunicare con ROS pubblicando o sottoscrivendosi ai topic. E' però necessario predisporre anche il Raspberry per poter comunicare con l'Arduino: collegate la vostra scheda al portale hotblack e digitate nella bara di ricerca di Google l'indirizzo IP assegnatovi dal sito (nel mio caso **192.168.0.112**)

![](https://user-images.githubusercontent.com/29255795/27025222-02c428f6-4f5a-11e7-8c25-2c40a3aa018f.png)

Si aprirà in questo modo una pagina chiamata **supervisor status**. Cercate la voce **ros_serial** e cliccate su **Start**

![](https://user-images.githubusercontent.com/29255795/27025423-b9efd336-4f5a-11e7-9b1e-0a82eb2bf6d8.png)

In questo modo sarà sufficiente collegare l'Arduino al Raspberry attraverso una qualunque delle sue porte USB e le due schede saranno pronte a lavorare in parallelo!

Scrittura di un semplice publisher per Arduino
----
Procediamo ora a scrivere il codice di un semplice pulisher per pubblicare una stringa testuale sulla console ROS. Apriamo quindi l'Arduino IDE e iniziamo un nuovo progetto. Andiamo ad includere immediatamente le librerie che utilizzeremo e la stringa `ros: :NodeHandle  nh;` che permetterà al nostro programma di creare publisher e subscriber:

```arduino
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
```
Ora dobbiamo definire il nostro publisher/subscriber. In questo caso lavoriamo su un publisher chiamato **chatter** che pubblicherà un messaggio di tipo **&str_msg**
```arduino
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
```
Nel **setup** inizializziamo il nostro nodo ROS e definiamo i topic a cui vogliamo sottoscriverci utilizzando la stinga `nh.subscribe(nomedeltopic)` e quelli che vogliamo pubblicare, come nel nostro caso, con la stringa `nh.advertise(nomedeltopic)`
```arduino
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}
```

Come ultimo passo nella funzione **loop** il nodo pubblica la stringa "Hello World" e si chiama la funzione `ros::spinOnce()` con la quale si gestiscono tutte le **callback**
```arduino
void loop()
  {
    str_msg.data = hello;
    chatter.publish( &str_msg );
    nh.spinOnce();
    delay(1000);
  }
```
Codice completo
---
```arduino
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```

Per avere altre informazioni e tutorial più approfonditi potete visitare la pagina dedicata a [rosserial_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials) del sito Ros.org
