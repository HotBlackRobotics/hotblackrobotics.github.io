---
title: "Come creare messaggi custom in ROS"
layout: post
date: 2018-02-26
image: /assets/imgs/2018-02-26-ros-custom-messages/ros_custom.png
lang: it
tag:
 - ROS

author: fiorellazza
description: "Creare nuovi tipi di messaggio ROS"
---
Ciao a tutti! Oggi metterò insieme una serie di informazioni che ho raccolto mentre cercavo di creare un nuovo tipo di messaggio con ROS durante il mio progetto di tesi.
Questa potrebbe essere la vostra situazione nel caso aveste bisogno di un tipo di messaggio base con l'intento di semplificare la vostra applicazione: infatti alcuni [messaggi standard di ROS](http://wiki.ros.org/std_msgs/) sono fin troppo complessi per l'uso semplice che si vuol ottenere.
Spero che questo post possa riassumere e accelerare la creazione di messaggi custom. Iniziamo!

### Index
* TOC
{:toc}

# Creare il file .msg 
Assumo che abbiate già il vostro workspace catkin (creato seguendo [questo tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)) e che abbiate creato il vostro pacchetto ROS come spiegato [qui](http://wiki.ros.org/it/ROS/Tutorials/CreatingPackage#Creare_un_catkin_Package).

>**RMK**: è buona abitudine creare un pacchetto specifico per la definizione di messaggi, e.g., create un package chiamato `custom_msgs`.

Prima di tutto, dalla command line, entrate nella cartella del package, sfruttando il comando ROS `roscd`:

```bash
roscd custom_msgs
```

Una volta nella cartella, create una nuova cartella chiamata `msg`, tale che i messaggi custom contenuti in essa vengano riconosciuti auotmaticamente durante la compilazione del pacchetto:

```bash
mkdir msg
cd msg
```

Create il file di definizione del nuovo messaggio specificando direttamente il suo contenuto e salvandolo in un file con estensione `.msg`; nel mio caso, avevo bisogno di un semplice array di interi che ho chiamato `Servo_Array`.

```bash
echo "uint16[] data" > msg/Servo_Array.msg
```

Per controllare se la definizione del messaggio è stata salvata correttamente, potete semplicemente fare un check del contenuto:

```bash
cat Servo_Array.msg
```

# "Attivare" la generazione del messaggio
Per sollecitare la generazione dei nuovi tipo di messaggio durante la compilazione con catkin, dobbiamo modificare i file contenuti nel package `package.xml` e `CmakeLists.txt`:

- Aprite `package.xml`, e assicuratevi che queste due linee siano presenti e **de-commentatele**:

```xml
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

- Aprite `CmakeLists.txt`, aggiungete `message_generation` alla lista dei `COMPONENTS`, così:

```txt
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

- Esportare la dipendenza message runtime:

```txt
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

- Quindi de-commentate le seguenti righe (rimuovete `#`) e rimpiazzate `Message*.msg` con il vostro file .msg (nel mio caso `Servo_Array.msg`):

```txt
add_message_files(
  FILES
  Servo_Array.msg
)
```
- Infine de-commentate queste righe:

```txt
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

>**NB**: se avete più di un messaggio custom da aggiungere, create i relativi file .msg e aggiungeteli ogni volta fche un file .msg deve essere aggiunto nel file `CmakeLists.txt` (come specificato sopra).

# Re-building del package
Adesso che abbiamo creato dei nuovi messaggi, dobbiamo fare di nuovo il make del pacchetto:
#Nel vostro workspace catkin

```bash
roscd custom_msgs
cd ../..
catkin_make
```

>**NB**: supponiamo che vogliate scrivere degli script Python all'interno di un package chiamato, per esempio, `my_package`: per importare il messaggio custom nel vostro script, avrete bisogno la seguente riga `from custom_msgs.msg import Motors_Array`. Notate che gli scripts Python sono solitamente contenuti in una cartella `my_package/scripts`.

La maggior parte delle informazioni in questo post sono state prese da [qui](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Common_step_for_msg_and_srv).

# Messaggi custom e Rosserial Arduino
Nel caso abbiate bisogno di usare il vostro messaggio cusotm nel nodo seriale su Arduino, dovete solo copiare il vostro package `custom_msgs` nella cartella `ros_lib` (*cartella_di_sketch_Arduino*/libraries/ros_lib/). Dopo aver ri-lanciato l'editor Arduino, potete riferirvi al nuovo messaggio nel vostro sketch con `#include <custom_msgs/Motors_Array.h>`.

**Ciao!** :hibiscus: