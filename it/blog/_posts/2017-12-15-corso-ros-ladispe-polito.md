---
title: "Laboratori ROS LADISPE Politecnico di Torino"
redirect_from:
 - /2017/12/15/corso-ros-ladispe-polito/
layout: post
date: 2017-12-15
last_modified_at: 2018-02-01
image: /assets/imgs/2017-12-15-corso-ros-ladispe-polito.md/lab.png
headerImage: false
tag:
 - ROS
 - Corso
 - Robotica
 - Politecnico di Torino
author: ludusrusso
description: "Un laboratorio di ROS a cui possono partecipare gli studenti del Politecnico di Torino"
---

In questi giorni, ho scoperto (e la cosa mi ha fatto molto piacere), che dentro
il poli gli studenti iniziano a sentire la necessità di conoscere un po' il mondo
di ROS e della robotica, in modalità più pratica e non solo teorica.

Così, in po' per caso, e grazie all'aiuto del buon Luigi Spagnolo, abbiamo deciso di
far partire degli incontri non strutturati in cui diamo la possibilità agli studenti
interessati ad approfondire questo tema e smanettare con ROS, Raspberry Pi e Arduino
per costruire un proprio robot personale.

La modalità dei laboratori sarà libera e non direttamente supervisionata. Agli studenti
sarà data la possibilità di accedere ad orari prestabiliti al laboratorio LADISPE
del Politecnico di Torino e di avere a disposizione del materia per lavorare.

La mia idea, un po' ambiziosa ma fattibile, è di metterli nella condizione di progettare
e costruire un piccolo robot stampato in 3D e basato su ROS su Raspberry Pi 3 per il
controllo ad alto livello e Arduino per il controllo a basso livello di sensori e
attuatori.

In questa pagina, ed in quella del LADISPE, verranno pubblicati periodicamente
dei tutorial e guide per apprendere il funzionamento di ROS e di queste tecnologie,
per guidare passo passo i ragazzi verso la progettazione del proprio robot.

## Materiale

Il materiale sarà fornito da noi durante le ore dedicate ai laboratori.
Abbiamo a disposizione una serie di Raspberry Pi 3 Model B e schede Arduino su cui
iniziare a fare i primi esperimenti.

Stiamo pensando di attrezzare il laboratorio con una stampante 3D.


## Alcune informazioni su ROS

Prima di iniziare il tutorial, vorrei fare alcuni chiarimenti su cosa è (e cosa non è)
ROS e su quali sono le sue finalità.

Nonostante il nome (*Robot Operating System*) ROS è un **framework** per lo sviluppo
di *applicazioni* robotiche. È nato nel 2007 sulla necessità di evitare che ogni
laboratorio di ricerca sulla robotica di servizio dovesse costruirsi il proprio
set di funzioni base su cui poi costruire le proprie applicazioni. È stato quindi
rilasciato con licenza OpenSource ed è completamente aperto e modificabile.

Data la grossa necessità a cui questo rispondeva, è stato adottato in modo massiccio
in pochi anni da quasi la totalità dei centri di ricerca (pubblici e privati) sulla
Robotica nel mondo, tanto che al momento è considerato uno *standard di fatto* per
la prototipazione e lo sviluppo di applicazioni robotiche di servizio.

ROS è stato sviluppato in modo da essere fortemente estendibile, dando la possibilità
ai vari laboratori di ricerca di contribuire rilasciando il loro codice all'interno
di pacchetti ROS. In questo modo, all'interno di ROS è facile trovare tutti gli algoritmi
allo stato dell'arte per abilitare il robot a risolvere compiti complessi e standard (come la navigazione
autonoma), rendendo molto più semplice concentrarsi sull'applicazioni ad alto livello e
non sui problemi matematici.

Ovviamente ROS ha anche alcuni problemi, i principali sono, a mio avviso, i seguenti:

1. ROS nasce 10 anni fa avendo come obiettivo principalmente i laboratori di ricerca, un use case che al tempo prevedeva un robot, un server su cui fare le computazioni più pesanti e una connessione locale tra il robot e il server. Ovviamente, questo caso d'uso è superato e la ricerca è andata avanti verso tecnologie più evolute, come la *Cloud Robotics* o la *Collaborazione Multi robot*. In questi casi, ROS pone grosse limitazioni architetturali.
Molti progetti ([RoCon](http://wiki.ros.org/rocon), [Rapyuta](https://www.rapyuta-robotics.com/)) cercano di mettere una pezza a questi limiti di ROS, ma si sta anche cercando di [riprogettarlo dalle fondamenta](http://design.ros2.org/) per risolvere questi problemi.
2. Alcune decisioni dei programmatori iniziali sono contemporaneamente il punto forte e il punto debole del progetto. Ad esempio, la scelta di mantenere lo standard di scambio messaggi il più generico possibile rispetto all'hardware, da una parte fa si che gli stessi algoritmi vadano bene per robot con forme e caratteristiche diverse (un robot umanoide uno su ruote e un quadricottero, in ROS, sono controllabili nello stesso modo). Dall'altro crea un grosso overhead nella dimensione dei messaggi.
3. La curva di apprendimento di ROS è bella tosta, specialmente all'inizio (ve ne accorgerete se seguirete queste guide).

## Primo Incontro

Lo scopo del primo incontro è dare ai partecipanti la possibilità di prendere
confidenza con le basi di ROS e con Linux, sistema operativo su cui ROS è
stato costruito e mantenuto.

#### Set Up Raspberry Pi 3

  1. Scaricare l'immagine di [Ubuntu Mate 16.04](https://ubuntu-mate.org/download/).
  2. Copiare l'SD del Raspberry utilizzando i vari tool disponibili online (noi consigliamo [Etcher](https://etcher.io/))
  3. Impostare un nome utente e una password e configurare il SO
  4. Installare [Installazione ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Debian)

#### Set Up Ubuntu o Macchina virtuale

  1. Installare [Ubuntu 16.04](https://www.ubuntu-it.org/download) (si faccia attenzione alla versione) sul proprio computer o su macchina virtuale con VirtualBox
  2. configurare il SO
  3. Installare [Installazione ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Debian)

### Cosa fare

Avete a disposizione un Raspberry Pi 3 model B con sopra installato ROS Kinetic.
 - username: ros
 - password: ros

Il primo laboratorio consiste nell'iniziare a prendere confidenza con il sistema publishing
subscribing di ROS seguendo le seguenti guide:

 - [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
 - [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
 - [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
 - [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
 - [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

Alla fine dei tutorial, provare a scrivere un semplice nodo publisher che fa muovere la
turtlesim.


## Secondo Incontro: 12/02/2018, ore 14:30

**Scopo del laboratorio**: Configurare ROS con Arduino e [ROSSerial]() per controllare sensori e attuatori.

Avrete a disposizione degli Arduino UNO (o simili) da interfacciare con il vostro
Raspberry Pi, insieme a vari sensori ed attuatori forniti dal laboratorio.

Il vostro scopo sarà quello di interfacciare Arduino con ROS tramite il pacchetto ROSSerial.
Il laboratorio sarà alla base per la costruzione dei vostri robot, la cui architettura
è solitamente basata su Arduino Slave per il controllo a basso livello di motori e sensori e
Raspberry Pi con ROS da master che gestisce l'intelligenza.

### Breve tutorial di installazione

Installare `rosserial-python` e `rosserial-arduino``

```bash
sudo apt-get install ros-<DISTRO>-rosserial-python ros-<DISTRO>-rosserial-arduino
```

Generare le librerie da importare in Arduino IDE

```bash
rusrun rosserial-arduino make-libraries.py .
```

Questo creerà una cartella `ros_lib/` nella cartella attuale.

Lanciare Rosserial

```bash
rusrun rosserial-python rosserial-node.py /dev/ttyACM0
```

### Materiale

 - Pacchetto [ROSSerial](http://wiki.ros.org/rosserial)
 - Tutorial [ROSSerial Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials)
 - Arduino (cercate su google le vostre idee, ci sono un'infinità di informazioni)!

### Esempio: Controllare la bocca del robot InMoov Tramite ROS

Vediamo brevemente come controllare la bocca del [Robot InMoov](http://inmoov.fr/).

Recentemente abbiamo sviluppato una testa InMoov in grado di muovere la bocca. Il funzionamento
è molto semplice: un servo motore controlla la mandibola tramite un'accoppiamento vite-madrevite.

![InMoov con ROS](/assets/imgs/2017-12-15-corso-ros-ladispe-polito.md/inmoov.jpg)


Il servo motore è controllato tramite un Arduino sul PIN 11.

Per interfacciarlo con ROS, creiamo un semplice topic `/servo`, di tipo `std_msgs/UInt8`
a cui Arduino si sottoscrive. Su questo topic, verranno mandato la posizione
che il servomotore deve raggiungere.

Ecco il codice Arudino che ho implementato:

```Cpp
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <Servo.h>

ros::NodeHandle  nh;
Servo servo_inmoov;

void servoCb( const std_msgs::UInt8& msg){
  servo_inmoov.write(msg.data);  // blink the led
}

ros::Subscriber<std_msgs::UInt8> sub("servo", &servoCb);

void setup()
{
  servo_inmoov.attach(11);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
```

Ho quindi scritto un semplicissimo nodo ROS che pubblica i comandi sul topic `/servo`
per far aprire e chiudere la bocca al robot.

```python
import rospy
from std_msgs.msg import UInt8

pub = rospy.Publisher('servo', UInt8, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(50)

while not rospy.is_shutdown():
	for i in range(50, 160):
		pub.publish(i)
		r.sleep()
	for i in range(160, 50, -1):
		pub.publish(i)
		r.sleep()
```

Questo è il risutato :D

![InMoov con ROS](/assets/imgs/2017-12-15-corso-ros-ladispe-polito.md/inmoov1.jpg)

![Inmoov Apri la bocca](/assets/imgs/2017-12-15-corso-ros-ladispe-polito.md/inmoov.gif)

## Dove trovare informazioni

- Gruppo Telegram [Robot Secret Lab](https://t.me/joinchat/AXbBRBH-FYXqeLiFGz54bA)
- Gruppo Facebook [Robot Developers Italiani](https://www.facebook.com/groups/493163691070528)
