---
title: "Controllare siBOT dalla piattaforma HBR"
layout: post
date: 2018-02-14
image: /assets/imgs/2018-02-07-sibot-cloud/cover.png
lang: it
tag:
 - sibot
 - Arduino
 - Hbrain
 - Cloud
 - Python

author: fiorellazza
description: "Come controllare il manipolatore siBOT utilizzando la piattaforma HBR"
---
In questo tutorial vedremo come collegare il manipolatore antropomorfo, siBOT, alla piattaforma HBR.

siBOT è l'insieme del design [EEZYBOT MK2](http://www.eezyrobots.it/eba_mk2.html), progetto italino Open Source, e del sistema di nodi ROS necessari a controllarlo. Ho utilizzato questo braccio per testare l'architettura sviluppata per il mio progetto di tesi, NTBD, di cui trovate maggiori informazioni in questo [post]({{ site.baseurl }}{% post_url /it/blog/2018-01-17-ntbd-guide-part-I %}).

<p align="center">
    <image src="/assets/imgs/2018-01-17-ntbd/sibot.png"  height="250"/>
</p>
Vedremo quali sono gli step necessari a collegare siBOT in Cloud per controllarlo da piattaforma HBR, sia nel **joint space** (invio degli angoli desiderati per i motori), sia nel **task space** (in questo caso invio posizioni desiderate nello spazio cartesiano che vengono convertite in angoli per i motori). 
### Indice
* TOC
{:toc}

# 1. Ingredienti
 Ci serviranno: 
- 1 braccio siBOT (braccio EEZYBOT MK2 + nodo ROS Arduino)
- 1 Raspberry Pi con l'immagine HBrain sull'SD

# 2. Controllo nello spazio dei motori
<p align="center">
    <image src="/assets/imgs/2018-01-17-ntbd/5_eezybotfrontservo.jpg"  height="300"/>
</p>
Il controllo più semplice è quello nello spazio dei motori: basterà infatti decidere quali valori vogliamo dare ai tre servo motori ed inviarli al robot.
Possiamo definire una sequenza di configurazioni:
```
motors_list = [m11,m21,m31,m12,m22,m32,...m1n,m2n,m3n]
```
e leggerla correttamente nel nostro sketch per inviare ciascuna sequenza in modo ciclico per far eseguire al robot una sorta di "routine".

## 2.1 Generatore di valori servo desiderati - Sketch 
Create un nuovo sketch e chiamatelo *motors_generator_sibot*. Il contenuto dovrà essere il seguente:

```python
import dotbot_ros
from std_msgs.msg import Int16MultiArray
import time

class Node(dotbot_ros.DotbotNode):
    node_name = 'motors_generator'

    def setup(self):
        self.pub = dotbot_ros.Publisher('motors_nointerp', Int16MultiArray)
        self.gripper = 25
        
        motors_list = [90, 90, 90, 180, 100, 30, 0, 140, 60]
        self.gripp_list = ['open','closed','open']
        n = 3
        self.m_list = [motors_list[i:i+n] for i in range(0, len(motors_list), n)]     
        self.loop_rate = dotbot_ros.Rate(0.33)
    
    def loop(self):
        for indx, motors_seq in enumerate(self.m_list): 
            seq = Int16MultiArray()
            if self.gripp_list[indx] == 'open':
                self.gripper = 100
            else:
                self.gripper = 25
            seq.data = [m for m in motors_seq, self.gripper]
            self.pub.publish(seq)
            time.sleep(3)
```

## 2.2 Generatore di valori servo desiderati - Sketch con commenti
Qui di seguito riporto il codice commentato. Come potete vedere, la struttura è quella utilizzata finora per i nodi ROS su HBrain: vengono definite le funzioni setup() e loop(), con l'aggiunta di una funzione di callback.

```python
import dotbot_ros
from std_msgs.msg import Int16MultiArray
import time

class Node(dotbot_ros.DotbotNode):
    node_name = 'motors_generator'

    def setup(self):
    # Definiamo un publisherper pubblicare le sequenze di motori
        self.pub = dotbot_ros.Publisher('motors_nointerp', Int16MultiArray)
    # Inizializziamo il valore del gripper a chiuso
        self.gripper = 25
    # Sequenza di angoli desiderati (da leggere 3 alla volta)
        motors_list = [90, 90, 90, 180, 100, 30, 0, 140, 60]
    # Sequenza di configurazioni desiderate per il gripper
        self.gripp_list = ['open','closed','open']
    # Come detto, prendiamo i valori in gruppi da n elementi
        n = 3
    # Il seguente codice ritorna una lista di liste da 3 elementi: 
    # [[motor_seq1], [motor_seq2],...[motor_seqn]]
        self.m_list = [motors_list[i:i+n] for i in range(0, len(motors_list), n)]     
        self.loop_rate = dotbot_ros.Rate(0.33)
    
    def loop(self):
    # Durante il ciclo for nella lista di liste, salvo anche l'indice 
    # di ogni elemento così da poter ciclare nella lista di valori per il gripper
        for indx, motors_seq in enumerate(self.m_list): 
            seq = Int16MultiArray()
            if self.gripp_list[indx] == 'open':
                self.gripper = 100
            else:
                self.gripper = 25
            seq.data = [m for m in motors_seq, self.gripper]
    # Pubblichiamo sul topic definito da pub la sequenza di motori, 
    # unione degli angoli dei miniservo dei giunti e il valore per il gripper.
            self.pub.publish(seq)
            time.sleep(3)
```

# 3. Controllo della posizione dell'End Effector
Abbiamo visto come controllare i valori dei motori di siBOT, ma spesso nella robotica industriale l'obbiettivo è controllare la posizione e l'orientamento dell'**End Effector** (**EE**) che in questo caso corrisponde al giunto della pinza (gripper).
<p align="center">
    <image src="/assets/imgs/2018-02-07-sibot-cloud/5_trigoreal.jpeg"  height="300"/>
</p>
Per poter ottenere la posizione desiderata è necessario manipolare questa informazione e trasformarla in valori per i motori del braccio robotico, i quali sono l'unico modo per muovere il robot stesso. Questa operazione si chiama **cinematica inversa** e tramite calcoli, analitici o numerici, consente di trovare il valore dei motori corrispondenti ad una certa posizione dell'EE. Notate che in questo caso, possiamo definire solo la posizione desiderata (non l'orientamento) dal momento che la pinza non ruota per configurazione fisica del braccio.
Anche in questo caso definiamo una sequenza di posizioni desiderate che vorremmo l'EE del braccio raggiungesse.

## 3.1 Generatore di posizioni EE  - Sketch
Create un nuovo sketch e chiamatelo *positions_generator_sibot*. Il contenuto dovrà essere il seguente:

```python
import dotbot_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time

class Node(dotbot_ros.DotbotNode):
    node_name = 'position_generator'

    def setup(self):
        self.pub = dotbot_ros.Publisher('desired_position_nointerp', Point)
        self.pubG = dotbot_ros.Publisher('gripper_value', String)

        desPos = [150, 0, 235, 130, 50, 140, 130, -30, 90]
        self.gripp_list = ['open','closed','open']
        n = 3
        self.desP = [desPos[i:i+n] for i in range(0, len(desPos), n)]
        self.loop_rate = dotbot_ros.Rate(0.33)
    
    def loop(self):
        for indx,pos in enumerate(self.desP):
            self.pubG.publish(self.gripp_list[indx])
            des_pos =  Point(*pos)
            self.pub.publish(des_pos)
            time.sleep(3)
```

## 3.2 Generatore di posizioni EE desiderate - Sketch con commenti
Come potete notare, nel caso delle posizioni desiderate non abbiamo la funzione di callback all'arrivo di messaggi sul topic in cui viene specificato lo stato della pinza (gripper_value): questa funzione di callback verrà infatti definita nel nodo per la cinematica inversa in cui vengono pubblicati i valori dei servo-motori insieme al valore per il gripper sull'apposito topic.

```python
import dotbot_ros
from geometry_msgs.msg import Point
import time

class Node(dotbot_ros.DotbotNode):
    node_name = 'position_generator'

    def setup(self):
    # Definiamo un publisher per pubblicare la sequenza di posizioni 
    # per l'EE ed uno per il valore del gripper
        self.pub = dotbot_ros.Publisher('desired_position_nointerp', Point)
        self.pubG = dotbot_ros.Publisher('gripper_value', String)
    # Posizioni in coordinate cartesiane espresse in millimetri
    # (da leggere [x1,y1,z1,x2,y2,z2...xn,yn,zn])
        desPos = [150, 0, 235, 130, 50, 140, 130, -30, 90]
    # Sequenza di configurazioni desiderate per il gripper
        self.gripp_list = ['open','closed','open']
    # Come detto, prendiamo i valori in gruppi da n elementi
        n = 3
    # Il seguente codice ritorna una lista di liste da 3 elementi. Ogni lista 
    # è una posizione in coordinate Cartesiane: 
    # [[pos1], [pos2],...[posn]]
        self.desP = [desPos[i:i+n] for i in range(0, len(desPos), n)]
        self.loop_rate = dotbot_ros.Rate(0.33)
    
    def loop(self):
    # Durante il ciclo for nella lista di liste, salvo anche l'indice 
    # di ogni elemento così da poter ciclare nella lista di valori per il gripper
        for indx,pos in enumerate(self.desP):
    # Pubblichiamo sul topic gripper_value la stringa per definire se la pinza
    # debba essere aperta o chiusa in quella configurazione
            self.pubG.publish(self.gripp_list[indx])
            des_pos =  Point(*pos)
    # Pubblichiamo sul topic definito da pub la posizione desiderata, come messaggio Point
            self.pub.publish(des_pos)
            time.sleep(5)
```
## 3.3 Nodo di cinematica inversa - Sketch
Una volta che le posizioni desiderate verranno pubblicate, sarà necessario convertirle nei valori dei servo corrispondenti. Esistono molte soluzioni a seconda della complessità del problema (per esempio i gradi di libertà, Degrees Of Freedom) ma quella calcolata da me è di tipo geometrico, ancora possibile visti i 3 DOF del braccio.
Create un nuovo sketch e chiamatelo *IK_sibot*. Il contenuto dovrà essere il seguente:

```python
import dotbot_ros
import math
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sys import stdout

class Node(dotbot_ros.DotbotNode):
    node_name = 'IK'

    def setup(self):
        dotbot_ros.Subscriber("desired_position", Point, self.callback)
        dotbot_ros.Subscriber("gripper_value", String, self.gripper_callback)
        self.pub = dotbot_ros.Publisher('motors', Int16MultiArray)
        self.gripper = 25
        
    def hipo(self,x,y):
        return math.sqrt(x*x + y*y)

    def lawOfCosines(self,a,b,c):
        rate = (a*a + b*b - c*c) / (2 * a * b)
        if abs(rate) > 1:
            if max(rate,0) == 0:
                rate = -1
            if max(rate,0) == rate:
                rate = 1
        return math.acos(rate)

    def deg(self,rad):
        return rad * 180 / math.pi
    
    def gripper_callback(self, msg):
        if msg.data == "open":
            self.gripper = 100
        else:
            self.gripper = 25
            
    def callback(self,data):
        L0 = 50
        L1 = 35
        L2 = 150 
        L3 = 150
        cartP = {'xEE':data.x, 'yEE': data.y, 'zEE': data.z}

        L = L0 + L1
        cylP = {'theta': math.atan(cartP['yEE']/cartP['xEE']), 'r':self.hipo(cartP['xEE'], cartP['yEE']), 'zhat':cartP['zEE']-L}
        zhat = cylP['zhat']
        rho = self.hipo(cylP['r'], zhat)

        M1 = 2*cylP['theta'] + math.pi/2
        M2 = math.atan(zhat/cylP['r']) + self.lawOfCosines(L2,rho,L3)
        M3 = M2 + self.lawOfCosines(L2,L3,rho) - math.pi/2
        angles = [M1,math.pi - M2,M3]
        values = Int16MultiArray()
        values.data = [self.deg(angle) for angle in angles]
        values.data.append(self.gripper)

        if values.data[0] > 180:
            values.data[0] = 180
            print " motor 1 has been saturated!"
        if values.data[1] > 145:
            values.data[1] = 145
            print " motor 2 has been saturated!"
        if values.data[1] < 55:
            values.data[1] = 55 
            print " motor 2 has been saturated!"
        if values.data[2] > 110:
            values.data[2] = 110
            print " motor 3 has been saturated!"
        if values.data[2] < 20:
            values.data[2] = 20
            print " motor 3 has been saturated!" 
        stdout.flush()  
        self.pub.publish(values)
```
## 3.4 Nodo di cinematica inversa - Sketch con commenti

```python
import dotbot_ros
import math
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sys import stdout

class Node(dotbot_ros.DotbotNode):
    node_name = 'IK'

    def setup(self):
        dotbot_ros.Subscriber("desired_position", Point, self.callback)
        dotbot_ros.Subscriber("gripper_value", String, self.gripper_callback)
        self.pub = dotbot_ros.Publisher('motors', Int16MultiArray)
        self.gripper = 25

# Qui vengono definite alcune funzioni utili a risolvere la cinematica inversa
    def hipo(self,x,y):
        return math.sqrt(x*x + y*y)

    def lawOfCosines(self,a,b,c):
        rate = (a*a + b*b - c*c) / (2 * a * b)
        if abs(rate) > 1:
            if max(rate,0) == 0:
                rate = -1
            if max(rate,0) == rate:
                rate = 1
        return math.acos(rate)

    def deg(self,rad):
        return rad * 180 / math.pi
# Funzione di callback per impostare il valore del servo della pinza 
# a seconda della stringa pubblicata su gripper_value
    def gripper_callback(self, msg):
        if msg.data == "open":
            self.gripper = 100
        else:
            self.gripper = 25
# Funzione di callback, chiamata alla ricezione della posizione desiderata 
# sull'apposito topic, in cui vengono calcolati i valori per i servo ed uniti al valore del gripper
    def callback(self,data):
        L0 = 50
        L1 = 35
        L2 = 150 
        L3 = 150
# Posizione desiderata estrapolata dal messaggio di tipo Point
        cartP = {'xEE':data.x, 'yEE': data.y, 'zEE': data.z}
        L = L0 + L1
# Posizione desiderata in coordinate cilindriche
        cylP = {'theta': math.atan(cartP['yEE']/cartP['xEE']), 'r':self.hipo(cartP['xEE'], cartP['yEE']), 'zhat':cartP['zEE']-L}
        zhat = cylP['zhat']
        rho = self.hipo(cylP['r'], zhat)

        M1 = 2*cylP['theta'] + math.pi/2
        M2 = math.atan(zhat/cylP['r']) + self.lawOfCosines(L2,rho,L3)
        M3 = M2 + self.lawOfCosines(L2,L3,rho) - math.pi/2

        angles = [M1,math.pi - M2,M3]
        values = Int16MultiArray()
        values.data = [self.deg(angle) for angle in angles]
        values.data.append(self.gripper)
# Limitiamo i valori dei motori ai limiti definiti dalla struttura fisica del robot 
        if values.data[0] > 180:
            values.data[0] = 180
            print " motor 1 has been saturated!"
        if values.data[1] > 145:
            values.data[1] = 145
            print " motor 2 has been saturated!"
        if values.data[1] < 55:
            values.data[1] = 55 
            print " motor 2 has been saturated!"
        if values.data[2] > 110:
            values.data[2] = 110
            print " motor 3 has been saturated!"
        if values.data[2] < 20:
            values.data[2] = 20
            print " motor 3 has been saturated!"
        stdout.flush()
# Pubblichiamo sul topic definito da pub la sequenza di motori, 
# unione degli angoli dei miniservo dei giunti e il valore per il gripper.
        self.pub.publish(values)
```
# 4. Interpolazione
I nodi per controllare il manipolatore sono pronti però manca ancora un nodo che renda i movimenti da una configurazione all'altra più fluidi: un nodo di interpolazione. Per mantenere il tutto semplice assumiamo che non ci siano ostacoli da evitare ed implementiamo un'interpolazione di tipo lineare. Implementiamo quindi un nodo di path planning molto semplice.
## 4.1 Nodo di path planning - Sketch con commenti
Procederò direttamente a riportare il codice commentato. Copiate questo codice in uno sketch chiamato *linear_interp_sibot*.

```python
import dotbot_ros
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Int16MultiArray
import time

class Node(dotbot_ros.DotbotNode):
    node_name = 'interpolator'

    def setup(self):
# Definiamo i publisher e subscriber
        self.pub = dotbot_ros.Publisher('desired_position', Point)
        self.pubM = dotbot_ros.Publisher('motors', Int16MultiArray)
        dotbot_ros.Subscriber("desired_position_nointerp", Point, self.callback)
        dotbot_ros.Subscriber("motors_nointerp", Int16MultiArray, self.motors_callback)
        self.i = 0
        self.pointA = Point()
        self.pointB = Point()
        self.motorA = Int16MultiArray()
        self.motorB = Int16MultiArray()
# Qui vengono alcune funzioni utili al calcolo dei punti di interpolazione
    def coord_distance_AB(self,a,b):
        d = Point()
        d.x = abs(b.x-a.x)
        d.y = abs(b.y-a.y)
        d.z = abs(b.z-a.z)
        return d
    
    def values_distance_AB(self,a,b):
        d = Int16MultiArray()
        d.data.append(abs(b.data[0]-a.data[0]))
        d.data.append(abs(b.data[1]-a.data[1]))
        d.data.append(abs(b.data[2]-a.data[2]))
        return d

# Funzione di callback chiamata alla ricezione di una sequenza di valori per i motori
# Legge la sequenza corrente poi la paragona a quella precedente per calcolare i valori intermedi
# in 20 steps (N=20)
    def motors_callback(self,data):
        N = 20
        self.i += 1
        if self.i == 1:
            self.motorA = data
            self.pubM.publish(self.motorA)
        else: 
            self.motorB = data
            d = self.values_distance_AB(self.motorA, self.motorB)
            if d.data[0] == 0 and d.data[1] == 0 and d.data[2] == 0 and d.data[3] == 0 :
                self.motorA = self.motorB
                self.pubM.publish(self.motorA)
            else:
                for self.i in range(1,N+1):
                    M = Int16MultiArray()
                    if self.motorA.data[0] < self.motorB.data[0]:
                        M.data.append(self.motorA.data[0] + d.data[0]/N)
                    else:
                        M.data.append(self.motorA.data[0] - d.data[0]/N)
                    if self.motorA.data[1] < self.motorB.data[1]:
                        M.data.append(self.motorA.data[1] + d.data[1]/N)
                    else:
                        M.data.append(self.motorA.data[1] - d.data[1]/N)
                    if self.motorA.data[2] < self.motorB.data[2]:
                        M.data.append(self.motorA.data[2] + d.data[2]/N)
                    else:
                        M.data.append(self.motorA.data[2] - d.data[2]/N)

                    M.data.append(self.motorB.data[3])

                    self.pubM.publish(M)
                    self.motorA = M
                    self.i += 1
                    time.sleep(0.05)
# Funzione di callback chiamata alla ricezione di una nuova posizione per l'EE
# Legge la posizione corrente poi la paragona a quella precedente per calcolare i valori intermedi
# in 20 steps (N=20)
    def callback(self,data):
        N = 20
        self.i += 1
        if self.i == 1:
            self.pointA = data
            self.pub.publish(self.pointA)
        else: 
            self.pointB = data
            d = self.coord_distance_AB(self.pointA, self.pointB)
            if d.x == 0 and d.y == 0 and d.z == 0:
                self.pointA = self.pointB
                self.pub.publish(self.pointA)
            else:
                for self.i in range(1,N+1):

                    P = Point()

                    if self.pointA.x < self.pointB.x:
                        P.x = self.pointA.x + d.x/N
                    else:
                        P.x = self.pointA.x - d.x/N

                    if self.pointA.y < self.pointB.y:
                        P.y = self.pointA.y + d.y/N
                    else:
                        P.y = self.pointA.y - d.y/N

                    if self.pointA.z < self.pointB.z:
                        P.z = self.pointA.z + d.z/N
                    else:
                        P.z = self.pointA.z - d.z/N
                    self.pub.publish(P)
                    self.pointA = P
                    self.i += 1
                    time.sleep(0.05)
```
NOTA: i valori per i servo possono essere solo interi, l'interpolazione quindi genera valori che possono essere diversi da quelli desiderati.
# 5. Sketch Arduino
Per poter controllare il manipolatore è necessario controllarne i motori con una scheda Arduino su cui verrà eseguito un nodo ROS seriale che leggerà i messaggi sui topic da noi specificati.
Caricate sulla vostra scheda Arduino il seguente sketch:
```c++
/*
 * siBOT servo control
 */
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#define USE_USBCON

ros::NodeHandle  nh;

Servo servo1, servo2, servo3, servo4;

// Funzione di callback (quando una nuova sequenza di motori viene pubblicata
// scrivi gli angoli sui rispettivi pin)
void motors_cb( const std_msgs::Int16MultiArray& angles_msg){
  
  servo1.write(angles_msg.data[0]);
  servo2.write(angles_msg.data[1]);
  servo3.write(angles_msg.data[2]);
  servo4.write(angles_msg.data[3]);
  
}

// Definizione del subscriber. Notate che il topic è specifico al robot su cui stanno
// eseguendo i nodi ROS (/hotbot/ in questo caso)
ros::Subscriber<std_msgs::Int16MultiArray> sub("/hotbot/motors",motors_cb);

void setup(){
    
  nh.initNode();
  nh.subscribe(sub);
  
  servo1.attach(2); //attach it to pin 2
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
```

# 6. Avvio di Rosserial e run dei nodi 
Affinchè il nodo seriale che esegue sulla scheda Arduino venga reso noto al sistema ROS, un nodo python seriale apposito deve essere lanciato. Per fare ciò attraverso la piattaforma, bisogna aprire questo [link](http://cloud.hotblackrobotics.com/cloud/webgui/camera) e cliccare su "Apri Manager Robot". Una volta aperta la pagina, dove compare "rosserial" cliccare su start per avviare il nodo seriale.

Una volta lanciato il nodo seriale, non ci resta che runnare i nodi necessari al controllo del robot, a seconda del controllo scelto:

- Controllo nello spazio dei motori: motors_generator_sibot, linear_interp_sibot
- Controllo della posizione dell'EE: positions_generator_sibot, IK_sibot, linear_interp_sibot

# 7. Esercizi

- Scrivere uno sketch chiamato *set_motors_sibot* che invii una sequenza per i motori ad ogni esecuzione di loop(). 
- Scrivere uno sketch chiamato *set_position_sibot* che invii una posizione per l'EE ad ogni esecuzione di loop().

**Hint**: sfruttate il codice fornito e modificatelo per renderlo più semplice ed usarlo come base per pubblicare sui topic giusti. Inoltre, per fare in modo che l'interpolazione venga eseguita basta runnare lo sketch *linear_interp_sibot* oltre allo sketch che avrete creato.

**Ciao ciao!** :hibiscus: