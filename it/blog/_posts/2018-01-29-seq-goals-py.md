---
title: "Inviare una sequenza di Goals alla ROS NavStack usando Python"
redirect_from:
 - /2018/01/29/seq-goals-py-ita/
layout: post
date: 2018-01-29
image: /assets/imgs/2018-01-29-goal/cover_seq_ita.png
tag:
 - ROS
 - navigation
 - python
author: fiorellazza
description: "Inviare una sequenza di pose desiderate alla ROS Navigation Stack usando un nodo Python"
---
![cover](/assets/imgs/2018-01-29-goal/coverpost.png)

Ciao a tutti!

Se avete letto il mio post, ["Inviare Goals alla Navigation Stack - versione nodo ROS Python"]({{ site.baseurl }}{% post_url /it/blog/2018-01-29-action-client-py %}), adesso dovreste essere in grado di inviare un singlo goal ad un robot mobile usando un nodo python. Che ne dite, invece, di inviare una *sequenza* di pose desiderate? In questo post vi fornirò un esempio per inviare diverse pose desiderate (posizioni cartesiane + orientamento espresso con i quaternioni) per una base mobile alla [ROS Navigation Stack](http://wiki.ros.org/navigation). Questo tutorial è sviluppato scegliendo come base mobile il robot TurtleBot 3 simulato, ma il nodo python è valido per qualunque robot scelto. Prima farò una panoramica sulla soluzione adattata e poi verrà spiegato il codice.

**Nota**: Uso ROS Kinetic. Assumerò che il lettore abbia conoscenze a proposito di [Nodi ROS](http://wiki.ros.org/Nodes), [Topics](http://wiki.ros.org/Topics), [Messaggi](http://wiki.ros.org/msg), [Actions](http://wiki.ros.org/actionlib#Overview) e Parametri ROS [ROS Parameters](http://wiki.ros.org/Parameter%20Server). La lettura del [post]() citato prima e relativa documentazione ROS è consigliata.

### Indice
* TOC
{:toc}

# 1. Download del progetto Github e del pacchetto turtlebot3
 Per poter lavorare con il mio esempio, clonate il progetto github, che potete trovare [qui](https://github.com/FiorellaSibona/turtlebot3_nav), nella vostra location preferita.

 Inoltre servirà il pacchetto ROS turtlebot3 per eseguire la simulazione. Per ROS kinetic:

```bash
 sudo apt-get install ros-kinetic-turtlebot3-*
```

# 2. Goals come parametri ROS
L'idea è quella di salvare come parametri ROS la sequenza di pose desiderate da far processare all'Action Server ed eseguire al nostro robot mobile. Una volta che i dati sono salvati sul ROS Parameter Server, possono essere facilmente recuperati e confezionati successivamente in messaggi ROS predefiniti di tipo Goal tramite il nodo Python, in modo tale che possano essere correttamente interpretati ed eseguiti dall'Action Server.

Salvare i goal come parametri, consente all'utente di modificare solo il launch file, in cui i parametri sono settati, senza alcuna modifica al codice del nodo.

# 3. Launch files
il launch fil [**movebase_seq.launch**](https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/launch/movebase_seq.launch) è molto semplice e, come anticipato, ha il ruolo di settare le positioni e orientamenti desiderati da far assumere alla base mobile. Come potete vedere, il nodo, contenuto nel pacchetto "simple_navigation_goals" e con nome del file specificato nell'argomento *type*, è lanciato con alcuni [*parametri ROS privati*](http://wiki.ros.org/Parameter%20Server#Private_Parameters) specificati nel [tag <*rosparam*>](http://wiki.ros.org/rosparam). Notate che i parametri privati dovranno essere chiamati come *nome_name/nome_parametro*.

Prima viene definita la sequenza di posizioni desiderata nel sistema di riferimento Cartesiano. La lista *p_seq*, per esempio, avendo *n* punti, deve essere interpretata nel seguente modo:

```
p_seq = [x1,y1,z1,x2,y2,z2,...xn,yn,zn]
```

Dopo viene specificata la sequenza di angoli di impabardata (yaw angles) desiderati, espressi in gradi. Infatti, essendo il movimento del robot mobile sul piano xy, possiamo avere una variazione di orientamento solo attorno all'asse z del sistema di riferimento della mappa. Sicuramente il nostro robot non può inclinarsi entrando nel pavimento!
<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/rpy.png" />
</p>
<br>
Gli angoli vanno specificati in gradi per mantenere l'inserimento dati semplice e verranno convertiti in radianti nel nodo. Più dettaglia saranno dati nelle sezioni seguenti.

Il launch file [**gazebo_navigation_rviz.launch**](https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/launch/gazebo_navigation_rviz.launch), setta e lancia i nodi necessari alla visualizzazione del Turtlebot simulato con la mappa che ho ottenuto usando la funzionalità di mapping fornita dal pacchetto turtlebot3_slam (il quale sfrutta [gmapping](http://wiki.ros.org/gmapping)).

La cartella *launch* contiene anche una copia di alcuni file di launch che normalmente lanceremmo così come sono dal pacchetto turtlebot3 package, ma ho avuto bisogno di fare alcune modifiche per specificare la mia mappa e la mia configurazione di rviz (file contenuti nella cartella [/config](https://github.com/FiorellaSibona/turtlebot3_nav/tree/devel/catkin_ws/src/simple_navigation_goals/config)), per darvi in mano un esempio funzionante e già impostato.

# 4. Nodo Python - Codice
Ho usato [questo codice](https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/nodes/move_base_square.py) come riferimento.

Qui c'è il codice completo senza commenti.Per i commenti guarda la [Sezione 5](#5-nodo-python---codice-e-commenti).


```python
#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

```

# 5. Nodo Python - Codice e commenti
Qui viene dato il codice completo di commenti. Per il codice senza commenti, guarda la [Sezione 4](#4-nodo-python---codice).

```python
#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        # Sono necessari solo gli angoli di imbardata (no rotazioni attorno agli assi x e y) in gradi:
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        # Lista dei quaternioni desiderati:
        quat_seq = list()
        # Lista delle pose desiderate:
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            # Spacchettamento della lista di quaternioni e passaggio dei valori al costruttore del messaggio Quaternion
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Restituisce una lista di liste[[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            #Sfrutta la variabile n per ciclare in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        #Crea un action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #Per stampare la posa corrente ad ogni feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Riferimento per i valori dello goal status: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
```

- Notate che l'operatore Pyhton [\*](https://docs.python.org/2/tutorial/controlflow.html#unpacking-argument-lists) è qui utilizzato come operatore di "spacchettamento", cioè estrae i valori della lista.

- Per ulteriori informazioni riguardo la funzione *quaternion_from_euler*, guardate [qui](https://www.lfd.uci.edu/~gohlke/code/transformations.py.html) and [here](https://answers.ros.org/question/53688/euler-angle-convention-in-tf/).

- Notate che il nodo Python è stato definito come classe per semplificare il codice in caso di uso futuro.

# 6. Setup e simulazione
Adesso che avete capito tutto della mia soluzione (si spera!), dovete solo eseguire i file di launch e vedrete il turtlebot muoversi verso le pose desiderate!
## 6.1. Settare il modello per Turtlebot
Per evitare l'errore a proposito del modello ogni volta che un nodo del pacchetto turtlebot3 viene lanciato, vi suggerito di eseguire questo comando:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

## 6.2. Lanciare Gazebo e Rviz
Ricordate sempre di *runnare* i file di setup di ROS e di catkin. Quindi eseguite:

```bash
roslaunch simple_navigation_goals gazebo_navigation_rviz.launch
```

Il launch file [gazebo_navigation_rviz.launch](https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/launch/gazebo_navigation_rviz.launch) avvia Gazebo e Rviz insieme ai nodi di navigazione.
<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/6.2.0.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.2.1.png"  height="250" />
</p>

## 6.3. Settare la posa corrente di Turtlebot
Per eseguire tutti gli step per spostarsi alle pose desiderate, il turtlebot ha bisogno di sapere (almeno approssimativamente) dove si trova sulla mappa. Per fare ciò, in Rviz, premete il bottone **2D Pose Estimate**, cliccate poi nella posizione approssimativa dove viene visualizzato il turtlebot in Gazebo e, prima di rilasciare, settate anche il suo orientamento.
<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/6.3.2.png"  height="30"/>
</p>
<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/6.2.1.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.3.0.png"  height="250" />
</p>
<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/6.2.1.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.3.1.png"  height="250"/>
</p>

## 6.4. Lanciare il nodo movebase_seq e caricare i parametri
In un nuovo terminale eseguire il seguente comando:

```bash
roslaunch simple_navigation_goals movebase_seq.launch
```

La pianificaione di navigazione può necessitare di alcuni istanti ma dovreste vedere il turtlebot andare verso le pose desiderate definite nel launch file.
Il percorso *verde* è quello calcolato dal global planner mentre quello *blu* è il path calcolato dal local planner il quale cambia frequentemente, a seconda della percezione del robot di ciò che lo circonda, nel tempo.

<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/6.4.0.png"  height="250"/>
    <image src="/assets/imgs/2018-01-29-goal/6.4.1.png"  height="250"/>
</p>

Sul terminale dovreste vedere alcune informazioni a proposito di come sta procedendo l'esecuzione del goal corrente.

Adesso dovreste avere un esempio funzionante per inviare una sequenza di pose alla navigation stack sul vostro robot.

**A presto!** :hibiscus:
