---
title: "Inviare Goals alla Navigation Stack - versione nodo ROS Python"
redirect_from:
 - /2018/01/29/action-client-py-ita/
layout: post
date: 2018-01-29
image: /assets/imgs/2018-01-29-goal/cover.png
tag:
 - ROS
 - navigation
 - Python
author: fiorellazza
description: "Inviare un goal all ROS navigation stack utilizzando un nodo Python"
---
Ehilà!

Questo post ha l'obbiettivo di fornire un esempio in codice Python per inviare una posa *goal* (posizione ed orientamento desiderati) ad un robot, nel mio caso un robot [TurtleBot3](http://wiki.ros.org/Robots/TurtleBot) simulato, sfruttando la [ROS Navigation Stack](http://wiki.ros.org/navigation). Solitamente, ad un robot autonomo mobile viene richiesto di raggiungere un'ubicazione desiderata. Per fare ciò, deve avere alcune informazioni e combinarle tra loro: avere una mappa dell'ambiente in cui si trova, percepire ciò che lo circonda, localizzare se' stesso e pianificare i propri movimenti. La ROS Navigation Stack assume il ruolo di "guidare" la base mobile a muoversi verso quell'obbiettivo, evitando eventuali ostacoli e mettendo insieme tutte le informazioni a disposizione.
Usando codice, l'utente può inviare alla navigation stack una posa desiderata da far assumere al robot. Per scrivere il nodo correttamente, ho seguito il [tutorial per il nodo C++ "Sending Goals to the Navigation Stack"](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals), quindi il mio scopo è quello di darvi un equivalente per il nodo Python.

**Nota**: Uso ROS Kinetic. Assumerò che il lettore abbia conoscenze a proposito di [Nodi ROS](http://wiki.ros.org/Nodes), [Topics](http://wiki.ros.org/Topics), [Messaggi](http://wiki.ros.org/msg) e [Actions](http://wiki.ros.org/actionlib#Overview). Alcune informazioni a proposito di queste ultime verranno date durante la descrizione delle librerie.

### Indice:
* TOC
{:toc}

# 1. La libreria ***actionlib***
La ROS navigation stack è basata sulle ROS Actions: infatti le Actions sono la scelta migliore nei casi in cui un nodo voglia inviare una richiesta ad un altro nodo e riceverà una risposta dopo un tempo relativamente lungo. Per evitare che l'utente si chieda che cosa stia succedendo e se tutto stia andando come desiderato o meno, le Actions implementano un meccanismo di *feedback*, il quale permette all'utente di ricevere informazioni di tanto in tanto. Le Actions sono basate sul paradigma Client-Server: la [libreria **actionlib**](http://wiki.ros.org/actionlib#Client-Server_Interaction) fornisce gli strumenti e un'interfaccia per creare un Action Server che esegua le richieste di goal inviate dal Client. Gli elementi principali del meccanismo delle ROS actions sono: *goal*, *result*, e *feedback*. Ognuno di essi è specificato da un tipo di Messaggio ROS, contenuto in un *action definition file*, con estensione "*.action*".

Per maggiori informazioni fate riferimento alla [descrizione dettagliata di actionlib](http://wiki.ros.org/actionlib/DetailedDescription).

# 2. Il nodo MoveBase
Il [nodo ROS move_base](http://wiki.ros.org/move_base), è il componente più importante della navigation stack il quale permette di configurare, runnare ed interagire con quest'ultima. Il nodo move_base implementa un *SimpleActionServer*, un action server con la restrizione di ricevere un solo goal alla volta, che riceve goals in messaggi di tipo [*geometry_msgs/PoseStamped*](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html). Per comunicare con questo nodo, viene usata l'interfaccia SimpleActionClient. Il nodo move_base cerca di raggiungere la posa desiderata combinando un motion planner globale ed uno locale per portare a termine il task di navigazione il quale include evitare ostacoli.

<p align="center">
    <image src="/assets/imgs/2018-01-29-goal/movebase.png"/>
</p>
<br>
# 3. Creazione del Nodo - Codice
Ecco qui il codice dell'intero nodo senza commenti. Per i commenti andate alla [Sezione 4](#4-creazione-del-nodo---codice-e-commenti).

```python
#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
```

# 4. Creazione del Nodo - Codice e commenti
Ecco qui il codice completo di commenti. L'intero codice del nodo senza commenti è dato nella  [Sezione 3](#3-creazione-del-nodo---codice).

```python
#!/usr/bin/env python
# license removed for brevity

import rospy

# Importa il SimpleActionClient
import actionlib
# Importa il file .action ed i messaggi usati dalla move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

   # Crea un action client chiamato "move_base" con action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

   # Aspetta che l'action si sia avviato ed abbia iniziato ad essere ricettivo per i goal
    client.wait_for_server()

   # Crea un nuovo goal con il costruttore MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Muovere di 0.5 metri avanti lungo l'asse x del sistema di riferimento della mappa
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.orientation.w = 1.0

   # Invia il goal all'action server.
    client.send_goal(goal)
   # Aspetta che il server finisca di eseguire la richiesta
    wait = client.wait_for_result()
   # Se il risultato non arriva, assumiamo che il Server non sia disponibile
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Restituisce il risultato dell'esecuzione dell'action
        return client.get_result()   

# Se il nodo Python viene eseguito come processo principale (eseguito direttamente)
if __name__ == '__main__':
    try:
       # Inizializza un nodo rospy per permettere al SimpleActionClient di interagire in ROS
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
```

Abbiamo finito! Questo è un semplice esempio di nodo Python per inviare una posa desideata alla navigation stack per muovere un robot mobile. Come potete notare, per motivi di semplicità, essendo questo un tutorial base, non vengono sfruttati i meccanismi di feedback propri delle Actions ed il risultato non è indicativo del reale status del goal. Per avere un esempio più completo, vi consiglio la lettura del post ["Inviare una sequenza di Goals alla ROS NavStack usando Python"]().

**Grazie per l'attenzione e a presto!** :hibiscus:
