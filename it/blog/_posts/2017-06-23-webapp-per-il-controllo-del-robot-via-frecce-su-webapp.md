---
title: "Webapp per il controllo del robot via frecce su WebApp"
layout: post
date: 2017-06-23 14:31:31
image:
headerImage: false
lang: it
tag:

redirect_from: /blog/posts/2017-06-23-webapp-per-il-controllo-del-robot-via-frecce-su-webapp
author: Ruslan
description: ""
---

![enter image description here](http://i.imgur.com/7GYgCAW.png)
In questo tutorial vedremo come creare una WebApp in grado di gestire il robot attraverso dei bottoni che fungono da frecce sulla pagina web. In particolare vedremo:

 - come sviluppare una WebApp in grado di controllare il robot
   attraverso le frecce (UP,DOWN,LEFT,RIGHT)
 - come gestire la velocità del robot attraverso uno slider
 - come sviluppare lo sketch in ROS per sottoscriversi al topic della   
   WebApp


La webapp
---------

 Come prima cosa andiamo a sviluppare l'applicazione web che ci permetterà di controllare il robot tramite le frecce. Il codice Html lo potete trovare [qui](https://github.com/ganduras/webapp1/blob/master/index.html).

**Analizziamo il codice:**
All'interno del file abbiamo la prima parte scritta in JavaScript che permette alla Webapp di comunicare con la piattaforma (riga 42).

    <script type="text/javascript">
    start_ros('192.168.0.108', 'cyberbot', '192.168.0.108', '192.168.0.108/bridge/');
    </script>
 IMPORTANTE: ricordate di modificare i campi '*192.168.0.108*' e '*cyberbot*' inserendo l'IP e il nome del bot.

La seconda parte di questo file JavaScript si occupa di gestire la velocità (riga 47).

    <script>
    $(document).ready(function(){

                function update() {


                    var cmdJoy = new ROSLIB.Topic({
      		ros : ros,
      		name : '/' + robot.name +'/velocita',
      		messageType : 'std_msgs/Float32'
    		});


                    var tasks_time = $('#tasks_time').slider('value');


    	var joy = new ROSLIB.Message({
    	"data": tasks_time
    	});
    	cmdJoy.publish(joy);
     }

               $( "#tasks_time" ).slider({
                    range: "max",
                    step: 0.1,
                    min: 0,
                    max: 1,
                    stop: function() {
                        update();

                    }
                });


            });



    </script>

La funzione *update()*  è un  nodo publisher  che si sottoscrive al topic */velocita*  ed invia i dati ogni volta che lo slider viene trascinato.
La velocità deve essere espressa nel range tra 0 e 1 quindi utilizziamo il formato *std_msgs/Float32*

La terza funzione è *myFunction* (riga 209) , questa funzione viene chiamata ogni volta che l'utente fa click su uno dei pulsanti. Come parametro la funzione riceve la posizione di tipo String.
Ogni volta che la funzione viene chiamata , il nodo si sottoscrive al topic */comando* ed invia la direzione.
A questo punto la parte relativa alla WebApp è completa e possiamo iniziare a scrivere lo sketch in Python su ROS.

Sketch ROS
----------

Il primo passo è quello di importare le librerie:

    import dotbot_ros
    from sys import stdout
    from std_msgs.msg import String
    from std_msgs.msg import Float32
    from gpiozero import Robot

   La prima libreria è ovviamente quella di ROS, la seconda è *stdout* che ha il compito di forzare la stampa effettiva sulla shell. Le librerie successive sono quelle che ci servono per estrarre i valori, cioè String per la direzione e Float32 per la velocità.  Importiamo l'oggetto Robot dalla libreria gpiozero per gestirne il movimento del robot.

Come al solito, il nostro programma è composto da un nodo ROS, la funzione principale è la funzione setup, che si occupa di inizializzare il robot e creare una callback di gestione.

    self.speed = 0
    self.posizione = 'none'
    self.robot = Robot(left=(9, 10), right=(7, 8))
    dotbot_ros.Subscriber('comando', String, self.direzione)
    dotbot_ros.Subscriber('velocita', Float32, self.speedy)
Inizializziamo le variabili speed e posizione. Dopodichè cambiamo le coppie di Pin GPIO a cui sono collegate i due motori.

Sottoscriviamoci ai Topic ROS e usiamo le Callback.
---------------------------------------------------

     dotbot_ros.Subscriber('comando', String, self.direzione)
Serve per sottoscriversi al nodo *comando*, ricevere una Stringa e passare i dati alla funzione di callback self.direzione.
La stessa cosa vale per la velocità:

    dotbot_ros.Subscriber('velocita', Float32, self.speedy)
Ci sottoscriviamo al nodo *velocita* , riceviamo un messaggio di tipo Float32 e passiamo i dati alla funzione di callback *speedy*.

La funzione che mette insieme la direzione e la velocità è:

    def controller(self,speed,posizione):
            if self.posizione == 'avanti':
                self.robot.forward(self.speed)

            elif self.posizione == 'indietro':
                self.robot.backward(self.speed)

            elif self.posizione == 'destra':
                self.robot.right(self.speed)

            elif self.posizione == 'sinistra':
                self.robot.left(self.speed)

            elif self.posizione == 'stop':
                self.robot.stop()
Questa funzione riceve come parametro  *self* (rappresenta il nodo), *speed* (la velocità) e *posizione*.
 La funzione *controller* viene chiamata in 2 casi:

 1.

    def speedy(self, msg):
                self.speed = msg.data
                self.controller(self.speed,self.posizione)

Quando viene eseguita la funzione di callback "speedy" .

 2.  

    def direzione(self, msg):
            self.posizione = msg.data
            self.controller(self.speed,self.posizione)

Quando viene eseguita la funzione di callback "direzione".
N.B: Queste due funzioni non possono funzionare indipendentemente. Perchè il robot non può muoversi senza la velocità e senza direzione.

Codice completo
---------------

Ecco il codice completo del nostro programma

    import dotbot_ros
    from sys import stdout
    from std_msgs.msg import String
    from std_msgs.msg import Float32
    from gpiozero import Robot


    class Node(dotbot_ros.DotbotNode):
        node_name = 'webapp'


        def setup(self):  
            self.speed = 0
            self.posizione = 'none'
            self.robot = Robot(left=(9, 10), right=(7, 8))
            dotbot_ros.Subscriber('comando', String, self.direzione)
            dotbot_ros.Subscriber('velocita', Float32, self.speedy)

        def controller(self,speed,posizione):
            if self.posizione == 'avanti':
                self.robot.forward(self.speed)

            elif self.posizione == 'indietro':
                self.robot.backward(self.speed)

            elif self.posizione == 'destra':
                self.robot.right(self.speed)

            elif self.posizione == 'sinistra':
                self.robot.left(self.speed)

            elif self.posizione == 'stop':
                self.robot.stop()


        def speedy(self, msg):
            self.speed = msg.data
            self.controller(self.speed,self.posizione)




        def direzione(self, msg):
            self.posizione = msg.data
            self.controller(self.speed,self.posizione)
