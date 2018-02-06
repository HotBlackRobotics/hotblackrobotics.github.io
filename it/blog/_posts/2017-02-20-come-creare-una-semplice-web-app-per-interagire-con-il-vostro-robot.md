---
title: "Come creare una semplice Web App per interagire con il vostro robot"
layout: post
date: 2017-02-20 18:03:13
image:
headerImage: false
tag:

redirect_from: 
 - /2017/02/20/come-creare-una-semplice-web-app-per-interagire-con-il-vostro-robot/
 - /blog/posts/2017-02-20-come-creare-una-semplice-web-app-per-interagire-con-il-vostro-robot
author: sgabello
description: ""
lang: it
---

Ecco un semplice tutorial per spiegare un po' meglio come funzionano le Web App. Innanzitutto con il termine Web App intendiamo le applicazioni web che potete trovare nella tendina "Apps" in piattaforma. Queste Web App sono fatte da noi come esempio, ma ovviamente il codice è open source e in questo tutorial vi spiegheremo come potrete scrivere le vostre Web App. Attenzione però le Web App che andrete a scrivere (per motivi di sicurezza) saranno utilizzabili solo in locale sul vostro computer!

![] (http://res.cloudinary.com/www-hotblackrobotics-com/image/upload/v1487612501/Schermata_2017-02-20_alle_18.36.52_txfrza.png)    

Iniziamo a vedere il codice per una Web App di un publisher e subscriber in ROS. Aprite un documento di testo sul vostro pc e chiamatelo "pubsub.html".  Poi copiate e incollate il seguente codice.

```
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<script type="text/javascript" src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
<script type="text/javascript" src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
<script type="text/javascript" src="http://www.hotblackrobotics.com/cloud/webgui/static/js/initros.js"></script>
<script type="text/javascript">

start_ros('192.168.0.104', 'hotbot', '192.168.0.104:9090', 'None');


</script>
<script type="text/javascript" type="text/javascript">

  // Publishing a Topic
  // ------------------

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/' + robot.name  + '/command_velocity',
    messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message({
    linear : {
      x :100,
      y : 100,
      z : 0
    },
    angular : {
      x : 0,
      y : 0,
      z : 0
    }
  });
  cmdVel.publish(twist);

  // Subscribing to a Topic
  // ----------------------

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/' + robot.name  + '/listener',
    messageType : 'std_msgs/String'
  });

  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
  });

</script>
</head>

<body>
  <h1>Controlla ROS Console</h1>
  <p>Sto pubblicando un messaggio su '/< nome_del_tuo_robot >/command_velocity'</p>
  <p>E ascoltando su '/< nome_del_tuo_robot >//listener' un messaggio (guarda nella console del tuo browser) </p>

</body>
</html>
```

Dovete modificare l'indirizzo IP del robot quando usate questa funzione ```start_ros('192.168.0.104', 'hotbot', '192.168.0.104:9090', 'None');
``` .
Una volta connesso, nel mio caso ho ricevuto l'indirizzo IP 192.168.0.104 e il robot si chiama "hotbot". Poi andiamo a dichirare un publisher, che pubbblica sul topic relativo al nome del vostro robot ```/<nome del vostro robot>/command_velocity``` un tipo di messaggio geometry_msgs/Twist.

```
 var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/' + robot.name  + '/command_velocity',
    messageType : 'geometry_msgs/Twist'
  });
```
Riempiamo il messaggio così

```
var twist = new ROSLIB.Message({
    linear : {
      x :100,
      y : 100,
      z : 0
    },
    angular : {
      x : 0,
      y : 0,
      z : 0
    }
  });
```

 e lo pubblichiamo

 ```
  cmdVel.publish(twist);
 ```
 Per quanto riguara il subscriber, lo dichiariamo che rimanga in ascolto sul topic ``` '/<nome del vostro robot>/listener' ``` (messaggio std_msgs/String).
 E la funzione di call back:

 ```
  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
  });
 ```
 Vederete il risultato della funzione call back sul browser premendo col tasto destro del mouse e andando su console.
 Facile no? ;)
