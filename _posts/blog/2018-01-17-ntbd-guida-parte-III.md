---
title: "NTBD: guida step by step III"
layout: post
date: 2018-01-17
image: /assets/imgs/2018-01-17-ntbd/NTBD-logo-parte3.png
headerImage: true
tag:
 - Robotics
 - NTBD
 - Containers
 - container
 - docker
 - ROS
 - nginx
 - 3D printing
 - 3D
 - stampa 3D
 - manipolatori
 - manipolatore
category: blog
author: fiorellazza
description: "Cos'è e come utilizzare NTBD step by step, terzo articolo della serie"
---
[> Switch to the English version]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guide-part-III %})

[<< Torna a Post Parte I: una panoramica]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-I %})

[<< Torna a Post Parte II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %})

## Parte III: integrazione con altri manipolatori
Per adattare l'archiettura NTBD ad un manipolatore diverso da EEZYBOT, è necessario ridefinire i componenti astratti dell'archiettura.  Inoltre assumiamo che la scheda utilizzata per il controllo dei motori sia una scheda Arduino.
I componenti astratti sono implementati nell'Immagine Docker *ntbd_manipulator* e, come spiegato nel post ["Parte I: una panoramica"]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-I %}), dipendono dal braccio robotico scelto e vanno quindi modificati conformemente alla struttura scelta. 

**Note**: 
- Consiglio la lettura dei post [Parte I]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-I %}) e [Parte II]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %}) per capire cosa modificare per integrare il vostro braccio robotico all'architettura NTBD e come vanno collegati i dispositivi esterni per il controllo e per le WebApp.
- Per poter fare il build delle Immagini Docker è necessario [installare Docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#uninstall-old-versions). In questo tutorial assumo che il lettore abbia delle conoscenze su come sviluppare applicazioni in Docker. Per consigli sulla fase development, leggere l'[appendice](#appendice-docker-prod-vs-docker-devel).

### Indice
1. [Modificare i componenti astratti di NTBD](#modificare-i-componenti-astratti-di-ntbd)
2. [Fare il build della nuova immagine](#fare-il-build-della-nuova-immagine)
3. [Avviare il nuovo contenitore](#avviare-il-nuovo-contenitore)
4. [Appendice: Docker Prod vs Docker Devel](#appendice-docker-prod-vs-docker-devel)

### 1. Modificare i componenti astratti di NTBD
Sarà necessario scaricare il [progetto NTBD da github](https://github.com/HotBlackRobotics/ntbd) in modo tale da modificare i seguenti file prima di fare il build della nuova Immagine *ntbd_manipulator*:

- [*joint_names_sibot_urdf.yaml*](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/config/joint_names_sibot_urdf.yaml): in questo file sono definiti i nomi dei joint del braccio robotico, utili per lo scambio di messaggi ROS. Questa definizione è utile, per esempio, nel nodo [physical_2_visual](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/physical_2_visual), che deve quindi essere modificato di conseguenza.
- [*/meshes/*](https://github.com/HotBlackRobotics/ntbd/tree/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/meshes): questa cartella contiene le [meshes](https://it.wikipedia.org/wiki/Mesh_poligonale) per la visualizzazione del braccio robotico scelto, in formato [*STL*](https://it.wikipedia.org/wiki/STL_(formato_di_file)).
 - [siBOT_noEE.urdf](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf/urdf/siBOT_noEE.urdf): questo file deve contenere la definizione [URDF](http://sdk.rethinkrobotics.com/wiki/URDF) del nuovo manipolatore; può quindi essere rinominato a piacere con l'unico accorgimento di cambiare il nome anche nel launch file [NTBD_launch.launch](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/launch/NTBD_launch.launch).
 - [index.html](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/web/index.html) e [ntbd_lm.html](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/web/ntbd_lm.html) definiscono le applicazioni Web e devono essere modificati a seconda della nuova configurazione (per esempio, con i nuovi limiti).
 - [IK](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/IK), [FK](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/FK), [motor_values](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/motors_values) e [physical_2_visual](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/physical_2_visual): questi file sono tutti dipendenti dalla scelta del braccio robotico; per ulteriori informazioni riguardo al loro ruolo [>>vai al Post Parte I: una panoramica]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-I %}).
- [*NTBD_launch.launch*](https://github.com/HotBlackRobotics/ntbd/blob/devel/NTBD_manipulator/launch/NTBD_launch.launch): questo file deve essere modificato per modificare i parametri ROS dei limiti per le [coordinate della posizione](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/launch/NTBD_launch.launch#L16) e per i [motori](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/launch/NTBD_launch.launch#L24). 
- [myServoControl_ntbd.ino](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/myServoControl_ntbd.ino): ovviamente, cambiando  il manipolatore,  cambia la configurazione fisica del braccio (numero e tipo di motori) e di conseguenza lo sketch caricato sulla scheda Arduino.

**Nota**: 
- il package ROS [*manipulator_urdf*](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/NTBD_abstract_nodes/manipulator_urdf), che contiene tutte le info necessarie per utilizzare la definizione URDF del robot, può essere prodotto automaticamente partendo dall'assembly del robot, utilizzando il tool [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly).
- Nel caso si voglia modificare il nome di un file bisogna tener sempre conto del fatto che questo file potrebbe essere "chiamato" in qualche altro file e quindi quest'ultimo dovrebbe essere modificato di conseguenza. Il consiglio è quindi quello di *evitare di rinominare i file* affinchè non ci siano intoppi.

[**<<Torna all'indice**](#indice)

### 2. Fare il build della nuova immagine
Una volta modificati i file necessari, è ora di "buildare" la nuova immagine. Entrare nella cartella [NTBD_manipulator](https://github.com/HotBlackRobotics/ntbd/tree/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator) in cui è contenuto il file [Dockerfile.intel](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/Dockerfile.intel) ed eseguire il seguente comando.
```
docker build -t ntbd/manipulator:intel .
```

### 3. Avviare il nuovo contenitore
Dopo aver collegato tutti i dispositivi esterni, basterà quindi avviare il contenitore eseguendo:
```
docker-compose -f docker-compose.intel.yml up
```
che sfrutta il tool [Docker Compose](https://docs.docker.com/compose/overview/) con configurazione specificata nel file [docker-compose.intel.yml](https://github.com/HotBlackRobotics/ntbd/blob/06f5af9c35c814ff039fc60e410531724c96a11c/NTBD_manipulator/docker-compose.intel.yml).

L'integrazione di un nuovo braccio finisce qua, per informazioni a proposito dell'utilizzo delle WebApp implementate, vedi il [Post Parte II: un tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %}).

### Appendice: Docker prod vs Docker devel
Quando si lavora con Docker conviene, per motivi di organizzazione e ordine, usare un'Immagine per lo sviluppo (Development Image) ed una per la produzione (Production Image). Quest'ultima è la versione finale dell'applicazioe Docker, pronta per la distribuzione, mentre la prima è usata durante lo sviluppo dell'applicazione. Viene "buildata" un'immagine comune sulla quale si vanno a sviluppare la versione Prod e la versione Dev e, con qualche accorgimento, si può sfruttare al meglio il sistema di caching per la fase di *build* dell'immagine. Infatti, se i file modificati sono ancora in fase di debug, ad ogni build i layer, anche se già buildati in precedenza, vengono ri-buildati.

![docker](/assets/imgs/2018-01-17-ntbd/4_dockerdev.png)

Per la fase di sviluppo è quindi consigliato avere tutti i file non ancora definitivi in una cartella condivisa tra l'host e il container: tramite uno script bash, eseguito all'avvio del contenitore, i file vengono copiati nel container.
Questo, chiaramente, aumenta il tempo di avvio ma evita che l'immagine venga re-buildata ogni volta che un file viene modificato. L'Immagine Prod, che nel caso di ntbd è quella resa [disponibile su Docker Hub](https://hub.docker.com/r/hbrobotics/ntbd_manipulator/), copia i file definitivi dal *building context* (la cartella in cui sono contenute tutte le risorse necessarie all'esecuzione del container) al container, dal momento che tutti file, a questo punto, dovrebbero essere alla loro versione finale.

Di seguito un esempio di building context:

![buildingcontext](/assets/imgs/2018-01-17-ntbd/building-context.png)

Qui invece riporto il contenuto del file bash NTBD_devel_entrypoint.sh usato per la versione devel:
```
#!/usr/bin/env bash
set -e
echo "export TERM=xterm" >> ~/.bashrc
echo ". /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc
# Source form host:
# NTBD_core scripts & launch
cp /src/IK /catkin_ws/src/ntbd_core/scripts/IK
cp /src/physical_2_visual /catkin_ws/src/ntbd_core/scripts/physical_2_visual
cp /src/FK /catkin_ws/src/ntbd_core/scripts/FK
cp /src/motors_values /catkin_ws/src/ntbd_core/scripts/motors_values
# Make scripts executable to be used as nodes!
cd /catkin_ws/src/ntbd_core/scripts/
chmod +x IK && chmod +x physical_2_visual && chmod +x FK && chmod +x motors_values

cp /src/NTBD_launch.launch /catkin_ws/src/ntbd_core/launch/NTBD_launch.launch

# NTBD_urdf
cp -rf /src/manipulator_urdf/ /catkin_ws/src/manipulator_urdf/
# setup ros3djs config (comprehends nginx config)
cp -rf /src/manipulator_urdf/ /var/www/html/manipulator_urdf/
cp /src/NTBD_viz.html /var/www/html/NTBD_viz.html

cp /src/ntbd_lm.html /var/www/html/ntbd_lm.html

cp -rf /src/ros3djs/roswebconsole/ /var/www/html/roswebconsole/ 

source /catkin_ws/devel/setup.bash
cd /catkin_ws/ && catkin_make

# setup ros environment
 source /opt/ros/kinetic/setup.bash
 source /catkin_ws/devel/setup.bash
 
# start nginx
service nginx start

# Launch my ROS nodes and ros3djs URDF visualization
roslaunch ntbd_core NTBD_launch.launch

exec "$@"
```
[**<<Torna all'indice**](#indice)

## FINE PARTE III

[<< Torna a Post Parte I: una panoramica]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-I %})

[<< Torna a Post Parte II: tutorial]({{ site.baseurl }}{% post_url /_posts/blog/2018-01-17-ntbd-guida-parte-II %})
