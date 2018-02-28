---
title: "Ecco il vademecum di robotica per le Piccole Medie Imprese"
layout: post
date: 2018-02-26
image: https://www.ennomotive.com/wp-content/uploads/2016/07/agency-spring_universal-robots_cvi_3-1.jpg
lang: it
tag:
 - ROS

author: sgabello
description: "Ecco il vademecum di robotica per le Piccole Medie Imprese"
---

Che cosa significa veramente Industria 4.0 e cosa vuol dire in concreto innovare con la robotica in una Piccola Media Impresa? Questo breve articolo vorrei che spiegasse in modo semplice, ma dettagliato, come sta cambiando il mondo della robotica e cosa stiamo facendo come HotBlack Robotics per portare la robotica nelle (piccole e medie) aziende italiane.

## Una nuova robotica disegnata ad hoc per le PMI: la robotica collaborativa ##

Una volta la robotica che si vedeva in azienda era principalmente "robotica industriale" o meglio tradizionale, ovvero quel tipo di robotica che generalmente vive nella grande azienda e automatizza operazioni sempre uguali e noiose per gli operatori umani. Questo tipo di automazione richiede un investimento iniziale abbastanza importante che verrà però ammortizzato grazie al notevole aumento di produttività.

Ovviamente lo stesso tipo di ragionamento non si può fare per una PMI, in realtà non solo per il discorso economico, ma soprattutto per le necessità produttive!
Infatti la grande azienda ragiona sul concetto di [produzione di massa](https://it.wikipedia.org/wiki/Produzione_di_massa), dove le linee produttive sono molto grandi e realizzano grandi quantità di prodotti standardizzati. Di conseguenza più si riesce ad automatizzare il processo produttivo *standardizzato* più i costi diminuiranno.

Nelle PMI o nell'artigianato invece si lavora molto spesso su piccole serie produttive e non standardizzate (ad esempio "produzione su commessa"). Questo concetto ovviamente manda in crisi la robotica tradizionale e richiede un tipo di robotica più dinamica e con costi inferiori: **la robotica collaborativa**.

![saldatura robotica](https://blog.robotiq.com/hs-fs/hub/13401/file-1570131806-jpg/images/collaborative-robot-welding-resized-600.jpg?t=1519765326534)

La **robotica collaborativa** è l'ultima frontiera della robotica e ha un funzionamento completamente diverso dalla robotica industriale. Innanzitutto la cosa curiosa è che si chiama proprio "collaborativa" perchè il **robot deve collaborare con l'uomo** come strumento per migliorare il sistema produttivo. In generale il robot collaborativo ha prestazioni inferiori rispetto a quello industriale ma ancora in linea con le necessità di una PMI.

Vediamo le principali differenze:

|**robot collaborativo**|**robot industriale**|
|:---:|:---:|
|non può portare carichi elevati in genere 3-5 Kg| può portare carichi elevati anche di 200 Kg|
|il braccio robotico non è velocissimo (in genere 1 m/s)| il braccio robotico può arrivare tranquillamente a 3m/s|
|costi totali contenuti (intorno ai 20 mila Euro) | costi più elevati |
|programmazione semplificata e dinamica| programmazione più complessa e per niente flessibile |
|lavora con l'uomo in sicurezza | lavora in una gabbia di protezione|

Ok ora prendiamo in esame due lavorazioni molto usate nella PMI e vediamo come affrontarle con la robotica collaborativa.

## Saldatura ##

La differenza sostanziale dalla robotica tradizionale è che nelle grandi linee di saldatura c'è bisogno di una "maschera", ovvero una guida metallica che la testa robotica deve seguire per mantenere correttamente la posizione nello spazio della punta saldante. La programmazione del robot viene eseguita una sola volta da un consulente esperto e non è per niente flessibile. Per una PMI questo discorso è assurdo visto che le saldature non sono sempre esattamente uguali e bisogna costruire ogni volta una nuova maschera per la saldatura, chiamare nuovamente il consulente per riscrivere il software e tutto ciò fa ovviamente incrementare notevolmente il costo finale e rende il tutto inefficiente.

Con la robotica collaborativa invece basterà muovere a mano la testa saldante per registrare il percorso che il robot andrà a fare ed ogni volta che si vuole cambiare tipo di saldatura basterà ripetere il processo in modo veramente semplice! Inoltre non c'è bisogno della gabbia e di nessuna barriera protettiva perchè in caso di urto il robot si ferma automaticamente. Ovviamente se si vuole proteggere dalla testa saldante basta aggiungere un sensore che rileva la presenza dell'uomo e mette in pausa il processo.



<iframe width="560" height="315" src="https://www.youtube.com/embed/tGk2LQ5hDNE" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


<iframe width="560" height="315" src="https://www.youtube.com/embed/Ik6FjIXn8RI" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


Maggiori informazioni su questo [blog](https://blog.robotiq.com/bid/72421/Collaborative-Robots-for-Welding).

## Pick and place ##

Discorso analogo si può fare per operazioni noiose e ad oggi svolte in modo manuale per la movimentazione di materiali all'interno della catena produttiva. Un esempio di pick and place si può trovare qui.

<iframe width="560" height="315" src="https://www.youtube.com/embed/pIcxOGo7ieU" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

In questo caso, oltre a quello già anticipato, il robot collaborativo può essere ancora più dinamico aggiungendo anche una camera per la visione artificiale per individuare in modo automatico i pezzi da prendere. In questo caso il il vantaggio sta nell'evitare di stabilire a priori la posizione precisa dei pezzi nello spazio da reperire e sarà la visione artificiale ad individuarli in modo automatico.

<iframe width="560" height="315" src="https://www.youtube.com/embed/j2IPCZthTCw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Qui ci sono un po' di esempi sul sito [Universal Robots](https://www.universal-robots.com/applications/pick-and-place/) nella sezione "pick&place".




# Costi ##

Come penso abbiate intuito mi piace molto la robotica di Universal Robots, non ho nessun vantaggio a consigliarla ma effettivamente è secondo me una delle migliori marche per i robot collaborativi. In realtà non credo di essere il solo a pensarla così visto che negli ultimi anni Universal Robots è cresciuta in modo esponenziale grazie ai suoi incredibili prodotti. In ogni caso ovviamente anche KUKA e ABB hanno robot collaborativi e stanno lavorando più o meno verso la stessa direzione ma per ora i costi sono più alti e per lo più utilizzati a scopo di ricerca. Dovessi quindi impostare una semplice linea di saldatura o pick and place con le caratteristiche accennate sopra credo sceglierei un robot Universal Robots tra [questi](https://www.universal-robots.com/products/) che compaiono sul sito.

La vera innovazione comunque per me rimane il confronto di prezzo tra la saldatura robotica in modo tradizionale che, facendo un preventivo molto grossolano, si aggira intorno ai 100 mila Euro mentre con la robotica collaborativa sui 30/50. Basta comunque tener presente che un robot Universal Robots si aggira sui 20 mila Euro.

Ci sono anche altre soluzioni un po' più hobbistiche con prezzi decisamente inferiori.
[EVA automata](https://automata.tech/) è un braccio molto bello, anche se non è veramente collaborativo, ha buone specifiche techniche e costa 5000 Euro!

## HotBlack Robotics ##

Come spiegato da Dario D'elia di Tom's Hardware, da cui ho avuto il piacere di essere intervistato, in HotBlack Robotics vogliamo portare la robotica in Italia e nelle Piccole medie imprese tramite una community di consulenti esperti di robotica.

<iframe width="560" height="315" src="https://www.youtube.com/embed/TX4_QiOkArY" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>    

 Se siete interessati a portare la robotica all'interno della vostra azienda o ricevere maggiori informazioni vi prego di scrivermi!
 <a class="btn btn-lg btn-secondary btn-block" href="mailto:info@hotblackrobotics.com">Manda una e-mail</a>

**Ciao!** :robot:
