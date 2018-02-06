---
title: "Installare nuove librerie sul HBRain (temporaneamente)"
layout: post
date: 2017-03-16 17:04:59
image:
headerImage: false
lang: it
tag:
 - Tutorial
 - Raspberry
 - Hbrain

redirect_from: /blog/posts/2017-03-16-installare-nuove-librerie-sul-hbrain-temporaneamente
author: ludusrusso
description: ""
---

Un breve post per informarci su come installare nuove librerie Python sul robot utilizzabili dalla nostra interfaccia grafica.

<strong>A breve sarà possibile installare nuove librerie direttamente dalla webapp</strong>

## Accedere al Raspberry via SSH

Per prima cosa, è necessario accedere via SSH al Raspberry su cui si vuole installare la nuova libreria.

Per farlo, da mac o linux si può semplicemente eseguire la seguente linea di comando dal terminale

```
ssh root@<ip robot>
```
esempio:

```
ssh root@192.168.0.3
```

ed inserire la password `raspberry`

Da Windows, si può usare un client come [putty](http://www.putty.org/) inserendo come nome utente `root` e come password `raspberry`.

L'indirizzo IP del raspberry lo trovate sulla piattaforma


## Accedere al Virtualenv

Una volta entrati, eseguite il seguente codice per accedere al virtualenv della piattaforma:

```
workon ros
```

Vedrete apparire la stringa `(ros)` all'inizio della shell.

## Installare il pacchetto Python

Per installare un pacchetto, eseguite la linea di codice

```
pip install <nome pacchetto>
```
esempio

```
pip install telepot
```

Oppure eseguite i comandi come da documentazione del pacchetto richiesto.
