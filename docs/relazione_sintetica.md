# Argo GO2 — Relazione sintetica

**Navigazione autonoma, pianificazione e spiegazione vocale su Unitree Go2 con LLM a bordo**

## 1. Obiettivo

Il progetto rende il quadrupede Unitree Go2 capace di:

- ricevere un comando in linguaggio naturale (es. *"vai a controllare il laboratorio e poi passa dall'openspace"*);
- tradurlo in un piano di azioni;
- eseguirlo navigando in autonomia;
- comunicare a voce cosa sta facendo e, in caso di fallimento, spiegarne il motivo e ripianificare.

Tutto il calcolo avviene a bordo, su un NVIDIA Jetson Orin, senza servizi cloud.

## 2. Architettura

| Livello | Componente | Ruolo |
|---|---|---|
| Comprensione | **Gemma 4 E2B** (llama.cpp, locale) | Interpreta il comando e genera le spiegazioni |
| Pianificazione | **PlanSys2** (PDDL, planner POPF) | Decide la sequenza di azioni |
| Esecuzione | **Nav2** (ROS 2 Humble, in Docker) | Localizza il robot, pianifica il percorso, evita gli ostacoli |
| Percezione | LiDAR 3D Hesai → scansione 2D | Mappa, localizzazione e ostacoli |
| Voce | **Piper** (TTS offline) + **WebRTC** | Fa parlare il robot dal suo altoparlante |

Il sistema principale gira su ROS 2 Foxy. Nav2 gira su ROS 2 Humble in un container Docker e comunica con il resto tramite i topic e le action standard.

## 3. Modello dell'ambiente

- **Mappa:** l'ambiente è stato mappato con *slam_toolbox* usando il LiDAR. Durante la missione il robot si localizza sulla mappa con AMCL, partendo da una posa iniziale data da RViz.
- **Stanze:** tre luoghi (laboratorio di robotica, corridoio, openspace), definiti come waypoint, cioè coordinate (x, y, θ) sulla mappa.
- **Topologia:** nel problema PDDL le stanze sono collegate dal predicato `connected` (laboratorio ↔ corridoio ↔ openspace).
- **Dominio PDDL:** due azioni durative. `navigate` sposta il robot tra stanze collegate. `inspect` è un'ispezione simulata: il robot si inclina in avanti per alcuni secondi.

## 4. Flusso di funzionamento

1. **Comando.** Il testo arriva al nodo `gemma_bridge` attraverso un servizio ROS 2.
2. **Interpretazione.** Gemma estrae le stanze di destinazione in formato JSON. Se il comando cita luoghi sconosciuti, viene rifiutato. Se Gemma non risponde, c'è un fallback a parole chiave.
3. **Pianificazione.** La stanza di partenza si ricava dalla posa AMCL. Le stanze da visitare diventano goal `inspected` e PlanSys2 genera il piano.
4. **Esecuzione.** Ogni azione `navigate` invia a Nav2 un goal `NavigateToPose` con le coordinate della stanza di arrivo.
5. **Gestione dei fallimenti.** Se Nav2 fallisce, Gemma spiega il problema in prima persona. Il collegamento bloccato viene rimosso dal problema e PlanSys2 ripianifica le stanze rimanenti. Se non c'è un percorso alternativo, il robot lo comunica.
6. **Voce.** Ogni passo della missione viene pubblicato su `/supervisor/explanation` e letto ad alta voce dal Go2.

## 5. Sintesi vocale

Il Go2 non ha un TTS che si possa usare dall'esterno. L'audio viene quindi generato a bordo con Piper (voce italiana `it_IT-paola`) e inviato all'altoparlante del robot via WebRTC, lo stesso canale che usa l'app Unitree. La connessione resta aperta per tutta la missione, così ogni frase non richiede un nuovo handshake.

## 6. Limiti attuali

- Le stanze e i loro collegamenti sono definiti a mano.
- L'ispezione è simulata: non c'è un'analisi percettiva.
- Il Go2 accetta un solo client WebRTC alla volta: con la voce attiva, l'app Unitree non può essere collegata.

## 7. Conclusione

Il sistema unisce tre livelli: un LLM per capire e spiegare, un planner simbolico per decidere e uno stack di navigazione per agire. Il risultato è un robot comandabile in linguaggio naturale che rende conto a voce delle proprie azioni e dei propri fallimenti, interamente in locale.
