# Argo GO2 — Relazione tecnica

**Navigazione autonoma, pianificazione simbolica e spiegazione vocale su Unitree Go2 con LLM a bordo**

---

## Indice

1. Introduzione e obiettivi
2. Piattaforma hardware e software
3. Architettura generale
4. Modello dell'ambiente
5. Comprensione del linguaggio naturale con Gemma
6. Pianificazione con PlanSys2
7. Esecuzione con Nav2
8. Gestione dei fallimenti e ripianificazione
9. Comunicazione vocale
10. Avvio e orchestrazione del sistema
11. Limiti e sviluppi futuri
12. Conclusioni

---

## 1. Introduzione e obiettivi

L'obiettivo del progetto è un robot di servizio che si possa comandare con il linguaggio di tutti i giorni e che sappia **rendere conto delle proprie azioni**. Il quadrupede Unitree Go2 riceve un comando come *"vai a controllare il laboratorio e poi passa dall'openspace"*, lo trasforma in un piano, lo esegue navigando nell'edificio e racconta a voce cosa sta facendo. Se la navigazione fallisce, il robot spiega il problema e cerca un percorso alternativo.

Il progetto segue tre principi:

- **Separare le responsabilità.** Il modello linguistico non controlla direttamente il moto. Si limita a interpretare e a spiegare. Le decisioni sulla sequenza di azioni spettano a un planner simbolico, che è verificabile, e l'esecuzione a uno stack di navigazione consolidato.
- **Calcolo a bordo.** Tutti i componenti, LLM e sintesi vocale compresi, girano sul calcolatore del robot. Non serve una connessione internet.
- **Spiegabilità.** Ogni passo della missione e ogni fallimento viene comunicato all'operatore in italiano, sia come testo sia a voce.

## 2. Piattaforma hardware e software

**Hardware**

- Unitree Go2, quadrupede, con altoparlante integrato.
- NVIDIA Jetson Orin come calcolatore di bordo.
- LiDAR 3D Hesai, per mappatura, localizzazione e rilevamento ostacoli.
- Camera Intel RealSense, opzionale, usata solo per il rilevamento di oggetti con YOLO (disattivato di default).

**Software**

- ROS 2 Foxy su Ubuntu 20.04: driver, sensori, PlanSys2, nodi applicativi.
- ROS 2 Humble in un container Docker: stack Nav2.
- llama.cpp per eseguire Gemma 4 E2B (quantizzazione Q8_0, accelerazione GPU) con un'API compatibile OpenAI.
- Piper per la sintesi vocale neurale offline.
- La libreria open source `go2_webrtc_connect` per la comunicazione WebRTC con il Go2.

Nav2 gira su Humble invece che su Foxy perché la versione Humble è più matura e più stabile. Il container condivide la rete DDS con l'host: i due ambienti comunicano in modo trasparente attraverso i topic e le action standard (`/tf`, `/scan`, `/cmd_vel`, `/navigate_to_pose`).

## 3. Architettura generale

```
   Comando in linguaggio naturale
                │  servizio /gemma_interface/send_command
                ▼
   ┌────────────────────────┐   HTTP    ┌──────────────────────────┐
   │  gemma_bridge (Python) │ ◄───────► │ Gemma 4 E2B (llama.cpp)  │
   └───────────┬────────────┘           └──────────────────────────┘
               │ goal PDDL / execute_plan
               ▼
   ┌────────────────────────┐
   │  PlanSys2 (POPF)       │  dominio: navigate, inspect
   └───────────┬────────────┘
               │ azioni
               ▼
   ┌────────────────────────┐  NavigateToPose  ┌──────────────────────┐
   │  Esecutori C++         │ ───────────────► │ Nav2 Humble (Docker) │
   │  navigate / inspect    │                  │ AMCL, planner, ctrl  │
   └───────────┬────────────┘                  └──────────┬───────────┘
               │ /euler (inspect)                         │ /cmd_vel
               ▼                                          ▼
   ┌──────────────────────────────────────────────────────────────────┐
   │                    Driver Go2 + LiDAR Hesai                      │
   └──────────────────────────────────────────────────────────────────┘

   gemma_bridge ──► /supervisor/explanation ──► argo_voice_node ──► Piper ──► WebRTC ──► altoparlante Go2
```

I componenti principali sono:

| Componente | Pacchetto | Funzione |
|---|---|---|
| `gemma_bridge.py` | `go2_patrol_mission` | Riceve i comandi, interroga Gemma, configura PlanSys2, gestisce i fallimenti |
| `patrol_domain.pddl` | `go2_patrol_mission` | Dominio di pianificazione |
| `navigate_action_node` | `go2_patrol_mission` | Esegue `navigate` inviando goal a Nav2 |
| `inspect_action_node` | `go2_patrol_mission` | Esegue `inspect` inclinando il corpo del robot |
| `argo_voice_node.py` | `go2_hardware` | Riceve i messaggi della missione e li passa al worker vocale |
| `go2_speaker_worker.py` | `go2_hardware` | Sintesi Piper e invio audio via WebRTC |
| `go2_bringup` | `go2_bringup` | Driver Go2, LiDAR e conversione della nuvola di punti in scansione 2D |

## 4. Modello dell'ambiente

### 4.1 Mappatura

L'ambiente è stato mappato con **slam_toolbox** in modalità sincrona. La nuvola di punti del LiDAR Hesai passa prima da un filtro (`pointcloud_filter.py`), che elimina i punti che cadono sul corpo del robot. Poi viene convertita in una scansione laser 2D con `pointcloud_to_laserscan`. Il risultato è una mappa di occupazione (`map_1779889723.yaml`), che Nav2 carica all'avvio.

### 4.2 Localizzazione

Durante la missione il robot si localizza con **AMCL**, un filtro particellare, confrontando la scansione 2D con la mappa. All'avvio l'operatore dà la posa iniziale da RViz con la *2D Pose Estimate*.

### 4.3 Stanze e topologia

Le stanze sono un livello semantico definito a mano sopra la mappa metrica. Ogni stanza è un waypoint (`config/waypoints.yaml`):

| ID | Nome | x [m] | y [m] | θ [rad] |
|---|---|---|---|---|
| `lab_robotics` | Laboratorio di robotica | 2.5 | 0.5 | 0.0 |
| `corridor` | Corridoio | 11.0 | 3.2 | 0.0 |
| `openspace` | Openspace | 4.0 | 2.0 | 1.57 |

La topologia è espressa nel problema PDDL con il predicato `connected`. Il corridoio fa da snodo tra le altre due stanze:

```
lab_robotics  ⟷  corridor  ⟷  openspace
```

Ogni stanza ha anche una descrizione e un elenco di sinonimi ("lab", "laboratorio", "hallway", "open space", …). Le descrizioni servono a Gemma per riconoscerla, i sinonimi a normalizzare le risposte del modello.

## 5. Comprensione del linguaggio naturale con Gemma

### 5.1 Modello

Il modello è **Gemma 4 E2B** (Google), in versione *instruction-tuned* e quantizzato a 8 bit. È servito da `llama-server` (llama.cpp) sulla porta 8000, con tutti i layer su GPU e una finestra di contesto di 4096 token. È abbastanza piccolo da girare in tempo reale sul Jetson Orin insieme al resto dello stack.

### 5.2 Interpretazione del comando

Quando arriva un comando, `gemma_bridge` costruisce un prompt di sistema che:

- elenca le uniche stanze note, con le loro descrizioni;
- chiede di estrarre le stanze nell'ordine richiesto, usando solo gli ID ammessi;
- chiede di segnalare i luoghi sconosciuti e di rifiutare le richieste che non riguardano la navigazione;
- impone una risposta in JSON con questo schema:

```json
{"accepted": true, "target_rooms": ["lab_robotics", "openspace"], "unknown_places": [], "reply": "..."}
```

### 5.3 Validazione della risposta

L'output del modello non viene usato così com'è. Prima passa da diversi controlli deterministici:

- ogni stanza viene normalizzata con la tabella dei sinonimi, e quelle non riconosciute diventano "luoghi sconosciuti";
- vengono scartati i falsi positivi, per esempio quando il modello riporta l'intera frase come luogo;
- se tutte le stanze estratte sono valide, il comando viene accettato anche se il modello ha impostato per errore `accepted: false`;
- se Gemma non risponde o restituisce un JSON non valido, si passa a un **fallback a parole chiave** sugli ID delle stanze.

Se il comando viene rifiutato, il sistema compone da sé un messaggio in italiano che indica i luoghi conosciuti. Il messaggio non è generato da Gemma, per avere sempre una risposta corretta.

## 6. Pianificazione con PlanSys2

### 6.1 Dominio

Il dominio `patrol_domain.pddl` usa i tipi `robot` e `room` e i predicati `robot_at`, `connected` e `inspected`. Contiene due azioni durative:

- **`navigate ?r ?from ?to`** (10 s): richiede che il robot sia in `?from` e che `?from` sia collegata a `?to`; al termine il robot si trova in `?to`.
- **`inspect ?r ?rm`** (5 s): richiede che il robot resti in `?rm` per tutta la durata; al termine `?rm` risulta ispezionata.

### 6.2 Costruzione del problema

A ogni comando il bridge:

1. svuota la knowledge base di PlanSys2;
2. inserisce le istanze (il robot `go2` e le tre stanze) e i collegamenti;
3. ricava la stanza di partenza dalla posa AMCL, scegliendo il waypoint più vicino entro 1.5 m. Se il robot non è in nessuna stanza, crea un nodo `start_pos` collegato alle destinazioni;
4. imposta come goal `inspected` su ogni stanza richiesta (in congiunzione, se sono più di una);
5. chiede il piano al solver **POPF** e lo invia all'executor di PlanSys2 (`/execute_plan`).

Se un nuovo comando arriva mentre è in corso un altro piano, il piano precedente e i goal Nav2 attivi vengono annullati prima di ripartire.

### 6.3 Esecuzione delle azioni

- **`navigate_action_node`** (C++) legge le coordinate della stanza di arrivo da `waypoints.yaml` e invia un goal `NavigateToPose` a Nav2. Se il goal viene rifiutato, interrotto o annullato, pubblica un evento `failure:navigate:<robot>:<da>:<a>` e segnala il fallimento a PlanSys2.
- **`inspect_action_node`** (C++) chiama il servizio `/euler` del driver per inclinare il corpo in avanti (pitch 0.3 rad), attende 3 secondi e ripristina la posizione. L'ispezione è quindi simulata con un gesto visibile.

Il bridge riceve il feedback dell'executor e trasforma ogni cambio di stato in una frase per l'operatore, per esempio *"Mi sto muovendo da laboratorio verso corridoio"*, *"Ispezione di openspace completata"*, *"Missione completata"*.

## 7. Esecuzione con Nav2

Nav2 (ROS 2 Humble) gira nel container Docker con la configurazione `nav2_params_gio.yaml` e la mappa sopra descritta. Comprende:

- **AMCL** per la localizzazione;
- il **planner globale** per il percorso sulla mappa;
- il **controller locale** per seguire il percorso evitando gli ostacoli rilevati dal LiDAR;
- il **BT Navigator** per coordinare l'esecuzione e i comportamenti di recupero.

Il controller pubblica i comandi di velocità su `/cmd_vel`, che il driver del Go2 traduce in comandi di locomozione.

## 8. Gestione dei fallimenti e ripianificazione

Quando un'azione `navigate` fallisce, il bridge riceve l'evento di fallimento e avvia la procedura di recupero in un thread separato, per non bloccare il resto del sistema:

1. **Spiegazione.** Gemma riceve il contesto (stanza di partenza e di arrivo) e genera una spiegazione breve, in prima persona, che viene pubblicata e letta ad alta voce. Se il modello non risponde, si usa una frase predefinita.
2. **Aggiornamento del modello del mondo.** Il collegamento tra le due stanze viene rimosso dal problema, in entrambe le direzioni, e la posizione del robot viene aggiornata.
3. **Ripianificazione.** Le stanze non ancora ispezionate diventano il nuovo goal e PlanSys2 cerca un piano alternativo.
4. **Esito.** Se il piano esiste, viene eseguito. Altrimenti Gemma genera una spiegazione del fatto che il robot non può raggiungere le stanze rimanenti.

In questo modo un problema fisico (un passaggio bloccato) diventa un cambiamento della conoscenza simbolica (un collegamento rimosso), che il planner sa gestire.

## 9. Comunicazione vocale

### 9.1 Perché una pipeline dedicata

Il Go2 non ha un sintetizzatore vocale utilizzabile dall'esterno: il suo altoparlante riproduce solo l'audio che riceve. L'unico canale disponibile per inviarglielo è **WebRTC**, lo stesso usato dall'app Unitree. La voce è quindi generata a bordo e trasmessa al robot.

### 9.2 Struttura

La pipeline usa due processi separati:

- **`argo_voice_node.py`** (nodo ROS 2) è iscritto a `/supervisor/explanation`. Prepara le frasi: elimina gli ID tecnici, accorcia i messaggi di rifiuto e unisce le frasi molto ravvicinate. Poi le passa al worker attraverso lo standard input.
- **`go2_speaker_worker.py`** (processo senza ROS, in un virtualenv dedicato):
  1. sintetizza la frase con **Piper**, usando la voce italiana `it_IT-paola-medium`, in PCM a 48 kHz mono;
  2. aggiunge una breve pausa iniziale, perché lo speaker del Go2 impiega circa un secondo ad attivarsi;
  3. invia l'audio come traccia WebRTC in frame da 20 ms.

La separazione in due processi è una necessità tecnica: il middleware DDS di ROS 2 e la libreria WebRTC usano versioni incompatibili di OpenSSL, e nello stesso processo l'handshake WebRTC non si completa. Inoltre il worker tiene **aperta la connessione** per tutta la sessione: durante la navigazione, con la CPU sotto carico, un nuovo handshake per ogni frase spesso non riuscirebbe.

Piper è vincolato a due core e a priorità ridotta, così la sintesi non sottrae risorse a Nav2 e ai driver.

## 10. Avvio e orchestrazione del sistema

L'intero sistema si avvia con `scripts/start_pipeline.sh`, che apre una sessione tmux con una finestra per ogni componente. I componenti partono in ordine di dipendenza, e ciascuno aspetta che i precedenti siano pronti:

1. server Gemma (llama.cpp);
2. bringup del robot: driver Go2, LiDAR e conversione in scansione 2D;
3. Nav2: il container viene avviato quando sono disponibili la TF `odom → base_link` e i dati del LiDAR;
4. PlanSys2, esecutori delle azioni e `gemma_bridge`, quando Nav2 e Gemma sono attivi;
5. visualizzazione delle spiegazioni e nodo vocale;
6. un terminale per inviare i comandi.

Alcune opzioni si attivano con variabili d'ambiente (YOLO, RViz, voce, versione di Nav2). `scripts/stop_pipeline.sh` arresta tutto in modo ordinato.

## 11. Limiti e sviluppi futuri

| Limite | Possibile sviluppo |
|---|---|
| Stanze, waypoint e collegamenti definiti a mano | Segmentazione semantica automatica della mappa |
| L'ispezione è un gesto simulato | Ispezione percettiva con camera e rilevamento oggetti (YOLO è già integrabile) |
| Comandi solo testuali | Riconoscimento vocale a bordo (es. Whisper) per un'interazione completamente vocale |
| La spiegazione dei fallimenti usa solo il nome delle stanze | Aggiungere al contesto di Gemma gli ostacoli rilevati e lo stato di Nav2 |
| Un collegamento rimosso resta escluso per tutta la missione | Riprovare il passaggio dopo un certo tempo |
| Un solo client WebRTC alla volta: con la voce attiva l'app Unitree non può collegarsi | Vincolo del robot, da gestire a livello operativo |

## 12. Conclusioni

Il sistema dimostra che su un robot reale, e interamente a bordo, si possono integrare tre livelli complementari:

- un **LLM leggero** (Gemma 4 E2B), che interpreta comandi in linguaggio naturale e rende comprensibili all'operatore le situazioni di errore;
- un **planner simbolico** (PlanSys2/PDDL), che garantisce piani coerenti con un modello esplicito dell'ambiente e permette di ripianificare quando il mondo cambia;
- uno **stack di navigazione** consolidato (Nav2), che si occupa in sicurezza dell'esecuzione fisica.

L'LLM non agisce direttamente sul robot: le sue uscite vengono validate e passano dal planner. Questa scelta tiene insieme la flessibilità del linguaggio naturale e l'affidabilità di un sistema deterministico. La pipeline vocale, infine, chiude il ciclo di interazione: il robot non si limita a eseguire, ma comunica cosa fa e perché.
