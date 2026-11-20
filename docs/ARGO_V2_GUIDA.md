# Argo v2 — Guida ai codici e alla personalizzazione

Questa guida spiega i codici scritti per Argo v2, come lavorano insieme e **dove mettere le mani**
per cambiare luoghi, frasi, comportamento di Gemma e missioni.
La specifica completa resta in `README_instruction.md`; le cose ancora da fare in `TODO.md`.

---

## 1. Il quadro generale

```
 argo_cmd "di' a Vito che…"
        │  (servizio /argo/command, GemmaCommand.srv)
        ▼
 ┌──────────────────────────┐   G1 FormulateGoals   ┌──────────────────┐
 │ argo_mission_bt          │ ────────────────────▶ │ gemma_server.py  │──▶ llama.cpp (Gemma, porta 8000)
 │ (behavior tree, C++)     │ ◀──────────────────── │ (lingua, scelte) │
 │                          │   G3 AssessFailure    └──────────────────┘
 │  SetGoals → SyncKB →     │
 │  ComputePlan → Execute   │
 └───────────┬──────────────┘
             │ fatti + goal          piano (POPF)
             ▼                       ▲
 ┌──────────────────────────┐        │
 │ PlanSys2 (dominio v2)    │────────┘
 │ problem expert / planner │
 │ executor                 │── un'azione alla volta ──▶ executor delle azioni
 └──────────────────────────┘                            (navigate, deliver_message,
                                                          announce, chat, advance, …)
                                                                 │
                         Nav2 (Docker, Humble) ◀── NavigateToPose ┤
                         argo_voice_node      ◀── /argo/say  ─────┘  ──▶ speaker del Go2
```

Chi fa cosa (regola del progetto, vedi `CLAUDE.md`):

| Componente | Ruolo | Non fa mai |
|---|---|---|
| **Gemma** (`gemma_server.py`) | capisce la richiesta, sceglie da insiemi chiusi, scrive le frasi | non produce piani né azioni |
| **PlanSys2 + POPF** | decide l'ordine delle visite, l'energia, la zona caffè per ultima, il permesso | — |
| **Behavior tree** (`argo_mission_bt`) | flusso della missione, interruzioni, recupero dai guasti | — |
| **Executor** | una azione PDDL → cose concrete sul robot (Nav2, voce) | non decidono la missione |

### Il percorso di un comando, passo per passo

1. `argo_cmd "di' a Vito che la riunione è alle 15"` chiama `/argo/command`.
2. Il BT (`WaitCommand`) prende il comando e chiede a Gemma **G1** cosa significa:
   `mission=message`, `targets=[vito_room]`, `message_text="la riunione è alle 15"`.
3. Argo dice la risposta ("Ricevuto: porto il messaggio a Vito.").
4. `SetGoals` scrive nel KB di PlanSys2 i fatti (`has_message vito_door`, `unlocked vito_door`) e il
   goal `(and (message_delivered vito_door) (robot_at go2 lab_robotics))`.
5. `SyncKB` mette la posizione del robot (nodo più vicino alla posa AMCL) e la batteria.
6. `ComputePlan`: POPF calcola il piano (lab → hub_west → vito_door → consegna → ritorno).
7. `ExecutePlan`: PlanSys2 esegue le azioni una alla volta, ognuna con il suo executor.
8. Se qualcosa fallisce (es. porta bloccata), il BT chiede a Gemma **G3** una strategia
   (aspetta e riprova / salta le stanze irraggiungibili / torna a casa), la dice e ripianifica.
9. Alla fine `MissionReport` dice il resoconto e `ClearMission` pulisce il KB.

---

## 2. Mappa dei file

### Configurazione e modello
| File | Cosa contiene |
|---|---|
| `src/go2_patrol_mission/config/ontology.yaml` | **la fonte unica**: luoghi, porte, hub, pose, occupanti, sinonimi, archi, classi |
| `src/go2_patrol_mission/pddl/patrol_domain_v2.pddl` | dominio PDDL v2 (azioni e regole del planning) |
| `src/go2_patrol_mission/pddl/problems/*.yaml/.pddl` | i problemi di test P1–P10 (+ P1b, P8b, P10b) |
| `src/go2_patrol_mission/pddl/results/` | piani e `planning_results.csv` della suite |
| `config/waypoints.yaml` | **solo v1**, non toccare se vuoi che la v1 funzioni |

### Programmi
| File | Cosa fa |
|---|---|
| `src/go2_patrol_mission/scripts/gemma_server.py` | i 5 servizi di Gemma (G1, G3, G4, G5, G6) con validazione e fallback |
| `src/argo_mission_bt/` | il behavior tree della missione (pacchetto nuovo, C++) |
| `src/argo_mission_bt/behavior_trees/mission.xml` | **l'albero**: modificabile senza ricompilare il C++ |
| `src/go2_patrol_mission/src/navigate_action_node.cpp` | executor `navigate` (Nav2, sblocco, attesa voce, limite di tempo) |
| `src/go2_patrol_mission/src/deliver_message_action_node.cpp` | executor `deliver_message` |
| `src/go2_patrol_mission/src/announce_action_node.cpp` | executor `announce` (chiamata caffè) |
| `src/go2_patrol_mission/src/chat_action_node.cpp` | executor `chat` (barzellette alla macchinetta) |
| `src/go2_patrol_mission/src/advance_action_node.cpp` | executor `advance` (ordine "prima… poi…") |
| `src/go2_patrol_mission/src/inspect_action_node.cpp` | ispezione v1 (inclinazione + sblocco); la versione v2 con YOLO è da fare |
| `src/go2_patrol_mission/src/fake_action_executors.cpp` | executor finti per le prove senza robot |
| `src/go2_patrol_mission/include/go2_patrol_mission/speaking_action.hpp` | base comune degli executor che parlano |
| `src/go2_patrol_mission/scripts/waypoint_markers.py` | marker RViz su `/argo_waypoints` (stanze rosse, hub blu, archi grigi) |
| `src/go2_interfaces/srv/` | `FormulateGoals`, `AssessFailure`, `AppraiseFindings`, `Speak`, `Chat` |

### Launch e script
| File | Cosa fa |
|---|---|
| `src/go2_patrol_mission/launch/argo_mission_launch.py` | **lo stack v2**: PlanSys2 + executor + gemma_server + marker + BT |
| `src/go2_patrol_mission/test/plansys2_v2_check.launch.py` | PlanSys2 + executor veri/finti, senza BT (prove) |
| `scripts/start_pipeline.sh` | `ARGO_VERSION=v2` avvia la v2 nella finestra `patrol` |
| `scripts/argo_env.sh` | `argo_cmd` (usa `/argo/command` se c'è, altrimenti la v1), `wait_argo` |

### Strumenti (`tools/`)
| Script | A cosa serve |
|---|---|
| `record_pose.py` | salva pose con nome dalla TF `map → base_link` |
| `check_ontology.py` | controlla l'ontologia (albero, porte, classi, ID) |
| `compute_distances.py` | calcola le distanze degli archi sulla mappa e le scrive nell'ontologia |
| `gen_problem.py` | genera un problem PDDL (o i comandi per `plansys2_terminal` con `--terminal`) |
| `optimal_tours.py` | distanza minima teorica di una missione (per giudicare i piani) |
| `run_planning_tests.sh` | lancia POPF su tutti i problemi e scrive il CSV |
| `plansys2_check.py` | carica un problem in PlanSys2, pianifica ed esegue |
| `run_action.py` | esegue una o più azioni singole tramite PlanSys2 |
| `set_message.py` | pubblica a mano i messaggi su `/argo/messages` (senza BT) |

---

## 3. L'ontologia (`ontology.yaml`) — come aggiungere luoghi, porte, persone

Dopo **ogni** modifica:
```bash
source ~/argo_go2_ws/scripts/argo_env.sh
python3 tools/compute_distances.py src/go2_patrol_mission/config/ontology.yaml   # ricalcola le distanze
python3 tools/check_ontology.py   src/go2_patrol_mission/config/ontology.yaml    # deve stampare OK
colcon build --packages-select go2_patrol_mission                                 # installa il file
```
e riavvia la pipeline v2.

### Struttura
- `hubs`: punti del corridoio (x, y, theta).
- `places`: stanze e porte. Campi:
  - `class`: `Lab`, `CommonArea`, `AdminOffice`, `Office`, `DirectorOffice`, `Door` (vedi `classes`);
  - `pose`: dove si ferma il robot (theta in radianti, 0 = verso +x della mappa);
  - `occupant`: lista di persone (Gemma collega ogni nome alla stanza: "di' a Lia…" → `angelo_room`);
  - `synonyms`: altri nomi (il **primo** è quello che Argo usa nelle frasi);
  - `entrance`: per una stanza con porta è la porta; per una porta è l'hub più vicino;
  - `door_of` (solo porte): la stanza di cui è la porta;
  - `works_here: true`: la stanza viene chiamata con "chiama **tutti** per il caffè";
  - `dock`, `social_spot`, `needs_permission`, `viewpoints`.
- `edges`: archi `[a, b, distanza]`. **Il grafo deve restare un albero** (nessun anello).
- `classes`: `camera_allowed` decide dove si può ispezionare (privacy).

### Regole delle porte
- Il corridoio principale è una **catena** di hub e porte; una stanza con porta è collegata **solo** alla sua porta.
- Negli **uffici** (`class: Office`) Argo consegna/annuncia **dalla porta** (privacy).
- Nelle altre stanze passa dalla porta ed **entra** a parlare.

### Esempio: aggiungere la porta di Gianni
1. Rileva la posa davanti alla porta (`python3 tools/record_pose.py`, nome `gianni_door`).
2. In `places`:
   ```yaml
   gianni_door: {class: Door, door_of: gianni_room, pose: {x: …, y: …, theta: …}, entrance: hub_north2, synonyms: ["porta di Gianni"]}
   ```
   e in `gianni_room` metti `entrance: gianni_door`.
3. In `edges`: togli `[gianni_room, hub_north2, …]`, metti la porta **dentro** la catena
   (es. `[hub_north2, gianni_door, 1.0]` + `[gianni_door, hub_north3, 1.0]` al posto di
   `[hub_north2, hub_north3, …]`) e `[gianni_room, gianni_door, 1.0]`.
4. Distanze + controllo + build come sopra.

### Aggiungere una persona
Basta aggiungere il nome in `occupant` **e** in `synonyms` della sua stanza.

---

## 4. Il dominio PDDL (`patrol_domain_v2.pddl`)

| Azione | Quando si può fare | Effetto |
|---|---|---|
| `navigate ?r ?a ?b` | robot in `a`, `a`–`b` collegati e liberi, `b` accessibile, batteria ≥ riserva + consumo | robot in `b`, batteria scende |
| `announce ?r ?l` | robot in `l` per tutta l'azione, `needs_call l` | `announced l`, `to_call` −1 |
| `chat ?r ?l` | robot in `l`, `social_spot l`, **`to_call ≤ 0`** (tutti chiamati) | `socialized l` |
| `deliver_message ?r ?l` | robot in `l`, `has_message l`, `unlocked l` | `message_delivered l`, `done l` |
| `inspect_room ?r ?l` | robot in `l`, `camera_allowed l`, `unlocked l` | `inspected l` |
| `ask_permission ?r ?l ?h` | robot in `h` collegato a `l`, `needs_permission l` | `may_enter l` |
| `recharge ?r ?l` | robot alla cuccia, batteria ≤ 50 | batteria +50 |
| `advance ?p ?n` | `done p`, `precedes p n` | `unlocked n` (ordine "prima… poi…") |

Parametri numerici dall'ontologia: `speed` 0.5 m/s, `drain` 0.03 %/m, **`reserve` 15 %** (nessun piano
scende sotto il 15 %; il ritorno a casa di R1 usa riserva 0), `chat_time` 180 s.

La suite di test: `bash tools/run_planning_tests.sh` → `src/go2_patrol_mission/pddl/results/planning_results.csv`.

---

## 5. Gemma e `gemma_server.py`

### Come si avvia Gemma
`start_pipeline.sh` (finestra `gemma`) lancia llama.cpp:
```bash
~/llama.cpp/build/bin/llama-server -m ~/models/gemma-4-E2B-it-Q8_0.gguf --port 8000 \
  --ctx-size 4096 --n-gpu-layers 99 --reasoning off
```
Per cambiare modello: variabile `GEMMA_MODEL=/percorso/modello.gguf` prima di `start_pipeline.sh`.
Controllo: `curl --noproxy '*' http://127.0.0.1:8000/v1/models`.

### Parametri di `gemma_server.py`
| Parametro | Default | Significato |
|---|---|---|
| `gemma_api_base` | `http://127.0.0.1:8000/v1` | indirizzo di llama.cpp (anche argomento del launch v2) |
| `gemma_model_id` | `google/gemma-4-E2B-it` | nome del modello nella richiesta |
| `llm_timeout` | `10.0` s | oltre questo tempo risponde il **fallback** |
| `temperature` | `0.3` | più alto = frasi più varie (e più errori) |
| `ontology_file` | quella installata | da dove legge luoghi e persone |

> Con tutta la pipeline accesa Gemma è lento (~35 s per un comando G1): oggi il G1 risponde quasi sempre
> col fallback a parole chiave dopo 10 s. Per dargli più tempo: `llm_timeout` più alto (vedi `TODO.md`).

### I 5 servizi e dove sono i prompt
Tutti i prompt sono **in `gemma_server.py`**, nella funzione del servizio (variabile `system`):

| Servizio | Funzione | Cosa decide | Validazione | Fallback |
|---|---|---|---|---|
| `/argo/gemma/formulate_goals` (G1) | `formulate_goals` | missione, stanze, ordine, testo del messaggio, risposta | ID esistenti, missione in `coffee_call/message/inspect/none`, privacy, luoghi sconosciuti | `keyword_goals`: parole chiave su ID, sinonimi, occupanti, zone |
| `/argo/gemma/assess_failure` (G3) | `assess_failure` | strategia di recupero + frase | strategia in `WAIT_RETRY/ASK_CLEAR/DROP_UNREACHABLE/RETURN_HOME/ABORT` | `fallback_strategy`: prima riprova, poi salta, poi casa |
| `/argo/gemma/appraise_findings` (G4) | `appraise_findings` | NONE/REPORT dopo un'ispezione | numeri invariati | descrizione fissa |
| `/argo/gemma/speak` (G5) | `speak` | la frase per un evento | italiano, ≤ 25 parole, fatti invariati; **annuncio**: deve contenere "pausa" e "caffè", niente nomi | `template()`: una frase fissa per evento |
| `/argo/gemma/chat` (G6) | `chat` | una **barzelletta sul caffè** | niente saluti generici, deve parlare di caffè, mai ripetuta | lista `CHAT_LINES` |

### Cosa personalizzare, e dove
| Vuoi cambiare… | Dove |
|---|---|
| lo **stile delle barzellette** (le scrive Gemma) | il prompt in `chat()` di `gemma_server.py` |
| quanto Gemma è creativo / quanti tentativi | `CHAT_TEMPERATURE` (0.9) e `CHAT_ATTEMPTS` (3) |
| perché una battuta viene scartata | `bad_joke()`: saluti (`GENERIC_LINE`), non sul caffè (`COFFEE_WORDS`), copiata dalla lista, simile > 75 % a una già detta |
| le barzellette **di riserva** (solo se Gemma è spento o fallisce 3 volte) | `CHAT_LINES` in `gemma_server.py` (e `FALLBACK_LINES` in `chat_action_node.cpp`, se gemma_server è spento) |
| la **frase dell'annuncio** | `template()` (caso `announce`) e le regole in `speak()` |
| frasi fisse di batteria/guasto/permesso | `template()` |
| le parole che il fallback riconosce come missione | `keyword_goals()` (regex `coffee`, `message`, ispezione) |
| le zone del corridoio ("nord", "ovest"…) | `ZONE_WORDS` |
| le classi YOLO in italiano | `COCO_IT` |
| il limite di parole | `MAX_WORDS` |

Dopo una modifica: `colcon build --packages-select go2_patrol_mission` e riavvio.

### Test di Gemma
```bash
ros2 run go2_patrol_mission gemma_server.py &
python3 src/go2_patrol_mission/tests/run_gemma_tests.py          # 43 casi: accuratezza, latenza, fallback
python3 src/go2_patrol_mission/tests/run_gemma_tests.py --only M2 # una sola categoria
```
I casi sono in `src/go2_patrol_mission/tests/gemma_cases.yaml`: puoi aggiungerne di tuoi.

---

## 6. I comandi per Argo (`argo_cmd`)

Con la pipeline v2 accesa (`ARGO_VERSION=v2 ~/argo_go2_ws/scripts/start_pipeline.sh`):

| Tipo | Esempi | Cosa fa |
|---|---|---|
| **Messaggio (M2)** | `argo_cmd "di' a Vito che la riunione è alle 15"` · `"avvisa Paola che domani l'ufficio è chiuso"` · `"vai in openspace a dire che la pizza è arrivata"` | va alla porta (uffici) o nella stanza, dice il messaggio, torna |
| **Messaggio ordinato** | `"prima da Max e poi da Vito, digli che c'è la riunione"` | rispetta l'ordine (azione `advance`) |
| **Caffè (M1)** | `"chiama per il caffè Vito, Angelo e l'openspace"` · `"pausa caffè solo per il corridoio nord"` | annuncia in ogni stanza, chiacchiera alla zona caffè, torna |
| **Ispezione (M3)** | `"ispeziona l'openspace"` | ⚠️ `inspect_room` è ancora finto |

Regole del testo:
- I nomi delle persone e i sinonimi vengono da `ontology.yaml`.
- Il messaggio è tutto quello che segue "**che**" (nel fallback) — le parole non vengono mai cambiate.
- Luoghi o persone sconosciuti → rifiuto con spiegazione; frasi ambigue ("vai nel lab") → domanda.
- Non usare per ora il direttore né "chiama **tutti**" (passa dal direttore: `ask_permission` è finto).
- Cambia il testo a ogni prova: il nodo voce scarta una frase identica all'ultima detta.

Per fermare: **telecomando**; da terminale `Ctrl+C` nella finestra `patrol` e
`ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`.

---

## 7. Il behavior tree (`argo_mission_bt`)

### L'albero (`behavior_trees/mission.xml`)
```
SafeSequence (radice reattiva)
├── Safe:        BatteryOK ∧ SensorsOK, altrimenti R1 (batteria) o R2 (sensori)
├── HasMission:  MissionActive, altrimenti WaitCommand → GemmaFormulateGoals → Say → IsTrue → SetGoals
└── RunMission:  GoalSatisfied, oppure Retry×3 [ PlanAndAct | Recover ], oppure ReturnHome
                 poi MissionReport → Say → ClearMission
```
- **PlanAndAct**: `SyncKB → ComputePlan → ExecutePlan → GemmaAppraiseFindings`
- **Recover**: `SyncKB → GemmaAssessFailure → Say → ApplyStrategy → AlwaysFailure` (forza il nuovo tentativo)
- **R1** (SOC < 15 %): annulla il piano, lo dice, va a casa con riserva 0, si sdraia, aspetta la carica.
- **R2** (`/argo/health` FATAL): annulla, lo dice, aspetta che il sensore torni (⚠️ `health_monitor` non c'è ancora).

L'XML si modifica **senza ricompilare** il C++ (basta `colcon build --packages-select argo_mission_bt`
per installarlo): soglie di batteria (`min_soc`), numero di tentativi (`num_attempts`), frasi fisse di R1/R2.

### I nodi (C++, `src/argo_mission_bt/src/`)
| File | Nodi |
|---|---|
| `conditions.cpp` | `BatteryOK`, `BatteryLow`, `SensorsOK`, `MissionActive`, `GoalSatisfied`, `IsTrue`, `WaitBatteryOK`, `WaitHealthy` |
| `plansys2_nodes.cpp` | `SyncKB`, `SetGoals`, `SetHomeGoal`, `ClearMission`, `ComputePlan`, `ExecutePlan`, `CancelPlan`, `ApplyStrategy` |
| `gemma_nodes.cpp` | `GemmaFormulateGoals`, `GemmaAssessFailure`, `GemmaAppraiseFindings` (chiamate asincrone) |
| `interaction_nodes.cpp` | `WaitCommand`, `Say`, `MissionReport`, `LieDown` |
| `context.cpp` | ontologia, client PlanSys2, operazioni sul KB, regola "uffici dalla porta" (`delivery_node`) |
| `mission_node.cpp` | il nodo: sottoscrizioni, servizio `/argo/command`, caricamento del KB, `SafeSequence`, ciclo di tick |

Strategie di recupero (`ApplyStrategy`):
- `WAIT_RETRY`: aspetta 20 s, riapre i passaggi bloccati, riprova;
- `ASK_CLEAR`: dice "Mi scusi, posso passare?", aspetta 10 s, riprova;
- `DROP_UNREACHABLE`: toglie dal goal le stanze irraggiungibili (calcolate sul grafo);
- `RETURN_HOME` / `ABORT`: goal = solo casa.

Quando un `navigate` fallisce, quel tratto viene segnato non percorribile (`path_free` falso) finché la
missione non finisce o una strategia lo riapre.

### Parametri di `mission_node`
| Parametro | Default |
|---|---|
| `ontology_file` | ontologia installata |
| `bt_xml` | `behavior_trees/mission.xml` installato |
| `groot` | `true` (monitor Groot su porta 1666) |
| `tick_hz` | `10` |
| `lowstate_topic` | `/lowstate` (⚠️ da verificare sul robot, c'è anche `/lf/lowstate`) |

---

## 8. Gli executor (parametri utili)

| Executor | Parametri | Note |
|---|---|---|
| `navigate_action_node` | `nav_timeout_min` 90 s, `nav_timeout_factor` 3, `voice_wait_min` 1.5 s, `voice_wait_max` 45 s, `unlock_before_move` true, `check_standing` false | prima di partire aspetta che la voce finisca e manda **sempre** `balance_stand` (sblocco Select); se non arriva entro il limite annulla il goal e fallisce |
| `deliver_message_action_node` | `voice_start_timeout` 8 s, `voice_max_time` 90 s | testo da `/argo/messages`; "Vito, ho un messaggio per te: …" o "Ho un messaggio per tutti: …" |
| `announce_action_node` | `voice_start_timeout` 8 s | "Vito e Nicola M: Pausa caffè! …"; frase da G5 o fissa |
| `chat_action_node` | `chat_time` (−1 = ontologia), `line_period` 25 s | barzellette per `chat_time` secondi; non fallisce mai |
| `advance_action_node` | — | finisce subito |
| `fake_action_executors` | `actions`, `duration_s`, `fail_on` | per prove; `fail_on` fa fallire apposta un'azione |

La **voce** (`go2_hardware/scripts/argo_voice_node.py`): legge `/argo/say`, pubblica `/argo/voice_busy`,
e **non parla mentre il robot cammina** (usa gli eventi `start/success/failure:navigate` di
`/patrol/action_events`). Parametri: `hold_while_moving`, `max_moving_hold`.

---

## 9. Avvio e prove

### Pipeline v2
```bash
ARGO_VERSION=v2 ~/argo_go2_ws/scripts/start_pipeline.sh
```
Pronto quando: finestra `patrol` → `Argo pronto: aspetto comandi su /argo/command`; finestra `voice` →
`Collegato al Go2`. Poi 2D Pose Estimate in RViz e `argo_cmd "…"` nella finestra `cmd`.
La finestra `explain` mostra tutto quello che Argo dice.

### Argomenti di `argo_mission_launch.py` (lancio a mano)
```bash
ros2 launch go2_patrol_mission argo_mission_launch.py \
  real:=navigate,advance,deliver_message,announce,chat \   # executor veri (gli altri finti)
  chat_time:=60 \                                          # chiacchierata più corta
  gemma_api_base:=http://127.0.0.1:8000/v1 \
  groot:=true fail_on:="navigate go2 hub_west vito_door"   # (prove) fa fallire un tratto
```

### Prove senza muovere il robot (dominio DDS isolato)
```bash
export ROS_DOMAIN_ID=77
ros2 launch go2_patrol_mission argo_mission_launch.py real:=deliver_message,announce,chat fake_duration_s:=1.0
ros2 service call /argo/command go2_interfaces/srv/GemmaCommand "{command: \"di' a Vito che …\"}"
```

---

## 10. Modifiche fatte a PlanSys2 (compilato dal workspace)

Da ricordare se si aggiorna PlanSys2 (`src/ros2_planning_system`, branch `foxy-devel`):
1. `ActionExecutor`: il timer di ripetizione non chiama più oggetti distrutti (crash a fine piano).
2. `lifecycle_manager.hpp`: timeout 3 s → 15 s (il dominio v2 richiede ~3.4 s per essere letto).
3. `BTBuilder::get_graph`: requisiti numerici cumulativi (`chat` dopo tutti gli `announce`) e
   riduzione transitiva corretta (prima poteva mettere consegna e ritorno in parallelo, a caso).

---

## 11. Problemi noti e soluzioni rapide

| Sintomo | Causa / cosa fare |
|---|---|
| `argo_cmd` risponde "Conosco solo: laboratorio, corridoio, openspace" | stai usando la **v1**: riavvia con `ARGO_VERSION=v2` |
| Il robot resta fermo finché non premi Select | stallo del Go2: `navigate` manda `balance_stand`; controlla nel log `Sblocco il robot` |
| Argo non dice una frase | il nodo voce scarta la frase identica all'ultima: cambia testo |
| Risposta di Gemma dopo ~10 s, sempre `fallback=1` | Gemma lento con la pipeline accesa (vedi `llm_timeout`) |
| Il robot si ferma a ogni hub/porta | ogni nodo è un goal Nav2 separato (limite noto) |
| Tastiera/mouse non funzionano più dopo aver staccato l'hub | `nmcli device disconnect eth0` **prima** di staccare l'hub; poi solo riavvio |
| Internet si stacca ogni ~7 min | rinnovo DHCP che fallisce su eth0 (vedi `TODO.md`) |
