# TODO — cose lasciate in sospeso (Argo v2)

Aggiornato man mano durante il tutorial. Ogni voce: cosa, perché è in sospeso, da dove ripartire.

## Gemma / gemma_server (passo 7)
- [ ] **Latenza di Gemma con la pipeline accesa.** Un comando G1 richiede ~35 s (578 token di prompt,
      ~4-5 token/s) con driver + Hesai + Nav2 + filtro nuvola su CUDA accesi: il timeout di 10 s del README
      scatta sempre e risponde il fallback a parole chiave. Le frasi G5 (prompt corto) rispondono in ~4 s.
      Opzioni: timeout per servizio (G1 a 30 s), prompt G1 più corto, chiamate asincrone dal BT,
      liberare la GPU (filtro nuvola su CPU?), modello più piccolo.
- [ ] **Rifare il test set B con la GPU libera** (pipeline spenta, solo llama-server) e annotare
      accuratezza / latenza / fallback per le slide. Primo giro (con fallback): 39/43 (91%), poi 3 casi
      del fallback corretti.
- [ ] **Fallback G1: luoghi sconosciuti senza nome proprio** ("chiama per il caffè la cucina") non
      vengono riconosciuti (solo "vai da Pippo" lo è). Con Gemma attivo finiscono in `unknown`.

## Pose e ontologia (passi 1-3)
- [ ] Hub e punti di vista ricavati dalla mappa, non rilevati sul posto: verificarli con Nav2
      (`hub_north2` ha solo 0.36 m di margine sulla mappa). Correggere con `tools/record_pose.py`.
- [ ] `vito_room` (e gli altri uffici) devono essere **sulla porta, lato corridoio** (privacy): il 20/11
      con la porta di Vito chiusa il robot ha tentato di entrare (posa 7.80, 0.29 dentro l'ufficio) ed è
      rimasto bloccato contro la porta. Rilevare le pose con `tools/record_pose.py`.
- [ ] **Filtro della nuvola LiDAR** (`go2_bringup/launch/go2.launch.py`, `bounding_box` ±0.5 m): scarta
      gli ostacoli entro 0.5 m davanti al sensore, una porta vicina diventa invisibile -> il robot l'ha
      toccata. Verificare e ridurre il box al solo corpo del robot.
- [ ] Theta delle stanze presi dal design, non rilevati.
- [ ] `speed` (0.5 m/s) e `drain` (0.03 %/m) da calibrare con dei rosbag.
- [ ] Rieseguire `tools/compute_distances.py` dopo ogni modifica delle pose.

## Planning e PlanSys2 (passi 4-5)
- [ ] **Patch locali a PlanSys2 (foxy-devel, compilato dal workspace)**, da tenere a mente se si
      aggiorna PlanSys2: timer di `ActionExecutor` (crash a fine piano), timeout del lifecycle manager
      3 s -> 15 s, `BTBuilder` con requisiti numerici cumulativi (`chat`).
- [ ] Gli ottimi P2/P3 del README (99 m, 183 m) erano sulle distanze vecchie: aggiornare il README con
      96.8 m e 182.4 m.
- [ ] Tempi di pianificazione misurati solo sull'Orin (il README chiede anche il laptop).
- [ ] R1: il BT deve mettere `reserve go2 = 0` prima di pianificare il ritorno a casa (passo 10/11).

## Missioni v2 (passi 8-12) — stato al 20/11
- [x] `argo_cmd` -> gemma_server (G1) -> mission BT -> PlanSys2 -> executor: provato offline per i
      messaggi (Vito alla porta, openspace) e per il recupero (porta bloccata -> riprova -> torna in lab).
- [x] announce e chat veri (20/11): provati offline con una chiamata caffè (Vito + openspace).
- [ ] **Executor ancora finti** in `argo_mission_launch.py`: inspect_room, ask_permission, recharge
      (passi 8b/8c). Non mandare missioni che passano dal direttore (ask_permission finto: entrerebbe)
      né ispezioni. "Chiama tutti per il caffè" include il direttore: per ora chiamare stanze precise.
- [x] Porte sul corridoio (20/11): Vito, openspace, amministrazione, vision lab, Angelo, Max, Beppe,
      direttore. Il corridoio è una catena di hub e porte; gli uffici si servono dalla porta.
- [ ] Manca la porta di **Gianni** (gianni_room collegata direttamente a hub_north2).
- [ ] Theta delle porte calcolati "verso la stanza", non rilevati.
- [ ] Più nodi = più goal Nav2 = più soste brevi lungo il corridoio (limite noto): valutare
      NavigateThroughPoses per i tratti di solo corridoio.
- [ ] `health_monitor.py` (passo 9) non c'è: R2 non scatta mai.
- [ ] R1: verificare il nome del topic della batteria sul robot (`lowstate_topic`, default `/lowstate`;
      sul Go2 c'è anche `/lf/lowstate`).
- [ ] Altre patch a PlanSys2: `BTBuilder` prune_forward sostituito da una riduzione transitiva
      corretta (prima poteva mettere in parallelo consegna e ritorno, a caso).
- [ ] Sblocco `balance_stand` ora **sempre** prima di ogni navigate (e a fine ispezione): nello stato
      di stallo il Go2 riporta body_height 0.09 m anche in piedi, e il controllo "non rialzare un robot a
      cuccia" lo lasciava bloccato (20/11, v2). Il controllo resta con `check_standing:=true`.
- [ ] Limite di `navigate` alzato a 90 s minimo (60 s fermava il robot mentre passava la porta di Vito).

## Executor (passo 8)
- [ ] `plansys2_terminal` su Foxy non esegue una singola azione (`run (azione)` è un ToDo nel codice):
      si usa `tools/run_action.py`.
- [ ] Ispezione con camera e YOLO (passo 8c `inspect_room`) rimandata: per ora prove reali senza camera.
- [ ] **argo_voice_node scarta una frase identica all'ultima detta** (filtro anti-ripetizione): due
      consegne di fila con lo stesso testo fanno fallire la seconda (`voce non attiva`). Nelle prove usare
      testi diversi; da sistemare nel nodo voce (es. niente filtro per /argo/say) — CLAUDE.md vieta di
      modificare go2_hardware, va deciso.
- [ ] I goal di `navigate` non vengono annullati se si ferma il nodo con Ctrl+C: per fermare il robot
      usare il telecomando o `ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`.
- [ ] Dopo un `navigate` fallito il KB resta senza `robot_at` (l'effetto "at start" lo toglie): lo
      ripristina SyncKB del BT (passo 10, robot_at = nodo più vicino alla posa AMCL).

## Robot e sistema
- [ ] **Staccare l'hub USB-C con eth0 attiva blocca tutto l'USB fino al riavvio** (20/11 12:51 e 9/11):
      r8152 `Rx status -71` -> `xHCI host controller ... HC died` -> `unregister_netdevice: waiting for
      eth0`. Prima di staccare: `nmcli device disconnect eth0`. Soluzione definitiva: internet su
      un'interfaccia che non sia sull'hub staccabile (Wi-Fi USB su 1-2 o LAN su porta fissa), oppure
      lavorare via SSH/desktop remoto senza toccare l'hub.
- [ ] **Internet (eth0) si stacca ogni 7.5 min**: 
      il rinnovo DHCP fallisce a metà lease (900 s) e serve riconnettere a mano. Chiedere all'IT un IP statico riservato al MAC, oppure uno script di NetworkManager che riconnette eth0 quando il DHCP fallisce.
- [ ] Stallo del Go2 dopo averlo alzato col telecomando (ignora i comandi finché non si preme Start):
      rete di sicurezza nel driver (BalanceStand al primo cmd_vel dopo una pausa) proposta, non fatta.
- [ ] Il robot si scalda (~74 °C) con pipeline completa + Gemma: monitorare durante le prove.
- [ ] Ogni hub è un goal Nav2 separato: Argo si ferma brevemente a ogni hub (limite noto del README).

## Git
- [ ] Commit dei passi 0-7 ancora da fare sul branch `argo-v2-planning`.
