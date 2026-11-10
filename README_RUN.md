# Argo GO2 — Come avviare il sistema (Nav2 Humble + PlanSys2 + Gemma)

Guida operativa per avviare, usare e fermare la pipeline completa a bordo del Unitree Go2
(Jetson Orin NX). Tutti i comandi si lanciano **sul robot**.

---

## Architettura in breve

| Parte | Dove gira | Cosa fa |
|-------|-----------|---------|
| Gemma 4 E2B (`llama-server`) | Jetson, porta 8000 | Interpreta i comandi in linguaggio naturale e spiega gli eventi |
| Bringup (Foxy) | tmux, finestra `bringup` | Driver Go2, LiDAR Hesai, RealSense, filtro nuvola, `/scan`, TF |
| **Nav2 Humble** | container docker `nobody-unitree_ros-1` | AMCL, mappa, planner Smac, controller MPPI, costmap 3D (STVL) |
| PlanSys2 + Gemma bridge (Foxy) | tmux, finestra `patrol` | Pianifica le missioni (`navigate` / `inspect`) e manda i goal a Nav2 |

Sensori e pianificazione girano su ROS 2 **Foxy**; la navigazione gira su ROS 2 **Humble** nel
container. Comunicano tramite CycloneDDS sull'interfaccia **eth1** (rete interna del robot,
`192.168.123.x`).

---

## 0. Setup una tantum (già fatto su questo robot)

Questi passi servono solo la prima volta o su un robot nuovo.

```bash
# tmux
sudo apt install -y tmux

# Buffer UDP grandi per CycloneDDS (le nuvole Hesai sono ~1.5 MB).
# Senza questo il filtro della nuvola si blocca e il robot "perde" la localizzazione.
sudo tee /etc/sysctl.d/60-cyclonedds.conf <<'EOF'
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --system

# Il container Nav2 non deve partire da solo all'accensione (lo avvia lo script)
docker update --restart=no nobody-unitree_ros-1
```

Verifica: `sysctl net.core.rmem_max` deve dare `2147483647`.

---

## 1. Avvio

Prima di avviare: robot **acceso e in piedi**, telecomando a portata di mano.

```bash
~/argo_go2_ws/scripts/start_pipeline.sh
```

Lo script apre una sessione tmux `argo` con una finestra per ogni componente. Ogni finestra
**aspetta da sola** che il passo precedente sia pronto: non serve fare niente a mano.

| Finestra | Cosa contiene | Parte quando |
|----------|---------------|--------------|
| `gemma` | `llama-server` (se è già acceso non lo rilancia) | subito |
| `bringup` | driver Go2 + Hesai + RealSense + filtro + `/scan` | arrivano i dati del Go2 |
| `nav2` | log del container Nav2 Humble | ci sono TF `odom→base_link`, `/scan` e nuvola filtrata |
| `patrol` | PlanSys2 + action node + Gemma bridge | Nav2 è attivo e Gemma risponde |
| `explain` | diario della missione (messaggi di Gemma) | il bridge è pronto |
| `cmd` | terminale per i comandi + guida dei luoghi | — |

L'avvio completo richiede circa **2 minuti** (il container ricompila il suo pacchetto a ogni avvio).
È pronto quando nella finestra `nav2` compare `Managed nodes are active` e nella finestra
`explain` compare `In ascolto su /supervisor/explanation ...`.

### Opzioni

```bash
YOLO=1 ~/argo_go2_ws/scripts/start_pipeline.sh   # anche il detector YOLO
RVIZ=1 ~/argo_go2_ws/scripts/start_pipeline.sh   # apre anche RViz
NAV=foxy ~/argo_go2_ws/scripts/start_pipeline.sh # Nav2 Foxy locale invece di Humble (sconsigliato)
```

Con `NAV=humble` (default) la mappa è quella fissata nel launch del container:
`~/nobody/src/nobody_bringup/maps/institute_stiima_cnr.yaml`. La variabile `MAP` vale solo con `NAV=foxy`.

---

## 2. Muoversi in tmux

Tutti i comandi tmux si danno in due tempi: premi **`Ctrl+b`**, lascia, poi premi il tasto.

| Azione | Tasti |
|--------|-------|
| Elenco finestre (il modo più semplice) | `Ctrl+b` → `w`, frecce, Invio |
| Finestra successiva / precedente | `Ctrl+b` → `n` / `p` |
| Nuova finestra | `Ctrl+b` → `c` |
| Scorrere i log | `Ctrl+b` → `[`, frecce / PagSu, `q` per uscire |
| Staccarsi lasciando tutto acceso | `Ctrl+b` → `d` |
| Ricollegarsi | `tmux attach -t argo` |

---

## 3. Localizzazione (fondamentale)

1. Apri RViz **con l'ambiente giusto** (buffer DDS grandi, interfaccia eth1):
   ```bash
   source ~/argo_go2_ws/scripts/argo_env.sh && rviz2 -d ~/argo_go2_ws/go2_nav2.rviz
   ```
2. Clicca **2D Pose Estimate**, clicca sulla mappa dove si trova davvero il robot e trascina
   nella direzione del muso.
3. Controlla che **lo scan (rosso) si sovrapponga ai muri (neri)**. Se non coincide, rifai la
   2D Pose Estimate. Quasi tutti i casi di "robot che si perde" nascono da una posa iniziale sbagliata.

All'avvio AMCL si mette in (0.377, 0.001, yaw -0.065): se il robot parte da quel punto e con quell'orientamento
la 2D Pose Estimate non serve; se parte da un altro punto va data.

---

## 4. Dare comandi

### Prova manuale (senza Gemma)
In RViz: **Nav2 Goal**, clicca un punto a 2–3 m in zona libera.

### Con Gemma, in linguaggio naturale
Nella finestra `cmd`:

```bash
argo_cmd "vai a controllare il corridoio"
argo_cmd "passa dal corridoio e poi ispeziona il laboratorio"
argo_cmd "fai un giro: corridoio, poi lab, poi openspace"
argo_help        # mostra i luoghi conosciuti e gli esempi
```

- Il cane conosce **solo tre luoghi**: laboratorio di robotica (`lab_robotics`),
  corridoio (`corridor`), openspace (`openspace`).
- Qualsiasi altro luogo ("vai in cucina", "ufficio del prof"…) viene **rifiutato** con spiegazione.
  Se un comando contiene anche un solo luogo sconosciuto, viene rifiutato per intero.
- Un nuovo comando **annulla** quello in corso.
- Il bridge capisce da solo in quale stanza si trova il robot (dalla posa AMCL).

### Seguire la missione
La finestra `explain` mostra il diario con l'orario:

```
[11:58:01] Goal ACCETTATO: corridoio. Gemma: ... Piano avviato.
[11:58:02] Mi sto muovendo da laboratorio di robotica verso corridoio.
[11:59:40] Sono arrivato in corridoio.
[11:59:41] Inizio l'ispezione di corridoio.
[11:59:48] Missione completata: ho ispezionato corridoio.
```

In qualsiasi terminale (dopo il `source`) puoi aprirlo con `argo_explain`.

### Fermare il robot in emergenza
- **Telecomando** (sempre la prima scelta).
- Da terminale: `Ctrl+C` nella finestra `patrol` ferma PlanSys2.

---

## 5. Registrare lo schermo

```bash
source ~/argo_go2_ws/scripts/argo_env.sh
argo_rec test_corridoio        # -> ~/Videos/test_corridoio.mp4, premi q per fermare
```

In alternativa la scorciatoia GNOME `Ctrl+Shift+Alt+R` (limite di durata già rimosso).
`argo_rec` pesa meno sulla CPU: preferiscilo durante la navigazione.

---

## 6. Spegnimento

```bash
~/argo_go2_ws/scripts/stop_pipeline.sh
```

Ferma nell'ordine giusto tutte le finestre tmux **e il container Nav2 Humble**.

---

## 7. Problemi comuni

| Sintomo | Causa probabile | Cosa fare |
|---------|-----------------|-----------|
| Una finestra resta su `[argo] Aspetto ...` | il passo precedente non è pronto | guarda la finestra `bringup` |
| Nel `bringup` compare `hesai_ros_driver_node ... exit code -11` | segfault casuale all'avvio del driver Hesai | nessuna: si rilancia da solo dopo 2 s (`respawn`) |
| Nav2: `Extrapolation Error` / `Timed out waiting for transform` | manca `/scan` → AMCL non aggiorna `map→odom` | controlla `/scan` (sotto) |
| Il robot si perde | posa iniziale sbagliata o scan assente | rifai la 2D Pose Estimate, controlla `/scan` |
| RViz non vede nulla | aperto senza `argo_env.sh` | riaprilo come al punto 3 |
| `argo_cmd: command not found` | terminale senza ambiente | `source ~/argo_go2_ws/scripts/argo_env.sh` |
| Goal rifiutato | luogo non conosciuto | `argo_help` per l'elenco |

Controlli rapidi (dopo il `source`):

```bash
ros2 topic echo --qos-reliability best_effort /scan | grep frame_id   # deve scorrere
ros2 run tf2_ros tf2_echo map base_link                               # deve aggiornarsi
ps -o etime= -p $(pgrep -f hesai_ros_driver_node)                     # da quanto gira il Hesai
nstat -az UdpRcvbufErrors                                             # non deve crescere
```

**Non usare** i pulsanti *Startup / Reset* del pannello Navigation 2 di RViz con `NAV=foxy`:
su Foxy mandano in crash il planner. Con Humble non sono stati provati: in caso di dubbio usa
`stop_pipeline.sh` + `start_pipeline.sh`.

---

## 8. Limiti noti

- **Waypoint `openspace`** in `src/go2_patrol_mission/config/waypoints.yaml` è attualmente **su un
  muro** (4.0, 2.0): va spostato. Prendi le coordinate con *Publish Point* in RViz
  (`ros2 topic echo /clicked_point`) e aggiorna il file, poi
  `colcon build --packages-select go2_patrol_mission`.
- **Velocità**: in `~/nobody/src/nobody_bringup/config/nav2_params_gio.yaml` MPPI e
  velocity smoother sono a 1.5 m/s.
- **`/odom`** viene pubblicato una sola volta dal `go2_driver` (la TF `odom→base_link` invece è
  continua): il controller MPPI non conosce la velocità reale del robot.
- **Il container Humble** usa `~/nobody/cyclonedds.xml` (senza il buffer UDP da 10 MB).
- Per aggiungere un luogo vanno aggiornati: `ROOM_INFO` e `ROOM_ALIASES` in
  `src/go2_patrol_mission/scripts/gemma_bridge.py`, `config/waypoints.yaml` e i `connected` in
  `setup_plansys2_problem`.

---

## 9. File coinvolti

| File | Ruolo |
|------|-------|
| `scripts/start_pipeline.sh` / `stop_pipeline.sh` | avvio / spegnimento in tmux |
| `scripts/argo_env.sh` | ambiente (Foxy, CycloneDDS, workspace) + comandi `argo_cmd`, `argo_help`, `argo_explain`, `argo_rec` |
| `scripts/cyclonedds_go2_bigbuf.xml` | CycloneDDS su eth1 con buffer 10 MB (scelto in automatico) |
| `scripts/cyclonedds_go2.xml` | stessa config senza buffer grande (fallback) |
| `~/nobody/src/nobody_bringup/launch/nav.launch_gio.py` | launch Nav2 Humble nel container |
| `~/nobody/src/nobody_bringup/config/nav2_params_gio.yaml` | parametri Nav2 Humble |
| `src/go2_patrol_mission/` | PlanSys2, action node, Gemma bridge, waypoint |
| `src/hesai_ros_driver/launch/start.py` | driver Hesai con `respawn` |
