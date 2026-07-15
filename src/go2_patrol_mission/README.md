# Go2 Patrol Mission

Pacchetto ROS 2 per pattugliamento e ispezione multi-stanza con il Unitree Go2, utilizzando Nav2 e PlanSys2.

## 1. Prerequisiti a Bordo del Robot (Installazione PlanSys2)

Il tuo robot con ROS 2 Foxy non ha PlanSys2 preinstallato. Segui questi passi a bordo del Go2:

```bash
cd ~/argo_go2_ws/src
git clone -b foxy-devel https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git

# Ritorna al workspace e installa le dipendenze
cd ~/argo_go2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Compila il workspace (includerà sia PlanSys2 che questo nuovo pacchetto)
colcon build --symlink-install
```

## 2. Esecuzione del Sistema

Per far funzionare il tutto, hai bisogno di avere i sistemi base attivi (Driver Go2, Lidar, Nav2). 

**Terminale 1: Avvio Sistemi Base e Navigazione**
Avvia il driver e Nav2 come fai di solito (ad esempio con `navigation.launch.py` o il tuo launch file principale).

**Terminale 2: Avvio della Missione di Pattugliamento**
```bash
source ~/argo_go2_ws/install/setup.bash
ros2 launch go2_patrol_mission patrol_mission_launch.py
```
Questo launch file avvierà:
1. PlanSys2 (Monolithic) caricando il dominio `patrol_domain.pddl`
2. Nodo Azione Navigate (C++)
3. Nodo Azione Inspect (C++)
4. Nodo Bridge Gemma (Python)

## 3. Inviare un Comando Naturale

Usa il servizio esposto per inviare un comando in linguaggio naturale. Il bridge Python cercherà nel testo le stanze supportate (`lab_robotics`, `corridor`, `openspace`), configurerà il problema in PlanSys2, e farà partire il piano d'azione in automatico!

**Terminale 3:**
```bash
source ~/argo_go2_ws/install/setup.bash
ros2 service call /gemma_interface/send_command go2_interfaces/srv/GemmaCommand "{command: 'Vai a controllare lab_robotics e poi passa da openspace'}"
```

### Note
- **Navigazione:** I waypoint sono salvati in `config/waypoints.yaml`. Le coordinate possono essere cambiate a seconda della mappa.
- **Ispezione:** Quando il robot reaches (raggiunge) una stanza, l'azione di ispezione invia un comando `/euler` (pitch in avanti di 0.3) al driver, aspetta 3 secondi, e poi ripristina la postura a pitch 0.0.
