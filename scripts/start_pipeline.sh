#!/usr/bin/env bash
# Avvia tutta la pipeline Argo GO2 (Gemma + bringup + Nav2 + YOLO + PlanSys2) in tmux.
#
# Uso:
#   ~/argo_go2_ws/scripts/start_pipeline.sh                 # tutto
#   MAP=/percorso/mappa.yaml ~/argo_go2_ws/scripts/start_pipeline.sh
#   YOLO=0 RVIZ=1 ~/argo_go2_ws/scripts/start_pipeline.sh   # senza YOLO, con RViz
#   NAV=foxy ~/argo_go2_ws/scripts/start_pipeline.sh        # Nav2 Foxy locale invece del container Humble
#
# NAV=humble (default): Nav2 gira nel container docker 'nobody-unitree_ros-1' (ROS 2 Humble,
#   nobody/src/nobody_bringup/launch/nav.launch_gio.py + config/nav2_params_gio.yaml, mappa fissa
#   nel launch). Sensori, driver, PlanSys2 e Gemma restano qui su Foxy. MAP viene ignorata.
#
# Navigazione in tmux:  Ctrl+b poi n / p (finestra succ./prec.), Ctrl+b poi 0..7, Ctrl+b poi d (stacca)
# Per fermare tutto:    ~/argo_go2_ws/scripts/stop_pipeline.sh

set -e

SESSION=argo
WS="$HOME/argo_go2_ws"
ENV="$WS/scripts/argo_env.sh"
MAP="${MAP:-$WS/src/go2_nav/maps/map_1779889723.yaml}"
YOLO="${YOLO:-0}"
RVIZ="${RVIZ:-0}"
NAV="${NAV:-humble}"
NAV_CONTAINER=nobody-unitree_ros-1
GEMMA_MODEL="${GEMMA_MODEL:-$HOME/models/gemma-4-E2B-it-Q8_0.gguf}"

if ! command -v tmux >/dev/null; then
  echo "tmux non è installato. Installa con:  sudo apt install -y tmux"
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "La sessione '$SESSION' è già attiva: mi ci collego. (Per ripartire da zero: scripts/stop_pipeline.sh)"
  [ -n "$TMUX" ] && exec tmux switch-client -t "$SESSION" || exec tmux attach -t "$SESSION"
fi

if [ "$NAV" != "humble" ] && [ "$NAV" != "foxy" ]; then
  echo "NAV deve essere 'humble' o 'foxy' (ora: $NAV)"; exit 1
fi
if [ "$NAV" = "humble" ] && ! docker inspect "$NAV_CONTAINER" >/dev/null 2>&1; then
  echo "Container $NAV_CONTAINER non trovato: usa NAV=foxy oppure crealo con 'docker compose' in ~/nobody"; exit 1
fi

if [ "$NAV" = "foxy" ] && [ ! -f "$MAP" ]; then
  echo "Mappa non trovata: $MAP"; exit 1
fi

# Il container Nav2 Humble deve partire DOPO il bringup (e mai insieme al Nav2 Foxy):
# se è già acceso lo fermo, verrà riavviato dalla finestra nav2 quando i sensori sono pronti.
if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${NAV_CONTAINER}\$"; then
  if [ "$NAV" = "humble" ]; then
    echo "Il container $NAV_CONTAINER è già acceso: lo fermo, ripartirà dopo il bringup."
    docker stop "$NAV_CONTAINER" >/dev/null
  else
    echo "ATTENZIONE: il container $NAV_CONTAINER è attivo (contiene un altro Nav2"
    echo "            che va in conflitto con il Nav2 Foxy su /tf, /cmd_vel e /navigate_to_pose)."
    echo
    echo "   s + Invio  -> fermo il container e continuo   (consigliato)"
    echo "   Invio      -> continuo lasciandolo acceso"
    echo "   Ctrl+C     -> annullo tutto"
    ans=""
    read -r -t 20 -p "Scelta (default tra 20 s: continuo): " ans || true
    echo
    if [ "$ans" = "s" ] || [ "$ans" = "S" ]; then
      docker stop "$NAV_CONTAINER" >/dev/null && echo "Container fermato."
    fi
  fi
fi

# Ogni finestra è una bash con l'ambiente già caricato (niente prompt foxy/noetic, niente venv)
SHELL_CMD="bash --rcfile $ENV -i"

new_win() {  # new_win <nome> <comando>
  tmux new-window -t "$SESSION" -n "$1" "$SHELL_CMD"
  tmux send-keys  -t "$SESSION:$1" "$2" C-m
}

# Ripulisci il daemon ROS 2 (potrebbe avere in cache la vecchia config DDS su eth0)
bash -c "source $ENV >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1" || true

# 0 — Gemma (llama.cpp). Se è già acceso sulla porta 8000 non lo rilancia.
# Cronologia lunga: il driver Hesai stampa una riga per frame (10/s) e con il default (2000 righe)
# gli errori spariscono dopo ~3 minuti.
tmux start-server \; set-option -g history-limit 100000
tmux new-session -d -s "$SESSION" -n gemma "$SHELL_CMD"
tmux send-keys -t "$SESSION:gemma" \
  "if curl -sf http://127.0.0.1:8000/v1/models >/dev/null; then echo 'Gemma già in esecuzione sulla porta 8000'; else ~/llama.cpp/build/bin/llama-server -m $GEMMA_MODEL --port 8000 --ctx-size 4096 --n-gpu-layers 99 --reasoning off; fi" C-m

# 1 — Bringup robot: aspetta che arrivino i dati del Go2, poi driver + LiDAR + RealSense
new_win bringup \
  "wait_topic /utlidar/robot_pose geometry_msgs/msg/PoseStamped && ros2 launch go2_bringup go2.launch.py lidar:=True realsense:=True"

# 2 — Nav2: aspetta la TF odom->base_link e i dati LiDAR, poi avvia Nav2 (container Humble o Foxy locale)
if [ "$NAV" = "humble" ]; then
  # Il container all'avvio ricompila nobody_bringup e lancia nav.launch_gio.py (AMCL + Nav2 Humble).
  # Qui mostriamo i suoi log; Ctrl+C ferma solo la visualizzazione: per fermarlo usa stop_pipeline.sh
  new_win nav2 \
    "wait_tf odom base_link && wait_topic /scan sensor_msgs/msg/LaserScan && wait_topic /lidar_points_filtered sensor_msgs/msg/PointCloud2 && docker start $NAV_CONTAINER && docker logs -f --since 2s $NAV_CONTAINER"
else
  new_win nav2 \
    "wait_tf odom base_link && wait_topic /scan sensor_msgs/msg/LaserScan && ros2 launch go2_nav navigation.launch.py map:=$MAP"
fi

# 3 — YOLO (opzionale)
if [ "$YOLO" = "1" ]; then
  new_win yolo "wait_topic /camera/color/image_raw sensor_msgs/msg/Image && ros2 run go2_nav yolo_detector.py"
fi

# 4 — PlanSys2 + action executors + Gemma bridge: aspetta Nav2 attivo e Gemma pronto
new_win patrol "wait_nav2 && wait_gemma && ros2 launch go2_patrol_mission patrol_mission_launch.py"

# 5 — Spiegazioni di Gemma
new_win explain "wait_plansys2 && argo_explain"

# 6 — RViz (opzionale) per dare la 2D Pose Estimate
if [ "$RVIZ" = "1" ]; then
  new_win rviz "wait_nav2 && rviz2 -d $WS/go2_nav2.rviz"
fi

# 7 — Terminale comandi: comando pronto ma NON eseguito (premi Invio quando hai settato la posa iniziale)
tmux new-window -t "$SESSION" -n cmd "$SHELL_CMD"
tmux send-keys -t "$SESSION:cmd" "clear; argo_help; echo 'Prima: 2D Pose Estimate in RViz. Poi scrivi il comando qui sotto.'" C-m

tmux select-window -t "$SESSION:bringup"
[ -n "$TMUX" ] && exec tmux switch-client -t "$SESSION" || exec tmux attach -t "$SESSION"
