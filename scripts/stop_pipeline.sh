#!/usr/bin/env bash
# Ferma la pipeline Argo GO2: manda Ctrl+C a ogni finestra tmux, aspetta, poi chiude la sessione.
SESSION=argo

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Nessuna sessione '$SESSION' attiva."
  exit 0
fi

# Ordine inverso: prima PlanSys2/YOLO, poi Nav2, poi bringup, per ultimo Gemma
for w in cmd rviz explain patrol yolo nav2 bringup gemma; do
  tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true
  sleep 1
done

# Nav2 Humble nel container (se usato)
if docker ps --format '{{.Names}}' 2>/dev/null | grep -q '^nobody-unitree_ros-1$'; then
  echo "Fermo il container Nav2 Humble (nobody-unitree_ros-1)..."
  docker stop nobody-unitree_ros-1 >/dev/null
fi

echo "Aspetto la chiusura dei nodi (8 s)..."
sleep 8
tmux kill-session -t "$SESSION"

# Eventuali nodi rimasti appesi
pkill -f "ros2 launch go2_" 2>/dev/null || true
pkill -f "patrol_mission_launch" 2>/dev/null || true
pkill -f "yolo_detector.py" 2>/dev/null || true
source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && ros2 daemon stop >/dev/null 2>&1 || true
echo "Pipeline fermata."
