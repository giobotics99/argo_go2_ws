# Argo GO2 environment — source this instead of answering the foxy/noetic prompt.
#   source ~/argo_go2_ws/scripts/argo_env.sh
# Also used as --rcfile for every tmux window opened by start_pipeline.sh.

[ -f /etc/bash.bashrc ] && source /etc/bash.bashrc

ARGO_WS="$HOME/argo_go2_ws"
ARGO_SCRIPTS="$ARGO_WS/scripts"

# --- Leave any Python venv: the pipeline needs system python (/usr/bin/python3) ---
if [ -n "$VIRTUAL_ENV" ]; then
  deactivate 2>/dev/null || true
fi
unset VIRTUAL_ENV PYENV_VERSION
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -e 'venv-argo' -e '\.pyenv' | paste -sd: -)
export PATH

# --- CUDA (same as ~/.bashrc) ---
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:/usr/local/lib:$LD_LIBRARY_PATH
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1

# --- ROS 2 Foxy + CycloneDDS on the robot network (eth1) ---
source /opt/ros/foxy/setup.bash
[ -f "$HOME/cyclonedds_ws/install/setup.bash" ] && source "$HOME/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Buffer UDP grande solo se il kernel lo permette (sudo sysctl --system dopo /etc/sysctl.d/60-cyclonedds.conf)
if [ "$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)" -ge 10485760 ]; then
  export CYCLONEDDS_URI="$ARGO_SCRIPTS/cyclonedds_go2_bigbuf.xml"
else
  export CYCLONEDDS_URI="$ARGO_SCRIPTS/cyclonedds_go2.xml"
  echo "[argo_env] ATTENZIONE: buffer UDP piccoli (rmem_max=$(cat /proc/sys/net/core/rmem_max)). Lancia: sudo sysctl --system"
fi

# --- Workspace ---
if [ -f "$ARGO_WS/install/setup.bash" ]; then
  source "$ARGO_WS/install/setup.bash"
else
  echo "[argo_env] ATTENZIONE: $ARGO_WS/install/setup.bash non trovato — fai colcon build"
fi
cd "$ARGO_WS"

PS1='\[\e[1;32m\](argo)\[\e[0m\] \u@\h:\w\$ '

# ---------------- Wait helpers (used by start_pipeline.sh) ----------------

wait_gemma() {
  echo "[argo] Aspetto Gemma su http://127.0.0.1:8000 ..."
  until curl -sf http://127.0.0.1:8000/v1/models >/dev/null 2>&1; do sleep 2; done
  echo "[argo] Gemma pronto."
}

wait_topic() {  # wait_topic <topic> [type]
  echo "[argo] Aspetto messaggi su $1 ..."
  until timeout 5 ros2 topic echo "$1" ${2:+$2} 2>/dev/null | grep -q .; do
    echo "[argo]   ...ancora niente su $1"
  done
  echo "[argo] $1 OK."
}

wait_tf() {  # wait_tf <parent> <child>
  echo "[argo] Aspetto TF $1 -> $2 ..."
  until timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q Translation; do
    echo "[argo]   ...TF $1 -> $2 non ancora disponibile"
  done
  echo "[argo] TF $1 -> $2 OK."
}

wait_nav2() {
  echo "[argo] Aspetto che Nav2 (bt_navigator) sia active ..."
  until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; do sleep 2; done
  echo "[argo] Nav2 attivo."
}

wait_plansys2() {
  echo "[argo] Aspetto il servizio /gemma_interface/send_command ..."
  until ros2 service list 2>/dev/null | grep -q /gemma_interface/send_command; do sleep 2; done
  echo "[argo] Gemma bridge pronto."
}

# Send a natural-language command:  argo_cmd "Vai a controllare lab_robotics e poi passa da openspace"
argo_cmd() {
  # Stringa YAML tra doppi apici: gli apostrofi (l'openspace) vanno bene; escape di \ e "
  local txt
  txt=$(printf '%s' "$*" | sed -e 's/\\/\\\\/g' -e 's/"/\\"/g')
  ros2 service call /gemma_interface/send_command go2_interfaces/srv/GemmaCommand "{command: \"$txt\"}"
}

# Guida: luoghi che il cane conosce + esempi di comandi
argo_help() {
  /usr/bin/python3 - "$ARGO_WS/src/go2_patrol_mission/config/waypoints.yaml" <<'PY'
import sys, yaml
names = {'lab_robotics': 'Laboratorio di robotica', 'corridor': 'Corridoio', 'openspace': 'Openspace'}
alias = {'lab_robotics': 'lab, laboratorio, robotics lab',
         'corridor': 'corridoio, hallway',
         'openspace': 'open space, area aperta, ufficio condiviso'}
wp = yaml.safe_load(open(sys.argv[1]))['waypoints']
print("\n\033[1m=== ARGO GO2 — luoghi che conosco ===\033[0m")
for k, v in wp.items():
    print(f"  \033[1;32m{names.get(k, k):24s}\033[0m id: {k:13s} mappa: x={v['x']:5.1f}  y={v['y']:4.1f}")
    print(f"  {'':24s} puoi dire: {alias.get(k, '-')}")
print("\n  Percorsi possibili:  laboratorio <-> corridoio <-> openspace")
print("  Qualsiasi altro luogo (cucina, bagno, ufficio del prof...) viene RIFIUTATO con spiegazione.")
print("\n\033[1mEsempi:\033[0m")
print('  argo_cmd "vai a controllare l\'openspace"')
print('  argo_cmd "passa dal corridoio e poi ispeziona il laboratorio"')
print('  argo_cmd "fai un giro: corridoio, poi lab, poi openspace"')
print("\n  Un nuovo comando ANNULLA quello in corso.  Guida: argo_help\n")
PY
}

# Diario della missione: messaggi di Gemma/bridge con orario (Ctrl+C per uscire)
argo_explain() {
  /usr/bin/python3 - <<'PY'
import rclpy, datetime
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node('argo_explain_viewer')
def cb(m):
    t = datetime.datetime.now().strftime('%H:%M:%S')
    color = '\033[1;31m' if 'RIFIUTATO' in m.data or 'fallit' in m.data or 'errore' in m.data else \
            '\033[1;32m' if 'ACCETTATO' in m.data or 'completat' in m.data else '\033[0m'
    print(f"\033[90m[{t}]\033[0m {color}{m.data}\033[0m", flush=True)
n.create_subscription(String, '/supervisor/explanation', cb, 10)
print("In ascolto su /supervisor/explanation ...", flush=True)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
PY
}

# Registra lo schermo in ~/Videos (Ctrl+C o q per fermare). Leggero: 15 fps, x264 ultrafast.
#   argo_rec                -> schermo intero
#   argo_rec test_corridoio -> nome file personalizzato
argo_rec() {
  local name="${1:-argo_$(date +%Y%m%d_%H%M%S)}"
  local out="$HOME/Videos/${name}.mp4"
  local disp="${DISPLAY:-:1}"
  local size
  size=$(xdpyinfo -display "$disp" 2>/dev/null | awk '/dimensions:/{print $2}')
  mkdir -p "$HOME/Videos"
  echo "[argo] Registro $disp (${size:-1920x1080}) in $out — premi q o Ctrl+C per fermare"
  ffmpeg -loglevel warning -stats -f x11grab -framerate 15 -video_size "${size:-1920x1080}" -i "$disp" \
    -c:v libx264 -preset ultrafast -crf 28 -pix_fmt yuv420p "$out"
  echo "[argo] Salvato: $out"
}
