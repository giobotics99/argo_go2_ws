#!/usr/bin/env bash
# Crea ~/argo_go2_ws/venv-voice con la libreria WebRTC per far parlare il Go2.
# Separato dal Python di sistema (ROS) e da venv-argo. Da lanciare una volta sola.
#
#   ~/argo_go2_ws/scripts/setup_voice_venv.sh
#   source ~/argo_go2_ws/venv-voice/bin/activate     # per usarlo
set -e

VENV="$HOME/argo_go2_ws/venv-voice"
LIB="$HOME/go2_webrtc_connect"     # https://github.com/legion1581/go2_webrtc_connect (già clonato)

if [ ! -d "$LIB" ]; then
  git clone --depth 1 https://github.com/legion1581/go2_webrtc_connect.git "$LIB"
fi

# Python 3.8 di sistema: stessa versione di ROS Foxy, così il nodo voce potrà importare anche rclpy
/usr/bin/python3 -m venv "$VENV"
touch "$VENV/COLCON_IGNORE"   # colcon non deve esplorare il venv
source "$VENV/bin/activate"
pip install --upgrade pip wheel

# Dipendenze della libreria, SENZA pyaudio e opencv (servono solo a microfono/video, non alla voce):
# pyaudio richiederebbe 'sudo apt install portaudio19-dev'.
pip install "aiortc>=1.9.0" pycryptodome requests curl_cffi wasmtime lz4 numpy packaging pydub

# La libreria stessa, senza ritirare le dipendenze escluse sopra
pip install --no-deps -e "$LIB"

python - <<'PY'
import sys, types
sys.modules.setdefault('sounddevice', types.ModuleType('sounddevice'))
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection
print("OK: libreria WebRTC importata correttamente")
PY
echo
echo "Fatto. Per usarlo:  source $VENV/bin/activate"
