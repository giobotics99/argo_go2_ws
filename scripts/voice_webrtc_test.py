#!/usr/bin/env python3
"""
Prova isolata: fa dire una frase al Go2 tramite WebRTC (come l'app Unitree).

Uso (dentro venv-voice):
    source ~/argo_go2_ws/venv-voice/bin/activate
    python ~/argo_go2_ws/scripts/voice_webrtc_test.py                  # metodo megaphone
    python ~/argo_go2_ws/scripts/voice_webrtc_test.py --method track   # streaming come traccia audio
    python ~/argo_go2_ws/scripts/voice_webrtc_test.py --text "Ciao, sono Argo"

Variabili d'ambiente:
    GO2_IP       IP del robot sulla rete interna (default 192.168.123.161)
    GO2_AES_KEY  chiave AES-128 del robot (32 caratteri esadecimali): serve solo con firmware Go2 >= 1.1.15

IMPORTANTE: il Go2 accetta UNA sola connessione WebRTC. Chiudi l'app Unitree sul telefono prima della prova.
"""
import argparse
import asyncio
import audioop
import logging
import os
import subprocess
import sys
import tempfile
import types
import wave

# sounddevice serve alla libreria solo per riprodurre in locale l'audio ricevuto dal robot:
# senza la libreria di sistema portaudio fallisce all'import, quindi lo sostituiamo con un modulo vuoto.
try:
    import sounddevice  # noqa: F401
except Exception:
    sys.modules['sounddevice'] = types.ModuleType('sounddevice')

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod  # noqa: E402
from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub  # noqa: E402

HOME = os.path.expanduser('~')
PIPER_DIR = os.path.join(HOME, 'piper', 'piper')
VOICE = os.path.join(HOME, 'piper', 'voices', 'it_IT-paola-medium.onnx')


def synthesize(text, out_dir):
    """Piper -> WAV 22.05 kHz, poi conversione a 44.1 kHz mono 16 bit (formato usato dall'app)."""
    raw = os.path.join(out_dir, 'piper.wav')
    env = dict(os.environ, LD_LIBRARY_PATH=PIPER_DIR + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
    env.pop('LD_PRELOAD', None)
    subprocess.run([os.path.join(PIPER_DIR, 'piper'), '--model', VOICE, '--output_file', raw, '--quiet'],
                   input=text, text=True, env=env, check=True)
    with wave.open(raw, 'rb') as w:
        sw, rate, frames = w.getsampwidth(), w.getframerate(), w.readframes(w.getnframes())
    frames, _ = audioop.ratecv(frames, sw, 1, rate, 44100, None)
    out = os.path.join(out_dir, 'argo_voice.wav')
    with wave.open(out, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(44100)
        w.writeframes(frames)
    return out, len(frames) / 2 / 44100.0


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--text', default="Ciao, sono Argo. Mi senti? Questa è una prova della mia voce.")
    ap.add_argument('--method', choices=['megaphone', 'track'], default='megaphone')
    args = ap.parse_args()

    ip = os.environ.get('GO2_IP', '192.168.123.161')
    key = os.environ.get('GO2_AES_KEY') or None

    tmp = tempfile.mkdtemp(prefix='argo_voice_test_')
    wav, duration = synthesize(args.text, tmp)
    print(f"[1/3] Frase sintetizzata: {duration:.1f} s di audio ({wav})")

    print(f"[2/3] Connessione WebRTC a {ip} {'con' if key else 'senza'} chiave AES ...")
    kwargs = {'ip': ip}
    if key:
        kwargs['aes_128_key'] = key
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, **kwargs)
    await conn.connect()
    print("      connesso.")

    print(f"[3/3] Riproduzione con metodo '{args.method}' — ascolta il robot!")
    if args.method == 'megaphone':
        hub = WebRTCAudioHub(conn, logging.getLogger('audiohub'))
        await hub.enter_megaphone()
        await asyncio.sleep(0.3)
        await hub.upload_megaphone(wav)
        await asyncio.sleep(duration + 1.0)
        await hub.exit_megaphone()
    else:
        from aiortc.contrib.media import MediaPlayer
        player = MediaPlayer(wav)
        conn.pc.addTrack(player.audio)
        await asyncio.sleep(duration + 2.0)

    await conn.disconnect()
    print("Fatto. Hai sentito la frase?")


if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
