#!/usr/bin/env python3
"""
argo_voice — fa parlare il Go2 con le spiegazioni della missione.

Ascolta /supervisor/explanation (diario della missione / spiegazioni di Gemma) e /argo/say
(per prove manuali), sintetizza la frase con Piper (voce italiana, offline sul Jetson) e la
riproduce sullo speaker integrato del Go2 tramite l'API "megaphone" dell'audiohub:

    /api/audiohub/request  (unitree_api/Request)
        4001 ENTER_MEGAPHONE
        4003 UPLOAD_MEGAPHONE   blocchi base64 da 4096 caratteri di un WAV 44.1 kHz mono 16 bit
        4002 EXIT_MEGAPHONE

Le frasi vengono messe in coda e dette una alla volta; se la coda si allunga troppo
le più vecchie vengono scartate, così il cane non resta indietro rispetto alla missione.
"""
import audioop
import base64
import json
import os
import queue
import re
import subprocess
import tempfile
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request

API_ENTER_MEGAPHONE = 4001
API_EXIT_MEGAPHONE = 4002
API_UPLOAD_MEGAPHONE = 4003

# Nomi da leggere a voce al posto degli ID PDDL
SPOKEN_NAMES = {
    'lab_robotics': 'laboratorio di robotica',
    'corridor': 'corridoio',
    'openspace': 'open space',
    'start_pos': 'posizione di partenza',
}


def to_speech(text):
    """Rende un messaggio del diario adatto a essere detto a voce (None = non dirlo)."""
    t = text.strip()
    if not t:
        return None
    # Rifiuto: diciamo solo la spiegazione di Gemma (o il motivo), non l'elenco tecnico finale
    if t.startswith('Goal RIFIUTATO'):
        t = t.replace('Goal RIFIUTATO.', '', 1).strip()
        t = t.split('Conosco solo:')[0].strip()
        t = 'Non posso farlo. ' + re.sub(r'^Gemma:\s*', '', t)
    # Accettato: "Goal ACCETTATO: corridoio. Gemma: ... Piano avviato." -> frase breve
    elif t.startswith('Goal ACCETTATO'):
        rooms = t.split(':', 1)[1].split('.', 1)[0].strip()
        t = f'Ricevuto. Vado a ispezionare: {rooms}.'
    t = re.sub(r'\([^)]*\)', '', t)               # niente "(lab_robotics)" ecc.
    for room_id, name in SPOKEN_NAMES.items():
        t = re.sub(rf'\b{room_id}\b', name, t)
    t = t.replace('_', ' ')
    t = re.sub(r'\s+', ' ', t).strip()
    return t or None


class ArgoVoice(Node):
    def __init__(self):
        super().__init__('argo_voice')
        home = os.path.expanduser('~')
        self.declare_parameter('piper_dir', os.path.join(home, 'piper', 'piper'))
        self.declare_parameter('voice_model', os.path.join(home, 'piper', 'voices', 'it_IT-paola-medium.onnx'))
        self.declare_parameter('length_scale', 1.0)    # >1 parla più lento
        self.declare_parameter('max_queue', 3)         # frasi in attesa oltre le quali si scartano le vecchie
        self.declare_parameter('chunk_gap', 0.02)      # pausa (s) tra un blocco audio e il successivo
        self.declare_parameter('enabled', True)

        self.piper_dir = self.get_parameter('piper_dir').value
        self.voice_model = self.get_parameter('voice_model').value
        self.length_scale = float(self.get_parameter('length_scale').value)
        self.max_queue = int(self.get_parameter('max_queue').value)
        self.chunk_gap = float(self.get_parameter('chunk_gap').value)

        self.pub = self.create_publisher(Request, '/api/audiohub/request', 50)
        self.create_subscription(String, '/supervisor/explanation', self.on_text, 10)
        self.create_subscription(String, '/argo/say', self.on_text, 10)

        self.q = queue.Queue()
        self.last_text = None
        self.tmpdir = tempfile.mkdtemp(prefix='argo_voice_')
        self.piper = self.start_piper()
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info("Argo voice pronto: ascolto /supervisor/explanation e /argo/say")

    # ------------------------------------------------------------------ Piper
    def start_piper(self):
        """Piper resta acceso: il modello si carica una volta sola (~2 s) e poi ogni frase è più rapida."""
        env = dict(os.environ)
        env['LD_LIBRARY_PATH'] = self.piper_dir + ':' + env.get('LD_LIBRARY_PATH', '')
        env.pop('LD_PRELOAD', None)
        cmd = [os.path.join(self.piper_dir, 'piper'), '--model', self.voice_model,
               '--output_dir', self.tmpdir, '--length_scale', str(self.length_scale), '--quiet']
        self.get_logger().info(f"Avvio Piper: {' '.join(cmd)}")
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, env=env, text=True, bufsize=1)

    def synthesize(self, text):
        if self.piper.poll() is not None:
            self.get_logger().warn("Piper si è chiuso: lo riavvio")
            self.piper = self.start_piper()
        self.piper.stdin.write(text.replace('\n', ' ') + '\n')
        self.piper.stdin.flush()
        path = self.piper.stdout.readline().strip()   # Piper stampa il percorso del WAV generato
        return path if path and os.path.exists(path) else None

    @staticmethod
    def to_go2_wav(src):
        """Converte il WAV di Piper (22.05 kHz) nel formato usato dallo speaker del Go2: 44.1 kHz mono 16 bit."""
        with wave.open(src, 'rb') as w:
            ch, sw, rate, frames = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.readframes(w.getnframes())
        if ch == 2:
            frames = audioop.tomono(frames, sw, 0.5, 0.5)
        if sw != 2:
            frames = audioop.lin2lin(frames, sw, 2)
        if rate != 44100:
            frames, _ = audioop.ratecv(frames, 2, 1, rate, 44100, None)
        dst = src[:-4] + '_44k.wav'
        with wave.open(dst, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(44100)
            w.writeframes(frames)
        return dst, len(frames) / 2 / 44100.0

    # ------------------------------------------------------------------ Go2 speaker
    def send(self, api_id, param):
        r = Request()
        r.header.identity.id = int(time.time() * 1e6) % (2 ** 31)
        r.header.identity.api_id = api_id
        r.parameter = json.dumps(param)
        self.pub.publish(r)

    def play_on_go2(self, wav_path, duration):
        b64 = base64.b64encode(open(wav_path, 'rb').read()).decode('ascii')
        chunks = [b64[i:i + 4096] for i in range(0, len(b64), 4096)]
        self.send(API_ENTER_MEGAPHONE, {})
        time.sleep(0.3)
        for i, c in enumerate(chunks, 1):
            self.send(API_UPLOAD_MEGAPHONE, {'current_block_size': len(c), 'block_content': c,
                                             'current_block_index': i, 'total_block_number': len(chunks)})
            time.sleep(self.chunk_gap)
        time.sleep(duration + 0.8)      # lascia finire la riproduzione prima di uscire dal megafono
        self.send(API_EXIT_MEGAPHONE, {})

    # ------------------------------------------------------------------ coda
    def on_text(self, msg):
        if not self.get_parameter('enabled').value:
            return
        text = to_speech(msg.data)
        if not text or text == self.last_text:
            return
        self.last_text = text
        while self.q.qsize() >= self.max_queue:
            try:
                dropped = self.q.get_nowait()
                self.get_logger().warn(f"Coda piena, salto: {dropped}")
            except queue.Empty:
                break
        self.q.put(text)

    def worker(self):
        while rclpy.ok():
            text = self.q.get()
            try:
                t0 = time.time()
                raw = self.synthesize(text)
                if raw is None:
                    self.get_logger().error(f"Sintesi fallita: {text}")
                    continue
                wav, duration = self.to_go2_wav(raw)
                self.get_logger().info(f"[VOCE] ({time.time() - t0:.1f}s sintesi, {duration:.1f}s audio) {text}")
                self.play_on_go2(wav, duration)
                for p in (raw, wav):
                    try:
                        os.remove(p)
                    except OSError:
                        pass
            except Exception as e:
                self.get_logger().error(f"Errore voce: {e}")

    def destroy_node(self):
        try:
            self.send(API_EXIT_MEGAPHONE, {})
            self.piper.kill()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ArgoVoice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
