#!/usr/bin/env python3
"""
argo_voice_node — il Go2 legge ad alta voce le spiegazioni della missione.

Ascolta:
  /supervisor/explanation  (std_msgs/String)  diario della missione e spiegazioni di Gemma
  /argo/say                (std_msgs/String)  frasi per prove manuali

Il nodo prepara le frasi (toglie gli ID tecnici, accorcia i rifiuti, unisce le frasi ravvicinate)
e le passa a go2_speaker_worker.py --serve, un processo separato SENZA ROS che resta collegato al
Go2 via WebRTC per tutta la sessione (un nuovo handshake durante la navigazione spesso fallisce),
sintetizza ogni frase con Piper e la riproduce sullo speaker del cane.

Perché due processi: CycloneDDS (rclpy) usa OpenSSL 1.1, la libreria WebRTC usa OpenSSL 3; nello
stesso processo l'handshake WebRTC non si completa.

Uso:   ros2 run go2_hardware argo_voice_node.py
Il worker gira con il Python di ~/argo_go2_ws/venv-voice (scripts/setup_voice_venv.sh).

Variabili d'ambiente (passate anche al worker):
  GO2_IP (default 192.168.123.161), GO2_AES_KEY (solo firmware Go2 >= 1.1.15),
  ARGO_VOICE_VENV (default ~/argo_go2_ws/venv-voice)

NOTA: il Go2 accetta UNA sola connessione WebRTC: mentre il cane parla l'app Unitree non può
essere collegata (e se l'app è collegata il cane non riesce a parlare).
"""
import ctypes
import os
import queue
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def _die_with_parent():
    """Nel processo figlio: se il nodo muore (anche con Ctrl+C o kill), il kernel termina il worker.
    Evita worker orfani che tengono occupata l'unica connessione WebRTC del Go2."""
    PR_SET_PDEATHSIG = 1
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, 15)   # 15 = SIGTERM


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
    if t.startswith('Goal RIFIUTATO'):
        # "Non so dove si trova: X. Conosco solo: ..." (gli ID tra parentesi vengono tolti sotto)
        t = t.replace('Goal RIFIUTATO.', '', 1).strip()
        t = 'Non posso farlo. ' + re.sub(r'^Gemma:\s*', '', t)
    elif t.startswith('Goal ACCETTATO'):
        rooms = t.split(':', 1)[1].split('.', 1)[0].strip()
        t = f'Ricevuto. Vado a ispezionare: {rooms}.'
    t = re.sub(r'\s*\([^)]*\)', '', t)                 # niente "(lab_robotics)" ecc.
    for room_id, name in SPOKEN_NAMES.items():
        t = re.sub(rf'\b{room_id}\b', name, t)
    t = t.replace('_', ' ')
    t = re.sub(r'\s+', ' ', t).strip()
    return t or None


class ArgoVoice(Node):
    def __init__(self):
        super().__init__('argo_voice')
        self.declare_parameter('enabled', True)
        self.declare_parameter('batch_window', 1.5)   # s: frasi arrivate in questo intervallo -> una sola frase
        self.declare_parameter('max_batch', 4)        # massimo di frasi unite (le più vecchie oltre si scartano)

        self.batch_window = float(self.get_parameter('batch_window').value)
        self.max_batch = int(self.get_parameter('max_batch').value)
        self.venv = os.environ.get('ARGO_VOICE_VENV', os.path.expanduser('~/argo_go2_ws/venv-voice'))
        self.worker_script = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'go2_speaker_worker.py')

        self.q = queue.Queue()
        self.last_text = None
        self.python = os.path.join(self.venv, 'bin', 'python')
        if not os.path.exists(self.python):
            self.get_logger().error(f"Manca {self.python}: crea il venv con ~/argo_go2_ws/scripts/setup_voice_venv.sh")

        self.worker = None
        self.connected = False
        self.replies = queue.Queue()
        if os.path.exists(self.python):
            self.start_worker()

        self.create_subscription(String, '/supervisor/explanation', self.on_text, 10)
        self.create_subscription(String, '/argo/say', self.on_text, 10)
        threading.Thread(target=self.dispatcher, daemon=True).start()
        self.get_logger().info("Argo voice pronto: ascolto /supervisor/explanation e /argo/say")

    # ------------------------------------------------------------ worker (connessione permanente)
    def start_worker(self):
        """go2_speaker_worker.py --serve: si collega al Go2 una volta e resta collegato."""
        env = dict(os.environ, PYTHONWARNINGS='ignore', PYTHONUNBUFFERED='1')
        self.worker = subprocess.Popen([self.python, self.worker_script, '--serve'],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                       env=env, text=True, bufsize=1, preexec_fn=_die_with_parent)
        threading.Thread(target=self.read_worker, args=(self.worker,), daemon=True).start()
        self.get_logger().info(f"Worker audio avviato (pid {self.worker.pid}): mi collego al Go2...")

    def read_worker(self, proc):
        """Righe di stato del worker -> log ROS (su Foxy: una chiamata al logger per livello)."""
        for line in proc.stdout:
            level, _, msg = line.strip().partition(' ')
            if level == 'READY':
                self.connected = True
                self.get_logger().info(f"Collegato al Go2: pronto a parlare ({msg})")
            elif level == 'OK':
                self.replies.put(('OK', msg))
            elif level == 'ERROR':
                self.get_logger().error(f"Worker audio: {msg}")
                self.replies.put(('ERROR', msg))
            elif level == 'WARN':
                self.connected = False
                self.get_logger().warn(msg)
            # le altre righe (stato della connessione stampato dalla libreria) si ignorano
        self.connected = False
        self.replies.put(('ERROR', 'worker terminato'))

    def speak(self, sentence):
        """Manda la frase al worker e aspetta che sia stata detta."""
        if self.worker is None or self.worker.poll() is not None:
            self.get_logger().warn("Worker audio non attivo: lo riavvio")
            self.start_worker()
        if not self.connected:
            self.get_logger().warn("Non ancora collegato al Go2: la frase partirà appena la connessione è pronta")
        while not self.replies.empty():          # svuota risposte vecchie
            self.replies.get_nowait()
        t0 = time.time()
        try:
            self.worker.stdin.write(sentence.replace('\n', ' ') + '\n')
            self.worker.stdin.flush()
        except (BrokenPipeError, OSError):
            self.get_logger().error("Impossibile comunicare con il worker audio")
            return False
        try:
            status, msg = self.replies.get(timeout=180)
        except queue.Empty:
            status, msg = 'ERROR', 'nessuna risposta dal worker'
        if status == 'OK':
            self.get_logger().info(f"[VOCE] detta in {time.time() - t0:.1f}s: {sentence}")
            return True
        self.get_logger().error(f"Frase non detta ({msg}): {sentence}")
        return False

    # ------------------------------------------------------------ frasi
    def on_text(self, msg):
        if not self.get_parameter('enabled').value:
            return
        text = to_speech(msg.data)
        if not text or text == self.last_text:
            return
        self.last_text = text
        self.q.put(text)

    def dispatcher(self):
        while rclpy.ok():
            texts = [self.q.get()]
            # raccogli le frasi arrivate subito dopo: si dicono tutte con una sola connessione
            deadline = time.time() + self.batch_window
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                try:
                    texts.append(self.q.get(timeout=remaining))
                except queue.Empty:
                    break
            while not self.q.empty():               # se siamo rimasti indietro prendiamo anche il resto
                texts.append(self.q.get_nowait())
            if len(texts) > self.max_batch:
                self.get_logger().warn(f"Troppe frasi in coda: salto le {len(texts) - self.max_batch} più vecchie")
                texts = texts[-self.max_batch:]
            sentence = ' '.join(t if t.endswith(('.', '!', '?')) else t + '.' for t in texts)
            self.speak(sentence)


def main():
    rclpy.init()
    node = ArgoVoice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.worker is not None and node.worker.poll() is None:
            try:
                node.worker.stdin.close()        # il worker si scollega dal Go2 ed esce
                node.worker.wait(timeout=5)
            except Exception:
                node.worker.kill()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
