#!/usr/bin/env python3
"""
go2_speaker_worker — fa parlare il Go2 tramite WebRTC (processo SENZA ROS, Python di venv-voice).

Due modalità:
  --serve   (usata da argo_voice_node.py) si collega UNA volta all'avvio e tiene la connessione
            aperta: le frasi arrivano su stdin (una per riga) e partono subito, senza nuovi handshake.
            Serve perché durante la navigazione (robot in moto, CPU carica) un nuovo handshake
            WebRTC spesso non si completa. Su stdout: READY / OK / WARN / ERROR.
  --text T  dice una sola frase ed esce (prove manuali):
            ~/argo_go2_ws/venv-voice/bin/python go2_speaker_worker.py --text "Ciao, sono Argo"

Nota sulla modalità --text:

Sequenza (identica a scripts/voice_webrtc_test.py --method track, verificata funzionante):
  1. Piper -> PCM 48 kHz mono in memoria, con 1.5 s di silenzio iniziale (lo speaker del Go2 impiega
     circa un secondo ad attivarsi: senza il silenzio l'inizio della frase si perde)
  2. connessione WebRTC al Go2 (libreria legion1581/go2_webrtc_connect, come l'app Unitree)
  3. il PCM viene inviato come traccia audio (frame da 20 ms già pronti, temporizzati con precisione)
  4. disconnessione

Gira SENZA ROS (processo separato dal nodo ROS) e con il Python di venv-voice.
Ultima riga su stdout: "OK ..." oppure "ERROR ..." (letta da argo_voice_node). Exit code 0 = frase detta.

Variabili d'ambiente: GO2_IP (default 192.168.123.161), GO2_AES_KEY (firmware Go2 >= 1.1.15).
"""
import argparse
import asyncio
import threading
import audioop
import fractions
import time
import logging
import os
import subprocess
import sys
import tempfile
import types
import wave
import warnings

warnings.filterwarnings('ignore')
# sounddevice serve alla libreria solo per riprodurre in locale l'audio ricevuto dal robot
try:
    import sounddevice  # noqa: F401
except Exception:
    sys.modules['sounddevice'] = types.ModuleType('sounddevice')

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod  # noqa: E402
# NB: `av` e aiortc.mediastreams vanno importati SOLO DOPO la connessione (vedi make_pcm_track):
# importandoli all'inizio l'handshake DTLS con il Go2 non si completa mai (verificato più volte).

SAMPLE_RATE = 48000                 # frequenza nativa di Opus: niente ricampionamento dentro aiortc
FRAME_SAMPLES = 960                 # 20 ms per pacchetto
FRAME_BYTES = FRAME_SAMPLES * 2     # s16 mono


HOME = os.path.expanduser('~')
PIPER_DIR = os.environ.get('ARGO_PIPER_DIR', os.path.join(HOME, 'piper', 'piper'))
VOICE = os.environ.get('ARGO_PIPER_VOICE', os.path.join(HOME, 'piper', 'voices', 'it_IT-paola-medium.onnx'))
LEAD_SILENCE = 1.5     # s di silenzio prima della voce (connessione appena aperta: lo speaker si sta attivando)
SERVE_LEAD_SILENCE = 0.6   # con la connessione già aperta basta meno
TAIL_SILENCE = 0.5     # s di silenzio dopo
RETRY_WAIT = 20.0      # s tra un tentativo di connessione e l'altro (modalità --serve)


def report(level, msg=''):
    """Riga di stato per argo_voice_node."""
    print(f"{level} {msg}".rstrip(), flush=True)


def synthesize(text, out_dir, lead=LEAD_SILENCE):
    """Piper -> PCM s16 mono 48 kHz in memoria, con silenzio iniziale/finale. Ritorna (pcm, durata)."""
    raw = os.path.join(out_dir, 'piper.wav')
    env = dict(os.environ, LD_LIBRARY_PATH=PIPER_DIR + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
    env.pop('LD_PRELOAD', None)
    # Piper usa tutti i core per ~2 s a frase: lo confiniamo su 2 core a bassa priorità
    # (+0.3 s di sintesi) per non togliere CPU a Nav2, LiDAR e driver. ARGO_VOICE_CPUS="" = nessun limite.
    cpus = os.environ.get('ARGO_VOICE_CPUS', '6,7')
    prefix = ['nice', '-n', '10'] + (['taskset', '-c', cpus] if cpus else [])
    subprocess.run(prefix + [os.path.join(PIPER_DIR, 'piper'), '--model', VOICE, '--output_file', raw, '--quiet'],
                   input=text, text=True, env=env, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with wave.open(raw, 'rb') as w:
        ch, sw, rate, frames = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.readframes(w.getnframes())
    if ch == 2:
        frames = audioop.tomono(frames, sw, 0.5, 0.5)
    if sw != 2:
        frames = audioop.lin2lin(frames, sw, 2)
    if rate != SAMPLE_RATE:
        frames, _ = audioop.ratecv(frames, 2, 1, rate, SAMPLE_RATE, None)
    pcm = bytes(int(lead * SAMPLE_RATE) * 2) + frames + bytes(int(TAIL_SILENCE * SAMPLE_RATE) * 2)
    return pcm, len(pcm) / 2 / SAMPLE_RATE


def make_pcm_track(pcm=b''):
    """Crea la traccia audio. Gli import stanno qui dentro di proposito: vedi nota sopra."""
    from av import AudioFrame
    from aiortc.mediastreams import MediaStreamError, MediaStreamTrack

    class PcmTrack(MediaStreamTrack):
        """Traccia audio continua: silenzio quando non c'è niente da dire, il PCM in coda quando c'è.
        Frame da 20 ms già pronti, inviati con temporizzazione precisa (più leggera di MediaPlayer)."""
        kind = 'audio'

        def __init__(self, pcm):
            super().__init__()
            self._buf = bytearray(pcm)
            self._silence = bytes(FRAME_BYTES)
            self._i = 0
            self._start = None

        def push(self, pcm):
            self._buf.extend(pcm)

        def remaining(self):
            """Secondi di audio ancora da inviare."""
            return len(self._buf) / 2 / SAMPLE_RATE

        async def recv(self):
            if self.readyState != 'live':
                raise MediaStreamError
            if self._start is None:
                self._start = time.monotonic()
            else:
                wait = self._start + self._i * FRAME_SAMPLES / SAMPLE_RATE - time.monotonic()
                if wait > 0:
                    await asyncio.sleep(wait)
                elif wait < -0.5:
                    # siamo rimasti indietro (CPU carica): riallinea l'orologio invece di "recuperare" a raffica
                    self._start = time.monotonic() - self._i * FRAME_SAMPLES / SAMPLE_RATE
            if self._buf:
                data = bytes(self._buf[:FRAME_BYTES]).ljust(FRAME_BYTES, b'\0')
                del self._buf[:FRAME_BYTES]
            else:
                data = self._silence
            frame = AudioFrame(format='s16', layout='mono', samples=FRAME_SAMPLES)
            frame.planes[0].update(data)
            frame.pts = self._i * FRAME_SAMPLES
            frame.sample_rate = SAMPLE_RATE
            frame.time_base = fractions.Fraction(1, SAMPLE_RATE)
            self._i += 1
            return frame

    return PcmTrack(pcm)


def describe_ice(conn):
    """Coppia di candidati ICE scelta (per capire su quale rete passa la connessione)."""
    try:
        for t in conn.pc.getTransceivers():
            ice = t.sender.transport.transport
            c = ice._connection
            pair = next(iter(c._nominated.values()), None)
            locs = sorted({x.host for x in c.local_candidates})
            if pair is not None:
                return f"coppia {pair.local_candidate.host} -> {pair.remote_candidate.host}; locali {locs}"
            return f"nessuna coppia nominata; locali {locs}"
    except Exception as e:
        return f"n/d ({e})"
    return "n/d"


async def main(text, pcm, duration):
    ip = os.environ.get('GO2_IP', '192.168.123.161')
    key = os.environ.get('GO2_AES_KEY') or None
    kwargs = {'ip': ip}
    if key:
        kwargs['aes_128_key'] = key
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, **kwargs)
    try:
        try:
            await conn.connect()
        finally:
            print(f"ICE: {describe_ice(conn)}", file=sys.stderr, flush=True)
        conn.pc.addTrack(make_pcm_track(pcm))
        await asyncio.sleep(duration + 1.0)
    finally:
        # Chiudere SEMPRE la connessione, anche se l'handshake è fallito: una sessione lasciata
        # appesa fa rifiutare al Go2 i tentativi successivi per un po' di tempo.
        try:
            await conn.disconnect()
        except Exception:
            pass


def new_connection():
    ip = os.environ.get('GO2_IP', '192.168.123.161')
    key = os.environ.get('GO2_AES_KEY') or None
    kwargs = {'ip': ip}
    if key:
        kwargs['aes_128_key'] = key
    return UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, **kwargs)


async def serve():
    """Connessione permanente: si collega all'avvio e resta collegata; le frasi arrivano da stdin."""
    loop = asyncio.get_running_loop()
    tmp = tempfile.mkdtemp(prefix='argo_speaker_')
    lines = asyncio.Queue()

    def stdin_reader():                       # thread: stdin -> coda asyncio (None = stdin chiuso)
        for line in sys.stdin:
            loop.call_soon_threadsafe(lines.put_nowait, line.strip())
        loop.call_soon_threadsafe(lines.put_nowait, None)
    threading.Thread(target=stdin_reader, daemon=True).start()

    state = {'conn': None, 'track': None}

    def healthy():
        c = state['conn']
        return c is not None and c.pc is not None and c.pc.connectionState == 'connected'

    async def ensure_connected():
        while not healthy():
            old = state['conn']
            if old is not None:
                report('WARN', 'Connessione WebRTC persa: mi ricollego')
                try:
                    await old.disconnect()
                except Exception:
                    pass
            state['conn'] = state['track'] = None
            conn = new_connection()
            try:
                await conn.connect()
            except Exception as e:
                report('WARN', f"Connessione al Go2 fallita ({str(e)[:110]}). Riprovo tra {RETRY_WAIT:.0f} s")
                try:
                    await conn.disconnect()
                except Exception:
                    pass
                await asyncio.sleep(RETRY_WAIT)
                continue
            track = make_pcm_track()
            conn.pc.addTrack(track)
            state['conn'], state['track'] = conn, track
            report('READY', 'collegato al Go2')

    try:
        await ensure_connected()
        while True:
            try:
                text = await asyncio.wait_for(lines.get(), 5.0)
            except asyncio.TimeoutError:
                if not healthy():
                    await ensure_connected()
                continue
            if text is None:              # il nodo ROS ha chiuso stdin: usciamo
                break
            if not text:
                continue
            try:
                pcm, duration = await loop.run_in_executor(None, synthesize, text, tmp, SERVE_LEAD_SILENCE)
                await ensure_connected()
                state['track'].push(pcm)
                await asyncio.sleep(state['track'].remaining() + 0.3)
                report('OK', f"{duration:.1f}s")
            except Exception as e:
                report('ERROR', str(e)[:200])
    finally:
        if state['conn'] is not None:
            try:
                await state['conn'].disconnect()
            except Exception:
                pass
        subprocess.run(['rm', '-rf', tmp])


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--text', help='dice una frase ed esce')
    g.add_argument('--serve', action='store_true', help='connessione permanente, frasi da stdin')
    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)
    if args.serve:
        try:
            asyncio.run(serve())
        except KeyboardInterrupt:
            pass
        sys.exit(0)
    tmp = tempfile.mkdtemp(prefix='argo_speaker_')
    try:
        pcm, duration = synthesize(args.text, tmp)
        asyncio.run(main(args.text, pcm, duration))
        print(f"OK {duration:.1f}s", flush=True)
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as e:
        print(f"ERROR {str(e)[:200]}", flush=True)
        sys.exit(1)
    finally:
        subprocess.run(['rm', '-rf', tmp])
