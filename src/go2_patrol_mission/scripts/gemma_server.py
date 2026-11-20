#!/usr/bin/env python3
"""gemma_server — Argo's "head": language and judgement with Gemma, as ROS services (Argo v2).

Services (go2_interfaces/srv):
  /argo/gemma/formulate_goals   FormulateGoals    G1+G2  request -> mission, targets, message
  /argo/gemma/assess_failure    AssessFailure     G3     recovery strategy from a closed set
  /argo/gemma/appraise_findings AppraiseFindings  G4     NONE | REPORT after an inspection
  /argo/gemma/speak             Speak             G5     sentence for an event
  /argo/gemma/chat              Chat              G6     small talk at the coffee area

Rules (README_instruction.md, Phase 2):
  - every prompt is built from ontology.yaml: no room list in this file;
  - every answer is validated (IDs exist, closed sets, privacy: inspect only camera_allowed places);
  - on timeout / invalid JSON / invalid content a deterministic fallback answers (used_fallback=true);
  - speech: Italian, first person, at most 25 words, facts (numbers, names, times) never changed.
Gemma never produces actions or plans.

Usage: ros2 run go2_patrol_mission gemma_server.py [--ros-args -p llm_timeout:=10.0]
"""
import difflib
import json
import os
import re
import time
import unicodedata
import urllib.request

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from go2_interfaces.srv import AppraiseFindings, AssessFailure, Chat, FormulateGoals, Speak
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

MISSIONS = ('coffee_call', 'message', 'inspect', 'none')
STRATEGIES = ('WAIT_RETRY', 'ASK_CLEAR', 'DROP_UNREACHABLE', 'RETURN_HOME', 'ABORT')
VERDICTS = ('NONE', 'REPORT')
EVENTS = ('announce', 'message', 'inspection', 'failure', 'battery_low', 'sensor_fault', 'permission')
MAX_WORDS = 25
# Words that betray a non-Italian answer (Gemma sometimes drifts to Portuguese/Spanish/English)
NOT_ITALIAN = re.compile(r'\b(você|não|está|estou|usted|ustedes|estoy|también|the|and|you|have|will)\b|[¿¡ãõ]',
                         re.IGNORECASE)
# Corridor zones a person may name ("solo il corridoio nord"): word -> substring of the hub IDs
ZONE_WORDS = {'nord': 'north', 'north': 'north', 'ovest': 'west', 'west': 'west',
              'centro': 'center', 'centrale': 'center', 'incrocio': 'junction'}
# Italian words for the most common YOLO (COCO) classes, for spoken sentences
COCO_IT = {'person': ('persona', 'persone'), 'laptop': ('portatile', 'portatili'), 'chair': ('sedia', 'sedie'),
           'cup': ('tazza', 'tazze'), 'bottle': ('bottiglia', 'bottiglie'), 'tv': ('schermo', 'schermi'),
           'dining table': ('tavolo', 'tavoli'), 'book': ('libro', 'libri'), 'backpack': ('zaino', 'zaini'),
           'cell phone': ('telefono', 'telefoni'), 'keyboard': ('tastiera', 'tastiere')}
# G6: jokes are invented by Gemma (more creative temperature, a few attempts); CHAT_LINES is only the
# fallback when Gemma is off or keeps failing the checks.
CHAT_TEMPERATURE = 0.9
CHAT_ATTEMPTS = 3
CHAT_LINES = [
    'Perché il caffè non va mai in palestra? Perché è già abbastanza forte!',
    'Cosa dice una tazzina all\'altra? Senza di te la mia vita sarebbe amara!',
    'Qual è il caffè preferito dei programmatori? Il Java, ovviamente!',
    'Perché il cucchiaino è sempre stressato? Perché tutti lo fanno girare!',
    'Il barista mi ha chiesto: lo vuoi corto o lungo? Io ho risposto: a quattro zampe!',
    'Io e il caffè abbiamo una cosa in comune: senza la giusta carica non partiamo!',
    'Un caffè decaffeinato? È come un robot senza batteria: carino, ma non fa niente!',
    'Perché lo zucchero è timido? Perché si scioglie appena lo guardi!',
    'Sapete come si chiama un caffè che corre? Un espresso, naturalmente!',
    'Perché la tazza è andata dallo psicologo? Si sentiva sempre vuota dopo la pausa!',
    'Perché il caffè va sempre di fretta? Perché non vuole diventare freddo!',
]
# Generic small talk Gemma falls back to ("Ciao! Che bella giornata oggi"): never a coffee joke
GENERIC_LINE = re.compile(r'^\W*(ciao|salve|buongiorno|ehi)\b|bella giornata|stai bene|state bene|come stai|'
                          r'sono qui per|che bello veder|molto gentil|amico mio', re.IGNORECASE)
COFFEE_WORDS = re.compile(r'caffè|tazz|moka|espresso|barista|cialda|zucchero|cappuccin|macchinett', re.IGNORECASE)


def fold(text):
    """Lower case without accents, for matching names and keywords."""
    text = unicodedata.normalize('NFKD', str(text)).encode('ascii', 'ignore').decode()
    return re.sub(r'\s+', ' ', text.lower()).strip()


def words(text):
    return len(str(text).split())


def number_str(v):
    return str(int(v)) if isinstance(v, float) and v.is_integer() else str(v)


class Ontology:
    """Places, names and classes read from ontology.yaml."""

    def __init__(self, path):
        with open(path) as f:
            self.data = yaml.safe_load(f)
        self.places = self.data['places']
        self.hubs = self.data['hubs']
        self.classes = self.data['classes']
        self.home = self.data['home']
        self.social = [p for p, v in self.places.items() if v.get('social_spot')]
        # name (folded) -> set of place IDs; names shared by two places are ambiguous
        self.names = {}
        for pid, p in self.places.items():
            for name in [pid, pid.replace('_', ' ')] + list(p.get('synonyms') or []) + list(p.get('occupant') or []):
                self.names.setdefault(fold(name), set()).add(pid)

    def camera_allowed(self, pid):
        return bool(self.classes.get(self.places[pid]['class'], {}).get('camera_allowed', False))

    def works_here(self):
        return [p for p, v in self.places.items() if v.get('works_here')]

    def spoken(self, pid):
        """Italian name for a place (first synonym), e.g. 'laboratorio di robotica'."""
        syn = self.places.get(pid, {}).get('synonyms') or []
        return syn[0] if syn else pid.replace('_', ' ')

    def occupants(self, pid):
        return list(self.places.get(pid, {}).get('occupant') or [])

    def resolve(self, name):
        """Place ID for a name/ID/synonym/occupant, or None (unknown or ambiguous)."""
        ids = self.names.get(fold(name))
        return next(iter(ids)) if ids and len(ids) == 1 else None

    def zone(self, word):
        key = ZONE_WORDS.get(fold(word))
        if not key:
            return []
        # A room's entrance may be its door: the corridor zone is the door's entrance hub
        hub_of = lambda v: self.places.get(v.get('entrance', ''), {}).get('entrance', v.get('entrance', ''))
        return [p for p, v in self.places.items() if key in hub_of(v) and v.get('works_here')]

    def find_in_text(self, text):
        """Places named in a free text, in order of appearance (longest names first)."""
        t = ' ' + fold(text) + ' '
        found = []
        for name in sorted(self.names, key=len, reverse=True):
            ids = self.names[name]
            for m in re.finditer(r'(?<![a-z0-9])' + re.escape(name) + r'(?![a-z0-9])', t):
                if len(ids) == 1:
                    found.append((m.start(), next(iter(ids))))
                t = t[:m.start()] + '#' * len(name) + t[m.end():]   # do not match it again
        found.sort()
        out = []
        for _, pid in found:
            if pid not in out:
                out.append(pid)
        return out

    def prompt_table(self):
        lines = []
        for pid, p in self.places.items():
            lines.append(
                f"- {pid} | class {p['class']} | people: {', '.join(self.occupants(pid)) or '-'} | "
                f"also called: {', '.join(p.get('synonyms') or []) or '-'} | entrance {p.get('entrance')} | "
                f"camera_allowed {'yes' if self.camera_allowed(pid) else 'no'} | "
                f"works_here {'yes' if p.get('works_here') else 'no'}")
        return '\n'.join(lines)


class GemmaServer(Node):

    def __init__(self):
        super().__init__('gemma_server')
        default_onto = os.path.join(get_package_share_directory('go2_patrol_mission'), 'config', 'ontology.yaml')
        self.declare_parameter('ontology_file', default_onto)
        self.declare_parameter('gemma_api_base', 'http://127.0.0.1:8000/v1')
        self.declare_parameter('gemma_model_id', 'google/gemma-4-E2B-it')
        self.declare_parameter('llm_timeout', 10.0)     # s, then fallback
        self.declare_parameter('temperature', 0.3)
        self.onto = Ontology(self.get_parameter('ontology_file').value)
        self.api = self.get_parameter('gemma_api_base').value
        self.model = self.get_parameter('gemma_model_id').value
        self.timeout = float(self.get_parameter('llm_timeout').value)
        self.temperature = float(self.get_parameter('temperature').value)
        self.chat_index = 0

        group = ReentrantCallbackGroup()   # services can be served in parallel
        self.create_service(FormulateGoals, '/argo/gemma/formulate_goals', self.formulate_goals, callback_group=group)
        self.create_service(AssessFailure, '/argo/gemma/assess_failure', self.assess_failure, callback_group=group)
        self.create_service(AppraiseFindings, '/argo/gemma/appraise_findings', self.appraise_findings,
                            callback_group=group)
        self.create_service(Speak, '/argo/gemma/speak', self.speak, callback_group=group)
        self.create_service(Chat, '/argo/gemma/chat', self.chat, callback_group=group)
        self.get_logger().info(
            f"gemma_server pronto: {len(self.onto.places)} luoghi da {self.get_parameter('ontology_file').value}, "
            f"timeout LLM {self.timeout:.0f} s")

    # ------------------------------------------------------------------ llama.cpp
    def llm_text(self, system_prompt, user_prompt, max_tokens, temperature=None):
        """Raw text from Gemma, or None on timeout / error (same call as gemma_bridge.py)."""
        payload = {'model': self.model, 'stream': False, 'max_tokens': max_tokens,
                   'temperature': self.temperature if temperature is None else temperature,
                   'messages': [{'role': 'system', 'content': system_prompt},
                                {'role': 'user', 'content': user_prompt}]}
        req = urllib.request.Request(f'{self.api}/chat/completions', data=json.dumps(payload).encode('utf-8'),
                                     headers={'Content-Type': 'application/json'}, method='POST')
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # never use a proxy for localhost
        try:
            with opener.open(req, timeout=self.timeout) as resp:
                data = json.loads(resp.read().decode('utf-8'))
            return data.get('choices', [{}])[0].get('message', {}).get('content', '').strip() or None
        except Exception as e:  # noqa: BLE001 - any failure means "use the fallback"
            self.get_logger().warn(f'[LLM] nessuna risposta valida da Gemma: {e}')
            return None

    def llm_json(self, system_prompt, user_prompt, max_tokens=200, temperature=None):
        """Dict parsed from Gemma's answer (tolerates ``` fences and extra prose), or None."""
        text = self.llm_text(system_prompt, user_prompt, max_tokens, temperature)
        if not text:
            return None
        start, end = text.find('{'), text.rfind('}')
        if start == -1 or end <= start:
            self.get_logger().warn(f'[LLM] risposta senza JSON: {text[:120]}')
            return None
        try:
            parsed = json.loads(text[start:end + 1])
        except json.JSONDecodeError:
            self.get_logger().warn(f'[LLM] JSON non valido: {text[:120]}')
            return None
        return parsed if isinstance(parsed, dict) else None

    def good_sentence(self, sentence, facts=()):
        """Italian, <= 25 words, non empty, and every fact appears unchanged."""
        s = str(sentence or '').strip().strip('"')
        if not s or words(s) > MAX_WORDS or NOT_ITALIAN.search(s):
            return None
        folded = fold(s)
        for fact in facts:
            if fold(fact) not in folded:
                return None
        return s

    # ------------------------------------------------------------------ G1 + G2
    def formulate_goals(self, request, response):
        t0 = time.time()
        system = (
            "You are the command interpreter of Argo, a robot dog that lives on an office floor.\n"
            "Places (ID | class | people working there | other names | entrance hub | camera_allowed | works_here):\n"
            f"{self.onto.prompt_table()}\n"
            "Missions: coffee_call = announce the coffee break in rooms, then chat at the coffee area; "
            "message = go to a person and say a message; inspect = look at a room with the camera; "
            "none = anything else.\n"
            "Rules:\n"
            "1. targets: ONLY place IDs from the list. A person's name means the room where that person works.\n"
            "2. coffee_call for everybody ('tutti', 'pausa caffè!', 'coffee time'): targets [\"ALL\"]. "
            "A corridor zone ('nord', 'ovest', 'centro'): every works_here room whose entrance hub has that word "
            "(north, west, center).\n"
            "3. ordered = true only if an explicit order is given ('prima ... poi ...'); keep that order.\n"
            "4. message_text: only for message, the content of the message, words unchanged, in Italian.\n"
            "5. If the place could be several IDs (e.g. just 'il lab') or the person is missing or only a "
            "pronoun ('lui', 'lei'), write a short Italian question in clarify_question and leave targets empty.\n"
            "6. Places or people not in the list go in unknown.\n"
            "7. If it is not a request for one of the missions, mission is none.\n"
            "8. reply: ONE short sentence in ITALIAN, first person, at most 20 words.\n"
            "Return ONLY JSON: {\"mission\": \"...\", \"targets\": [], \"ordered\": false, \"message_text\": \"\", "
            "\"clarify_question\": \"\", \"unknown\": [], \"reply\": \"\"}")
        user = f"Command: {request.command}\nContext: {request.context_json or '{}'}"
        parsed = self.llm_json(system, user, max_tokens=220)
        result = self.validate_goals(parsed, request.command) if parsed is not None else None
        if result is None:
            result = self.keyword_goals(request.command)
            result['used_fallback'] = True
        for key, value in result.items():
            setattr(response, key, value)
        self.get_logger().info(
            f"[G1] '{request.command}' -> accepted={response.accepted} mission={response.mission} "
            f"targets={list(response.targets)} ordered={response.ordered} clarify='{response.clarify_question}' "
            f"fallback={response.used_fallback} ({time.time() - t0:.1f} s)")
        return response

    def empty_goals(self):
        return {'accepted': False, 'mission': 'none', 'targets': [], 'ordered': False, 'message_text': '',
                'clarify_question': '', 'reply': '', 'used_fallback': False}

    def validate_goals(self, parsed, command):
        """Gemma's answer checked against the ontology; None = unusable (fallback)."""
        out = self.empty_goals()
        mission = str(parsed.get('mission', 'none')).strip().lower()
        if mission not in MISSIONS:
            return None
        clarify = str(parsed.get('clarify_question') or '').strip()
        unknown = [str(u) for u in (parsed.get('unknown') or []) if str(u).strip()]
        # An "unknown" place that is in fact a known name is not unknown
        unknown = [u for u in unknown if self.onto.resolve(u) is None and not self.onto.zone(u)]
        targets, bad = [], []
        for t in parsed.get('targets') or []:
            t = str(t).strip()
            if t.upper() == 'ALL':
                targets += [p for p in self.onto.works_here() if p not in targets]
                continue
            pid = t if t in self.onto.places else self.onto.resolve(t)
            zone = self.onto.zone(t)
            if pid:
                targets.append(pid) if pid not in targets else None
            elif zone:
                targets += [p for p in zone if p not in targets]
            else:
                bad.append(t)
        unknown += bad
        # Deterministic safety net: a name in the command that is not in the ontology at all
        unknown += [n for n in self.unknown_people(command) if n not in unknown]
        out['mission'] = mission
        out['ordered'] = bool(parsed.get('ordered', False)) and len(targets) > 1
        out['message_text'] = str(parsed.get('message_text') or '').strip() if mission == 'message' else ''

        if unknown:
            out['reply'] = (f"Non so dove si trova: {', '.join(dict.fromkeys(unknown))}. "
                            f"Conosco solo le stanze del piano.")
            return out
        if clarify and not targets:
            out['clarify_question'] = self.good_sentence(clarify) or 'Puoi dirmi meglio dove devo andare?'
            out['reply'] = out['clarify_question']
            return out
        if mission == 'none':
            out['reply'] = 'Posso chiamare per il caffè, portare messaggi o ispezionare le stanze.'
            return out
        if mission == 'coffee_call' and not targets:
            targets = self.onto.works_here()
        if not targets:
            out['clarify_question'] = 'Puoi dirmi in quale stanza o a chi devo andare?'
            out['reply'] = out['clarify_question']
            return out
        if mission == 'inspect':
            private = [t for t in targets if not self.onto.camera_allowed(t)]
            if private:
                out['reply'] = (f"Non posso ispezionare {', '.join(self.onto.spoken(t) for t in private)}: "
                                f"è un ufficio e rispetto la privacy.")
                return out
        if mission == 'message' and not out['message_text']:
            out['clarify_question'] = 'Qual è il messaggio che devo portare?'
            out['reply'] = out['clarify_question']
            return out
        out['accepted'] = True
        out['targets'] = targets
        out['reply'] = self.good_sentence(parsed.get('reply')) or self.goal_reply(mission, targets)
        return out

    def unknown_people(self, command):
        """Names after 'da' / 'a' (vai da Pippo, di' a Pippo) that are not in the ontology."""
        out = []
        for m in re.finditer(r"\b(?:da|a)\s+([A-Z][a-zà-ù]+)", command):
            name = m.group(1)
            if fold(name) not in self.onto.names and not self.onto.zone(name):
                out.append(name)
        return out

    def goal_reply(self, mission, targets):
        names = ', '.join(self.onto.spoken(t) for t in targets)
        if mission == 'coffee_call':
            return f'Ricevuto: annuncio la pausa caffè in {len(targets)} stanze.'
        if mission == 'message':
            people = ', '.join(n for t in targets for n in self.onto.occupants(t)[:1]) or names
            return f'Ricevuto: porto il messaggio a {people}.'
        return f'Ricevuto: vado a ispezionare {names}.'

    def keyword_goals(self, command):
        """G1 fallback: keywords on IDs, synonyms, occupants and corridor zones."""
        out = self.empty_goals()
        c = fold(command)
        coffee = re.search(r'caff|coffee|pausa', c)
        # "di' a" / "dì a" only: a bare "di" is everywhere ("laboratorio di visione")
        message = re.search(r"\b(digli|dille|dillo|dirlo|ditegli|messaggio|riferisci|avvisa|dire|dica)\b|\bdi'? a\b", c)
        # "di' a Vito che il caffè è pronto" is a message that mentions coffee, not a coffee call
        if message and (re.search(r'\bche\b', c) or not coffee):
            mission = 'message'
        elif coffee:
            mission = 'coffee_call'
        elif re.search(r'ispezion|ispeziona|controlla|guarda|verifica', c):
            mission = 'inspect'
        else:
            out['reply'] = 'Posso chiamare per il caffè, portare messaggi o ispezionare le stanze.'
            return out
        out['mission'] = mission
        unknown = self.unknown_people(command)
        if unknown:
            out['reply'] = f"Non so dove si trova: {', '.join(unknown)}. Conosco solo le stanze del piano."
            return out
        targets = self.onto.find_in_text(command)
        for w in c.split():
            targets += [p for p in self.onto.zone(w) if p not in targets]
        if mission == 'coffee_call' and (not targets or re.search(r'\btutti\b|everyone|all', c)):
            targets = self.onto.works_here()
        if not targets:
            out['clarify_question'] = 'Puoi dirmi in quale stanza o a chi devo andare?'
            out['reply'] = out['clarify_question']
            return out
        if mission == 'inspect':
            private = [t for t in targets if not self.onto.camera_allowed(t)]
            if private:
                out['reply'] = (f"Non posso ispezionare {', '.join(self.onto.spoken(t) for t in private)}: "
                                f"è un ufficio e rispetto la privacy.")
                return out
        if mission == 'message':
            m = re.search(r'\bche\s+(.+)$', command, re.IGNORECASE)
            out['message_text'] = m.group(1).strip() if m else ''
            if not out['message_text']:
                out['clarify_question'] = 'Qual è il messaggio che devo portare?'
                out['reply'] = out['clarify_question']
                return out
        out['ordered'] = bool(re.search(r'\bprima\b.*\bpoi\b', c)) and len(targets) > 1
        out['accepted'] = True
        out['targets'] = targets
        out['reply'] = self.goal_reply(mission, targets)
        return out

    # ------------------------------------------------------------------ G3
    def assess_failure(self, request, response):
        t0 = time.time()
        facts = {'failed_action': request.failed_action, 'action_args': list(request.action_args),
                 'detections': request.detections_json or '{}', 'battery': round(request.battery, 1),
                 'reachable_targets': list(request.reachable_targets),
                 'unreachable_targets': list(request.unreachable_targets), 'attempts': request.attempts}
        system = (
            "You decide how Argo, a robot dog, recovers after a failed action. Choose ONE strategy:\n"
            "WAIT_RETRY = wait 20 s and try again (temporary obstacle, few attempts);\n"
            "ASK_CLEAR = ask people to let me pass (people detected in the way);\n"
            "DROP_UNREACHABLE = give up the unreachable targets and keep the reachable ones;\n"
            "RETURN_HOME = go back to the lab (battery low or nothing left to do);\n"
            "ABORT = stop the mission.\n"
            "sentence: ONE Italian sentence, first person, at most 20 words, explaining the choice.\n"
            "Return ONLY JSON: {\"strategy\": \"...\", \"sentence\": \"...\"}")
        parsed = self.llm_json(system, json.dumps(facts, ensure_ascii=False), max_tokens=120)
        strategy = str((parsed or {}).get('strategy', '')).strip().upper()
        sentence = self.good_sentence((parsed or {}).get('sentence'))
        used_fallback = False
        if strategy not in STRATEGIES or not sentence:
            strategy, sentence = self.fallback_strategy(request)
            used_fallback = True
        if strategy == 'DROP_UNREACHABLE' and not request.reachable_targets:
            strategy, sentence = 'RETURN_HOME', 'Non posso raggiungere nessuna stanza rimasta: torno in laboratorio.'
        response.strategy, response.sentence, response.used_fallback = strategy, sentence, used_fallback
        self.get_logger().info(f'[G3] {facts} -> {strategy} fallback={used_fallback} ({time.time() - t0:.1f} s)')
        return response

    def fallback_strategy(self, request):
        # battery < 0 means unknown (no lowstate): never a reason to go home
        if 0 <= request.battery < self.onto.data['robot'].get('reserve', 15):
            return 'RETURN_HOME', 'Ho poca batteria: interrompo e torno in laboratorio.'
        if request.attempts < 1:
            return 'WAIT_RETRY', 'Il passaggio è bloccato: aspetto un momento e riprovo.'
        if request.reachable_targets:
            return 'DROP_UNREACHABLE', 'Il passaggio resta bloccato: salto le stanze che non posso raggiungere.'
        return 'RETURN_HOME', 'Non riesco a proseguire: torno in laboratorio.'

    # ------------------------------------------------------------------ G4
    def appraise_findings(self, request, response):
        t0 = time.time()
        detections = self.parse_detections(request.detections_json)
        room = self.onto.spoken(request.room) if request.room in self.onto.places else request.room
        system = (
            "Argo, a robot dog, has just inspected a room of an office. Decide whether what it saw is worth "
            "reporting to the staff (REPORT) or not (NONE): e.g. people in a room late in the evening, "
            "devices left on, unusual objects. Ordinary things during working hours are NONE.\n"
            "sentence: ONE Italian sentence, first person, at most 20 words; use the counts unchanged.\n"
            "Return ONLY JSON: {\"verdict\": \"NONE|REPORT\", \"sentence\": \"...\"}")
        user = json.dumps({'room': room, 'detections': detections, 'time_of_day': request.time_of_day},
                          ensure_ascii=False)
        parsed = self.llm_json(system, user, max_tokens=120)
        verdict = str((parsed or {}).get('verdict', '')).strip().upper()
        counts = [number_str(v) for v in detections.values()]
        sentence = self.good_sentence((parsed or {}).get('sentence'), counts)
        if verdict not in VERDICTS or not sentence:
            verdict = 'REPORT' if detections else 'NONE'
            sentence = self.describe_inspection(room, detections)
        response.verdict, response.sentence = verdict, sentence
        self.get_logger().info(f'[G4] {room} {detections} -> {verdict} ({time.time() - t0:.1f} s)')
        return response

    def parse_detections(self, text):
        try:
            data = json.loads(text or '{}')
            return {str(k): v for k, v in data.items() if isinstance(v, (int, float))} if isinstance(data, dict) else {}
        except json.JSONDecodeError:
            return {}

    def describe_inspection(self, room, detections):
        if not detections:
            return f'Ho ispezionato {room}: non ho visto niente di particolare.'
        parts = []
        for cls, n in sorted(detections.items(), key=lambda kv: -kv[1])[:4]:
            one, many = COCO_IT.get(cls, (cls, cls))
            parts.append(f'{number_str(n)} {one if n == 1 else many}')
        return f"Ho ispezionato {room}: vedo {', '.join(parts)}."

    # ------------------------------------------------------------------ G5
    def speak(self, request, response):
        t0 = time.time()
        event = request.event_type.strip()
        try:
            facts = json.loads(request.facts_json or '{}')
            facts = facts if isinstance(facts, dict) else {}
        except json.JSONDecodeError:
            facts = {}
        template = self.template(event, facts)
        if event == 'message':
            # Words of a message are never rewritten: the template is the sentence
            response.sentence, response.used_fallback = template, False
            return response
        must_keep = self.fact_values(facts)
        extra = ''
        if event == 'announce':
            # The fact of an announcement is the coffee break: it must be in the sentence. The names are
            # added by the announce executor, so Gemma must not repeat them.
            must_keep = must_keep + ['caff', 'pausa']   # "Io amo il caffè" is not an announcement
            extra = ("Announce the coffee break: everybody is invited to the coffee machine now. "
                     "Do NOT greet and do NOT say any name. ")
        system = (
            "You write what Argo, a friendly robot dog in an office, says aloud. ONE sentence in ITALIAN, "
            f"first person, at most 20 words. Keep every name and number of the facts unchanged. {extra}"
            f"Event: {event}. Example of the meaning: \"{template}\".\n"
            "Return ONLY JSON: {\"sentence\": \"...\"}")
        parsed = self.llm_json(system, json.dumps(facts, ensure_ascii=False), max_tokens=100)
        sentence = self.good_sentence((parsed or {}).get('sentence'), must_keep)
        if sentence and event == 'announce' and (
                GENERIC_LINE.search(sentence) or any(fold(n) in fold(sentence) for n in facts.get('names', []))):
            sentence = None   # empty announcement ("sono qui per annunciare!") or names repeated
        response.sentence = sentence or template
        response.used_fallback = sentence is None
        self.get_logger().info(f'[G5] {event} {facts} -> "{response.sentence}" fallback={response.used_fallback} '
                               f'({time.time() - t0:.1f} s)')
        return response

    def fact_values(self, facts):
        """Scalar facts that must appear unchanged in the sentence (names, numbers, times)."""
        out = []
        for k, v in facts.items():
            if isinstance(v, bool) or v is None or k in ('detections', 'room'):
                continue
            if isinstance(v, (int, float)):
                out.append(number_str(v))
            elif isinstance(v, str) and v.strip() and words(v) <= 3:
                out.append(self.onto.spoken(v) if v in self.onto.places else v)
        return out

    def template(self, event, f):
        room = f.get('room', '')
        room = self.onto.spoken(room) if room in self.onto.places else room
        if event == 'announce':
            return 'Pausa caffè! Vi aspetto alla macchinetta del caffè.'
        if event == 'message':
            name = f.get('name') or (self.onto.occupants(f.get('room', ''))[:1] or [''])[0]
            return f"{name}, ho un messaggio per te: {f.get('text', '')}".strip()
        if event == 'inspection':
            return self.describe_inspection(room, f.get('detections') or {})
        if event == 'failure':
            target = f.get('target', '')
            target = self.onto.spoken(target) if target in self.onto.places else target
            return f"Non riesco a raggiungere {target}: {f.get('reason', 'il passaggio è bloccato')}."
        if event == 'battery_low':
            return f"Ho la batteria al {number_str(f.get('battery', 0))}%: torno in laboratorio a ricaricarmi."
        if event == 'sensor_fault':
            return f"Il sensore {f.get('sensor', '')} non funziona: mi fermo per sicurezza."
        if event == 'permission':
            return f'Posso entrare in {room}?'
        return 'Va bene.'

    # ------------------------------------------------------------------ G6
    def chat(self, request, response):
        t0 = time.time()
        detections = self.parse_detections(request.detections_json)
        history = [h.strip() for h in (request.history or '').split('\n') if h.strip()]
        # The jokes are written by Gemma. The prompt describes the style without jokes to copy; the
        # fixed list CHAT_LINES is used only if Gemma fails CHAT_ATTEMPTS times.
        system = (
            "You are Argo, a funny robot dog at the office coffee machine during the coffee break. INVENT ONE "
            "NEW short JOKE about COFFEE in ITALIAN, at most 25 words: a question with a funny answer, or a pun "
            "on coffee words (moka, espresso, tazzina, cialda, zucchero, barista, ristretto...). "
            "No greetings, never start with 'Ciao'. Do not repeat or rephrase the jokes already told. "
            "If people are seen, you may joke about them. "
            "Return ONLY JSON: {\"line\": \"...\"}")
        user = json.dumps({'vedo': detections, 'gia_detto': history[-8:]}, ensure_ascii=False)
        line, source = None, 'lista'
        for attempt in range(CHAT_ATTEMPTS):
            parsed = self.llm_json(system, user, max_tokens=90, temperature=CHAT_TEMPERATURE)
            if parsed is None and attempt == 0 and not self.llm_reachable():
                break   # llama.cpp off: no point in retrying
            candidate = self.good_sentence((parsed or {}).get('line'))
            reason = self.bad_joke(candidate, history)
            if reason is None:
                line, source = candidate, 'gemma'
                break
            self.get_logger().info(f'[G6] scartata ({reason}): "{candidate}"')
        if line is None:
            candidates = [c for c in CHAT_LINES if c not in history] or CHAT_LINES
            line = candidates[self.chat_index % len(candidates)]
            self.chat_index += 1
        response.line = line
        self.get_logger().info(f'[G6] [{source}] "{line}" ({time.time() - t0:.1f} s)')
        return response

    def bad_joke(self, line, history):
        """Why a Gemma joke is rejected, or None if it is good."""
        if not line:
            return 'vuota, troppo lunga o non in italiano'
        if GENERIC_LINE.search(line):
            return 'saluto o frase generica'
        if not COFFEE_WORDS.search(line):
            return 'non parla di caffè'
        if any(fold(line) == fold(c) for c in CHAT_LINES):
            return 'copiata dalla lista fissa'
        if any(difflib.SequenceMatcher(None, fold(line), fold(h)).ratio() > 0.75 for h in history):
            return 'troppo simile a una già detta'
        return None

    def llm_reachable(self):
        try:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(f'{self.api}/models', timeout=2):
                return True
        except Exception:  # noqa: BLE001
            return False


def main():
    rclpy.init()
    node = GemmaServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
