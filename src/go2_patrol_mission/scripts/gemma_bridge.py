#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import re
import json
import urllib.request
import urllib.error
import time
import math
import os
import threading
import yaml

from ament_index_python.packages import get_package_share_directory
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# Import per interrogare lo stato di attivazione dei Lifecycle Nodes
from lifecycle_msgs.srv import GetState

from go2_interfaces.srv import GemmaCommand
from plansys2_msgs.msg import Node as PlanNode, Param, Tree
from plansys2_msgs.srv import (
    AffectParam, AffectNode, AddProblemGoal, ClearProblemKnowledge, 
    GetPlan, GetDomain, GetProblem, GetStates
)
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ActionExecutionInfo
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

# Luoghi che il robot conosce (devono coincidere con il dominio PDDL e con config/waypoints.yaml)
ROOM_INFO = {
    'lab_robotics': {
        'name_it': 'laboratorio di robotica',
        'desc': "the robotics lab: 'lab', 'laboratorio', 'lab di robotica', where robots are developed/built",
    },
    'corridor': {
        'name_it': 'corridoio',
        'desc': "the corridor: 'corridoio', 'hallway', the main passage connecting the rooms",
    },
    'openspace': {
        'name_it': 'openspace',
        'desc': "the open space: 'open space', 'openspace', open area, shared office",
    },
}


# Sinonimi -> room ID (Gemma a volte restituisce il nome in italiano invece dell'ID)
ROOM_ALIASES = {
    'lab_robotics': 'lab_robotics', 'lab robotics': 'lab_robotics', 'lab': 'lab_robotics',
    'laboratorio': 'lab_robotics', 'laboratorio di robotica': 'lab_robotics',
    'lab di robotica': 'lab_robotics', 'robotics lab': 'lab_robotics', 'il lab': 'lab_robotics',
    'il laboratorio': 'lab_robotics',
    'corridor': 'corridor', 'corridoio': 'corridor', 'il corridoio': 'corridor',
    'hallway': 'corridor', 'hall': 'corridor',
    'openspace': 'openspace', 'open space': 'openspace', 'open_space': 'openspace',
    "l'openspace": 'openspace', "l'open space": 'openspace', 'open area': 'openspace',
}


def normalize_room(name):
    """Restituisce il room ID se 'name' è una stanza nota (o un suo sinonimo), altrimenti None."""
    key = str(name).strip().lower().replace('-', ' ')
    return ROOM_ALIASES.get(key) or ROOM_ALIASES.get(key.replace(' ', '_'))


def room_it(room_id):
    if room_id == 'start_pos':
        return 'posizione di partenza'
    return ROOM_INFO.get(room_id, {}).get('name_it', room_id)


def known_places_it():
    return ', '.join(f"{v['name_it']} ({k})" for k, v in ROOM_INFO.items())


class GemmaBridge(Node):
    def __init__(self):
        super().__init__('gemma_bridge')
        
        self.cb_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()  # Lock per la concorrenza dei thread
        
        # Dichiarazione parametri LLM
        self.declare_parameter('gemma_api_base', 'http://127.0.0.1:8000/v1')
        self.declare_parameter('gemma_model_id', 'google/gemma-4-E2B-it')
        self.gemma_api_base = self.get_parameter('gemma_api_base').value
        self.gemma_model_id = self.get_parameter('gemma_model_id').value
        
        # Stato interno
        self.target_rooms = []
        self.rooms = ['lab_robotics', 'corridor', 'openspace']

        # Waypoint delle stanze (stesso file usato da navigate_action_node)
        self.declare_parameter('waypoints_file', os.path.join(
            get_package_share_directory('go2_patrol_mission'), 'config', 'waypoints.yaml'))
        # Distanza massima (m) dal waypoint per considerare il robot "dentro" una stanza
        self.declare_parameter('room_radius', 1.5)
        self.waypoints = self.load_waypoints(self.get_parameter('waypoints_file').value)
        self.room_radius = float(self.get_parameter('room_radius').value)

        # Posa reale del robot da AMCL (transient_local: riceviamo subito l'ultima posa)
        self.robot_pose = None
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, amcl_qos,
            callback_group=self.cb_group)

        # Piano PlanSys2 in esecuzione (per poterlo annullare quando arriva un nuovo comando)
        self.plan_goal_handle = None
        self.plan_lock = threading.Lock()
        # Fino a questo istante i 'failure:navigate' sono dovuti a una cancellazione voluta: niente recovery
        self.ignore_failures_until = 0.0
        self.nav2_cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=self.cb_group)
        
        self.srv = self.create_service(
            GemmaCommand, 
            '/gemma_interface/send_command', 
            self.command_callback,
            callback_group=self.cb_group
        )
        
        # Sottoscrizione eventi azioni per recupero/replanning
        self.events_sub = self.create_subscription(
            String,
            '/patrol/action_events',
            self.events_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Publisher per spiegazioni in linguaggio naturale
        self.explain_pub = self.create_publisher(String, '/supervisor/explanation', 10)
        
        # Client PlanSys2
        self.get_state_cli = self.create_client(GetState, '/problem_expert/get_state', callback_group=self.cb_group)
        self.get_domain_cli = self.create_client(GetDomain, '/domain_expert/get_domain', callback_group=self.cb_group)
        self.get_problem_cli = self.create_client(GetProblem, '/problem_expert/get_problem', callback_group=self.cb_group)
        self.clear_kb_cli = self.create_client(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge', callback_group=self.cb_group)
        self.add_inst_cli = self.create_client(AffectParam, '/problem_expert/add_problem_instance', callback_group=self.cb_group)
        self.add_pred_cli = self.create_client(AffectNode, '/problem_expert/add_problem_predicate', callback_group=self.cb_group)
        self.remove_pred_cli = self.create_client(AffectNode, '/problem_expert/remove_problem_predicate', callback_group=self.cb_group)
        self.get_predicates_cli = self.create_client(GetStates, '/problem_expert/get_problem_predicates', callback_group=self.cb_group)
        self.set_goal_cli = self.create_client(AddProblemGoal, '/problem_expert/add_problem_goal', callback_group=self.cb_group)
        self.get_plan_cli = self.create_client(GetPlan, '/planner/get_plan', callback_group=self.cb_group)
        
        # Action client per eseguire il piano
        self.exec_plan_cli = ActionClient(self, ExecutePlan, '/execute_plan', callback_group=self.cb_group)
        
        self.get_logger().info("Gemma Bridge started. Waiting for commands on /gemma_interface/send_command")

    def load_waypoints(self, path):
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            wps = {name: (float(v['x']), float(v['y'])) for name, v in data.get('waypoints', {}).items()}
            self.get_logger().info(f"Waypoint caricati da {path}: {list(wps.keys())}")
            return wps
        except Exception as e:
            self.get_logger().error(f"Impossibile leggere i waypoint da {path}: {e}")
            return {}

    def amcl_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def current_room(self):
        """Stanza in cui si trova il robot (waypoint più vicino entro room_radius), altrimenti None."""
        if self.robot_pose is None:
            self.get_logger().warn("Nessuna posa da /amcl_pose: hai dato la 2D Pose Estimate?")
            return None
        x, y = self.robot_pose
        best, best_d = None, float('inf')
        for room in self.rooms:
            if room in self.waypoints:
                wx, wy = self.waypoints[room]
                d = math.hypot(x - wx, y - wy)
                if d < best_d:
                    best, best_d = room, d
        self.get_logger().info(
            f"Posa robot ({x:.2f}, {y:.2f}); stanza più vicina: {best} a {best_d:.2f} m")
        return best if best_d <= self.room_radius else None

    def say(self, text):
        """Pubblica un messaggio leggibile su /supervisor/explanation (diario della missione)."""
        self.get_logger().info(f"[EXPLAIN] {text}")
        self.explain_pub.publish(String(data=text))

    def describe_action(self, info):
        """Frase per un cambio di stato di un'azione PlanSys2 (None = niente da dire)."""
        args = list(info.arguments)
        if info.action == 'navigate' and len(args) >= 3:
            src, dst = room_it(args[1]), room_it(args[2])
            if info.status == ActionExecutionInfo.EXECUTING:
                return f"Mi sto muovendo da {src} verso {dst}."
            if info.status == ActionExecutionInfo.SUCCEEDED:
                return f"Sono arrivato in {dst}."
            if info.status == ActionExecutionInfo.FAILED:
                return f"Non sono riuscito ad arrivare in {dst} ({info.message_status or 'navigazione fallita'})."
        elif info.action == 'inspect' and len(args) >= 2:
            room = room_it(args[1])
            if info.status == ActionExecutionInfo.EXECUTING:
                return f"Inizio l'ispezione di {room}."
            if info.status == ActionExecutionInfo.SUCCEEDED:
                return f"Ispezione di {room} completata."
            if info.status == ActionExecutionInfo.FAILED:
                return f"Ispezione di {room} fallita ({info.message_status or 'errore'})."
        return None

    def send_plan(self, plan):
        """Invia il piano a /execute_plan, racconta l'avanzamento e tiene traccia del goal per poterlo annullare."""
        exec_goal = ExecutePlan.Goal()
        exec_goal.plan = plan
        last_status = {}  # action_full_name -> ultimo stato già raccontato

        def _on_feedback(fb_msg):
            for info in fb_msg.feedback.action_execution_status:
                if last_status.get(info.action_full_name) == info.status:
                    continue
                last_status[info.action_full_name] = info.status
                text = self.describe_action(info)
                if text:
                    self.say(text)

        future = self.exec_plan_cli.send_goal_async(exec_goal, feedback_callback=_on_feedback)

        def _on_goal(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.say("PlanSys2 ha rifiutato il piano: non parto.")
                return
            with self.plan_lock:
                self.plan_goal_handle = handle

            def _on_result(res_fut):
                with self.plan_lock:
                    if self.plan_goal_handle is handle:
                        self.plan_goal_handle = None
                try:
                    wrapped = res_fut.result()
                except Exception:
                    return
                if wrapped.status == GoalStatus.STATUS_CANCELED:
                    self.say("Missione annullata.")
                elif wrapped.result.success:
                    with self.state_lock:
                        rooms = ', '.join(room_it(r) for r in self.target_rooms)
                    self.say(f"Missione completata: ho ispezionato {rooms}.")
                elif time.time() >= self.ignore_failures_until:
                    self.say("La missione si è interrotta per un errore.")
            handle.get_result_async().add_done_callback(_on_result)

        future.add_done_callback(_on_goal)

    def cancel_running_plan(self):
        """Annulla il piano in corso (se c'è) e ferma qualunque goal Nav2 attivo."""
        with self.plan_lock:
            handle = self.plan_goal_handle
            self.plan_goal_handle = None
        if handle is None:
            return
        self.get_logger().warn("Nuovo comando ricevuto: annullo il piano in corso e il goal Nav2.")
        self.ignore_failures_until = time.time() + 6.0
        handle.cancel_goal_async()
        # CancelGoal con goal_id e stamp a zero = annulla TUTTI i goal di navigate_to_pose
        self.call_service(self.nav2_cancel_cli, CancelGoal.Request(), timeout_sec=2.0)
        time.sleep(2.0)  # lascia tempo a executor e action node di tornare inattivi

    def call_service(self, client, request, timeout_sec=5.0):
        """Esegue una chiamata di servizio sicura con timeout per evitare stalli."""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            srv_name = getattr(client, 'srv_name', getattr(client, 'service_name', 'unknown_service'))
            self.get_logger().error(f'Service {srv_name} not available after {timeout_sec}s.')
            return None
            
        future = client.call_async(request)
        start_time = time.time()
        
        while rclpy.ok() and not future.done():
            if time.time() - start_time > timeout_sec:
                srv_name = getattr(client, 'srv_name', getattr(client, 'service_name', 'unknown_service'))
                self.get_logger().error(f'Service call to {srv_name} timed out.')
                return None
            time.sleep(0.01)
            
        return future.result()

    def wait_for_plansys2_active(self, timeout_sec=15.0):
        self.get_logger().info("Verifica dello stato di PlanSys2...")
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if not self.get_state_cli.wait_for_service(timeout_sec=1.0):
                time.sleep(0.5)
                continue
            
            req = GetState.Request()
            res = self.call_service(self.get_state_cli, req, timeout_sec=2.0)
            if res is not None and res.current_state.label == "active":
                self.get_logger().info("PlanSys2 è ATTIVO e pronto a ricevere comandi.")
                return True
            
            time.sleep(1.0)
        return False

    def query_gemma(self, prompt_text):
        url = "http://127.0.0.1:8000/v1/chat/completions"
        
        # Payload identico a quello del curl che ha funzionato
        payload = {
            "model": "google/gemma-4-E2B-it",
            "messages": [{"role": "user", "content": prompt_text}]
        }
        
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            
            # Forza l'ignoranza dei proxy locali
            proxy_handler = urllib.request.ProxyHandler({})
            opener = urllib.request.build_opener(proxy_handler)
            
            # Timeout impostato a 10 secondi per evitare blocchi eterni
            with opener.open(req, timeout=10.0) as response:
                body = response.read().decode("utf-8")
                result = json.loads(body)
                content = result.get("choices", [{}])[0].get("message", {}).get("content", "")
                return content.strip()
                
        except Exception as e:
            self.get_logger().error(f"Errore di comunicazione con l'LLM: {e}")
            return None

    def query_gemma_text(self, system_prompt, user_prompt, max_tokens=128):
        payload = {
            "model": self.gemma_model_id,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "max_tokens": max_tokens,
            "temperature": 0.3,
            "stream": False,
        }
        try:
            url = f"{self.gemma_api_base}/chat/completions"
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            
            # Forza urllib a ignorare i proxy di sistema per localhost
            proxy_handler = urllib.request.ProxyHandler({})
            opener = urllib.request.build_opener(proxy_handler)
            
            with opener.open(req, timeout=60.0) as resp:
                body = resp.read().decode("utf-8")
                
            data = json.loads(body)
            return data.get("choices", [{}])[0].get("message", {}).get("content", "").strip()
        except Exception as e:
            self.get_logger().error(f"[LLM] Error querying Gemma Text: {e}")
            return None

    def query_gemma_json(self, system_prompt, user_prompt, max_tokens=128):
        """Ask Gemma for JSON and parse it; returns a dict or None (-> keyword fallback)."""
        text = self.query_gemma_text(system_prompt, user_prompt, max_tokens=max_tokens)
        if not text:
            return None
        self.get_logger().info(f"[LLM] Raw Gemma output: {text}")
        # Tolerate ```json fences or extra prose around the object
        start, end = text.find('{'), text.rfind('}')
        if start == -1 or end <= start:
            return None
        try:
            parsed = json.loads(text[start:end + 1])
            return parsed if isinstance(parsed, dict) else None
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"[LLM] Could not parse JSON from Gemma: {e}")
            return None

    def command_callback(self, request, response):
        command_text = request.command.lower()
        self.get_logger().info(f"Received natural command: {command_text}")
        
        rooms_desc = "\n".join(f"- '{k}': {v['desc']}" for k, v in ROOM_INFO.items())
        system_prompt = (
            "You are the command interpreter of a Unitree Go2 quadruped robot that patrols a building. "
            "The robot knows ONLY these places (room IDs) and can only navigate to them and inspect them:\n"
            f"{rooms_desc}\n"
            "Rules:\n"
            "1. Extract the rooms the operator wants the robot to go to / inspect, in the order requested, "
            "using ONLY the room IDs above.\n"
            "2. If the command mentions ANY place that is not one of the rooms above (e.g. kitchen, bathroom, "
            "office 3, outside, a person's desk), list those places in 'unknown_places' and set 'accepted' to false.\n"
            "3. If the command is not a request to go somewhere or inspect something, set 'accepted' to false.\n"
            "4. 'reply' is ONE short sentence for the operator, in the SAME language as the command. "
            "If accepted, confirm the rooms. If rejected, explain why and say which places you know.\n"
            "Return ONLY raw JSON (no markdown, no backticks) with this exact schema:\n"
            '{"accepted": true, "target_rooms": ["room_id", ...], "unknown_places": ["..."], "reply": "..."}'
        )

        parsed = self.query_gemma_json(system_prompt, f"Command: {command_text}", max_tokens=256)
        target_rooms = []
        unknown_places = []
        reply = ""
        accepted = True

        if parsed is not None:
            for room in parsed.get("target_rooms") or []:
                room_id = normalize_room(room)
                if room_id is None:
                    unknown_places.append(str(room))
                elif room_id not in target_rooms:
                    target_rooms.append(room_id)
            # Un "luogo sconosciuto" che in realtà è un sinonimo di una stanza nota non conta
            for place in parsed.get("unknown_places") or []:
                if place and normalize_room(place) is None:
                    unknown_places.append(str(place))
            reply = str(parsed.get("reply") or "").strip()
            accepted = parsed.get("accepted", True) not in (False, 'false', 'False')
            if target_rooms and not unknown_places:
                # Gemma ha estratto solo stanze valide: il rifiuto sarebbe un suo errore di formato
                accepted = True
        else:
            # Gemma non raggiungibile / risposta illeggibile: fallback a parole chiave sugli ID esatti
            self.get_logger().warn("Gemma non ha risposto in modo valido: uso il fallback a parole chiave.")
            for room in ROOM_INFO:
                if room in command_text:
                    target_rooms.append(room)

        if unknown_places or not accepted or not target_rooms:
            if unknown_places:
                reason = f"Non so dove si trova: {', '.join(dict.fromkeys(unknown_places))}."
            elif not target_rooms:
                reason = "Non ho riconosciuto nessun luogo che conosco nel comando."
            else:
                reason = "Il comando non è una richiesta di navigazione o ispezione."
            msg = (f"Goal RIFIUTATO. {('Gemma: ' + reply) if reply else reason} "
                   f"Conosco solo: {known_places_it()}.")
            self.say(msg)
            response.success = False
            response.message = msg
            return response

        with self.state_lock:
            self.target_rooms = target_rooms
        self.get_logger().info(f"Final parsed target rooms: {target_rooms}")

        if not self.wait_for_plansys2_active():
            response.success = False
            response.message = "Impossibile completare l'operazione: PlanSys2 non è attivo."
            return response

        self.cancel_running_plan()
        self.setup_plansys2_problem(target_rooms)
        
        domain_res = self.call_service(self.get_domain_cli, GetDomain.Request())
        if not domain_res or not domain_res.success:
            response.success = False
            response.message = "Failed to retrieve PDDL domain."
            return response

        problem_res = self.call_service(self.get_problem_cli, GetProblem.Request())
        if not problem_res or not problem_res.success:
            response.success = False
            response.message = "Failed to retrieve PDDL problem."
            return response
        
        plan_req = GetPlan.Request()
        plan_req.domain = domain_res.domain
        plan_req.problem = problem_res.problem
        plan_res = self.call_service(self.get_plan_cli, plan_req)
        
        if not plan_res or not plan_res.success:
            response.success = False
            response.message = "Failed to generate a plan."
            return response
            
        plan_str = "\n".join([f"{item.time}: {item.action} [{item.duration}]" for item in plan_res.plan.items])
        
        if not self.exec_plan_cli.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = "Action server /execute_plan not available."
            return response
            
        self.send_plan(plan_res.plan)
        
        response.success = True
        response.message = (f"Goal ACCETTATO: {', '.join(ROOM_INFO[r]['name_it'] for r in target_rooms)}. "
                            f"{('Gemma: ' + reply + ' ') if reply else ''}Piano avviato.")
        self.say(response.message)
        response.plan = plan_str
        return response

    def setup_plansys2_problem(self, target_rooms):
        self.call_service(self.clear_kb_cli, ClearProblemKnowledge.Request())
        
        self.add_instance('go2', 'robot')
        self.add_instance('lab_robotics', 'room')
        self.add_instance('corridor', 'room')
        self.add_instance('openspace', 'room')
        
        self.add_predicate('connected', [('lab_robotics', 'room'), ('corridor', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('openspace', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('lab_robotics', 'room')])
        self.add_predicate('connected', [('openspace', 'room'), ('corridor', 'room')])

        # Posizione iniziale REALE del robot (da AMCL), non più fissa in lab_robotics
        start_room = self.current_room()
        if start_room is not None:
            self.get_logger().info(f"Il robot è in '{start_room}'")
            self.add_predicate('robot_at', [('go2', 'robot'), (start_room, 'room')])
        else:
            # Robot fuori da tutte le stanze (o posa sconosciuta): posizione fittizia
            # 'start_pos' collegata a tutte le stanze, così il piano parte sempre con un navigate
            self.get_logger().info("Il robot non è in nessuna stanza nota: parto da 'start_pos'")
            self.add_instance('start_pos', 'room')
            self.add_predicate('robot_at', [('go2', 'robot'), ('start_pos', 'room')])
            for room in self.rooms:
                self.add_predicate('connected', [('start_pos', 'room'), (room, 'room')])
        
        goal_req = AddProblemGoal.Request()
        tree = Tree()
        
        if len(target_rooms) == 1:
            node = PlanNode(node_type=PlanNode.PREDICATE, name='inspected', parameters=[Param(name=target_rooms[0], type='room')])
            tree.nodes = [node]
        else:
            root = PlanNode(node_id=0, node_type=PlanNode.AND)
            nodes = [root]
            for i, room in enumerate(target_rooms):
                child_id = i + 1
                root.children.append(child_id)
                nodes.append(PlanNode(
                    node_id=child_id, 
                    node_type=PlanNode.PREDICATE, 
                    name='inspected', 
                    parameters=[Param(name=room, type='room')]
                ))
            tree.nodes = nodes
            
        goal_req.tree = tree
        self.call_service(self.set_goal_cli, goal_req)

    def events_callback(self, msg):
        data = msg.data.strip()
        parts = data.split(':')
        if len(parts) >= 5 and parts[0] == "failure" and parts[1] == "navigate":
            if time.time() < self.ignore_failures_until:
                self.get_logger().info("Navigazione annullata per un nuovo comando: nessun recovery.")
                return
            from_room = parts[3]
            to_room = parts[4]
            self.get_logger().warn(f"Navigation failure detected from {from_room} to {to_room}. Recovering...")
            threading.Thread(target=self.handle_navigation_recovery, args=(from_room, to_room)).start()

    def handle_navigation_recovery(self, from_room, to_room):
        time.sleep(2.0)
        
        system_prompt = (
            "You are an onboard AI assistant on a Unitree Go2 quadruped robot. "
            f"While navigating to the room '{to_room}' from '{from_room}', the path planner failed due to an obstruction. "
            "Explain the failure to the user in a concise sentence in first person. Return ONLY the explanation."
        )
        explanation = self.query_gemma_text(system_prompt, f"Failed to navigate from {from_room} to {to_room}.")
        if not explanation:
            explanation = f"I failed to navigate from {from_room} to {to_room} because of a path blockage."
        
        explain_msg = String()
        explain_msg.data = explanation
        self.explain_pub.publish(explain_msg)
        
        # Il dominio PDDL non ha un predicato 'blocked': per escludere il passaggio
        # bloccato rimuoviamo il collegamento tra le due stanze.
        self.remove_predicate('connected', [(from_room, 'room'), (to_room, 'room')])
        self.remove_predicate('connected', [(to_room, 'room'), (from_room, 'room')])
        self.add_predicate('robot_at', [('go2', 'robot'), (from_room, 'room')])
        
        inspected_rooms = []
        predicates_res = self.call_service(self.get_predicates_cli, GetStates.Request())
        if predicates_res and predicates_res.success:
            for state in predicates_res.states:
                if state.name == 'inspected' and state.parameters:
                    inspected_rooms.append(state.parameters[0].name)
        
        with self.state_lock:
            current_targets = list(self.target_rooms)
            
        remaining_rooms = [r for r in current_targets if r not in inspected_rooms]
        
        if not remaining_rooms:
            self.get_logger().info("All target rooms already inspected. No replanning needed.")
            return
            
        goal_req = AddProblemGoal.Request()
        tree = Tree()
        
        if len(remaining_rooms) == 1:
            node = PlanNode(node_type=PlanNode.PREDICATE, name='inspected', parameters=[Param(name=remaining_rooms[0], type='room')])
            tree.nodes = [node]
        else:
            root = PlanNode(node_id=0, node_type=PlanNode.AND)
            nodes = [root]
            for i, room in enumerate(remaining_rooms):
                child_id = i + 1
                root.children.append(child_id)
                nodes.append(PlanNode(
                    node_id=child_id, 
                    node_type=PlanNode.PREDICATE, 
                    name='inspected', 
                    parameters=[Param(name=room, type='room')]
                ))
            tree.nodes = nodes
            
        goal_req.tree = tree
        self.call_service(self.set_goal_cli, goal_req)
        
        domain_res = self.call_service(self.get_domain_cli, GetDomain.Request())
        problem_res = self.call_service(self.get_problem_cli, GetProblem.Request())
        
        if not domain_res or not domain_res.success or not problem_res or not problem_res.success:
            self.get_logger().error("Failed to retrieve updated domain/problem for replanning.")
            return
            
        plan_req = GetPlan.Request()
        plan_req.domain = domain_res.domain
        plan_req.problem = problem_res.problem
        plan_res = self.call_service(self.get_plan_cli, plan_req)
        
        if not plan_res or not plan_res.success:
            no_path_prompt = (
                "You are an onboard AI assistant on a Unitree Go2 quadruped robot. "
                "The path planning system failed to find an alternative route to remaining rooms "
                f"({', '.join(remaining_rooms)}). Explain to the user in first person that you are stuck."
            )
            no_path_explanation = self.query_gemma_text(no_path_prompt, "No plan found.")
            if not no_path_explanation:
                no_path_explanation = f"I cannot find any alternative route to reach: {', '.join(remaining_rooms)}."
                
            explain_msg.data = no_path_explanation
            self.explain_pub.publish(explain_msg)
            return
            
        if not self.exec_plan_cli.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server /execute_plan not available during replanning.")
            return
            
        self.send_plan(plan_res.plan)
        
    def add_instance(self, name, type_name):
        req = AffectParam.Request()
        req.param.name = name
        req.param.type = type_name
        self.call_service(self.add_inst_cli, req)
        
    def add_predicate(self, name, params):
        req = AffectNode.Request()
        req.node.node_type = PlanNode.PREDICATE
        req.node.name = name
        for p_name, p_type in params:
            req.node.parameters.append(Param(name=p_name, type=p_type))
        self.call_service(self.add_pred_cli, req)

    def remove_predicate(self, name, params):
        req = AffectNode.Request()
        req.node.node_type = PlanNode.PREDICATE
        req.node.name = name
        for p_name, p_type in params:
            req.node.parameters.append(Param(name=p_name, type=p_type))
        self.call_service(self.remove_pred_cli, req)

def main(args=None):
    rclpy.init(args=args)
    bridge = GemmaBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()