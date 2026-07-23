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
import threading

# Import per interrogare lo stato di attivazione dei Lifecycle Nodes
from lifecycle_msgs.srv import GetState

from go2_interfaces.srv import GemmaCommand
from plansys2_msgs.msg import Node as PlanNode, Param, Tree
from plansys2_msgs.srv import (
    AffectParam, AffectNode, AddProblemGoal, ClearProblemKnowledge, 
    GetPlan, GetDomain, GetProblem, GetStates
)
from plansys2_msgs.action import ExecutePlan
from std_msgs.msg import String

class GemmaBridge(Node):
    def __init__(self):
        super().__init__('gemma_bridge')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Dichiarazione parametri LLM
        self.declare_parameter('gemma_api_base', 'http://localhost:8000/v1')
        self.declare_parameter('gemma_model_id', 'google/gemma-4-E2B-it')
        self.gemma_api_base = self.get_parameter('gemma_api_base').value
        self.gemma_model_id = self.get_parameter('gemma_model_id').value
        
        # Stato interno
        self.target_rooms = []
        
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
        
        # Client per monitorare se PlanSys2 è pronto ed evitare crash
        self.get_state_cli = self.create_client(GetState, '/problem_expert/get_state', callback_group=self.cb_group)
        
        # Client per recuperare il PDDL reale del dominio e del problema
        self.get_domain_cli = self.create_client(GetDomain, '/domain_expert/get_domain', callback_group=self.cb_group)
        self.get_problem_cli = self.create_client(GetProblem, '/problem_expert/get_problem', callback_group=self.cb_group)
        
        # Client PlanSys2 standard
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

    def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {client.srv_name} not available, waiting again...')
        future = client.call_async(request)
        
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
            
        return future.result()

    def wait_for_plansys2_active(self, timeout_sec=15.0):
        """Attende in sicurezza che il problem_expert sia nello stato 'active'."""
        self.get_logger().info("Verifica dello stato di PlanSys2...")
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if not self.get_state_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Servizio di stato non ancora disponibile, attendo...")
                time.sleep(0.5)
                continue
            
            req = GetState.Request()
            res = self.call_service(self.get_state_cli, req)
            if res is not None and res.current_state.label == "active":
                self.get_logger().info("PlanSys2 è ATTIVO e pronto a ricevere comandi.")
                return True
            
            self.get_logger().warn("PlanSys2 non è ancora attivo (in fase di boot). Attendo...")
            time.sleep(1.0)
        return False

    def query_gemma_json(self, system_prompt, user_prompt):
        payload = {
            "model": self.gemma_model_id,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "max_tokens": 256,
            "temperature": 0.2,
            "stream": False,
        }
        try:
            url = f"{self.gemma_api_base}/chat/completions"
            self.get_logger().info(f"[LLM] POST to {url}")
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                body = resp.read().decode("utf-8")
            data = json.loads(body)
            content = data.get("choices", [{}])[0].get("message", {}).get("content", "").strip()
            self.get_logger().info(f"[LLM] Response:\n{content}")
            
            # Remove potential markdown code blocks
            if content.startswith("```"):
                lines = content.splitlines()
                if lines[0].startswith("```"):
                    lines = lines[1:]
                if lines[-1].startswith("```"):
                    lines = lines[:-1]
                content = "\n".join(lines).strip()
            return json.loads(content)
        except Exception as e:
            self.get_logger().error(f"[LLM] Error querying Gemma JSON: {e}")
            return None

    def query_gemma_text(self, system_prompt, user_prompt):
        payload = {
            "model": self.gemma_model_id,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "max_tokens": 128,
            "temperature": 0.3,
            "stream": False,
        }
        try:
            url = f"{self.gemma_api_base}/chat/completions"
            self.get_logger().info(f"[LLM] POST to {url}")
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                body = resp.read().decode("utf-8")
            data = json.loads(body)
            content = data.get("choices", [{}])[0].get("message", {}).get("content", "").strip()
            self.get_logger().info(f"[LLM] Response: {content}")
            return content
        except Exception as e:
            self.get_logger().error(f"[LLM] Error querying Gemma Text: {e}")
            return None

    def command_callback(self, request, response):
        command_text = request.command.lower()
        self.get_logger().info(f"Received natural command: {command_text}")
        
        # 1. Parse target rooms using Gemma
        system_prompt = (
            "You are a robotic task planning assistant for a Unitree Go2 quadruped robot. "
            "Your job is to parse a natural language command from a human operator and extract the target rooms that the robot needs to inspect. "
            "The available room IDs in the PDDL domain are: 'lab_robotics', 'corridor', 'openspace'. "
            "Map semantic descriptions to these rooms: "
            "- 'lab_robotics' is the robotics lab, where robots are developed/built, or the lab. "
            "- 'corridor' is the hallway, main corridor, or path connecting the rooms. "
            "- 'openspace' is the open area, open space, or shared office. "
            "You must output a JSON list containing the room IDs in the order they should be visited / inspected. "
            "You MUST return your answer in raw JSON format matching this exact schema:\n"
            "{\n"
            "  \"target_rooms\": [\"room_id_1\", \"room_id_2\", ...]\n"
            "}\n"
            "Do not include any markdown formatting, code block backticks (like ```json), or other text. Return ONLY the raw JSON."
        )
        user_prompt = f"Command: '{command_text}'"
        
        parsed = self.query_gemma_json(system_prompt, user_prompt)
        target_rooms = []
        if parsed and isinstance(parsed.get("target_rooms"), list):
            for room in parsed["target_rooms"]:
                if room in ['lab_robotics', 'corridor', 'openspace']:
                    target_rooms.append(room)
        
        if not target_rooms:
            self.get_logger().warn("LLM parsing empty or failed. Using fallback regex parser.")
            for room in ['lab_robotics', 'corridor', 'openspace']:
                if room in command_text:
                    target_rooms.append(room)
                    
        if not target_rooms:
            response.success = False
            response.message = "No known rooms (lab_robotics, corridor, openspace) found in the command."
            return response
            
        self.get_logger().info(f"Final parsed target rooms: {target_rooms}")
        self.target_rooms = target_rooms

        # Controllo preventivo dello stato del Lifecycle
        if not self.wait_for_plansys2_active():
            response.success = False
            response.message = "Impossibile completare l'operazione: PlanSys2 non è attivo."
            self.get_logger().error("Errore: timeout attivazione PlanSys2.")
            return response

        # 2. Configura il problema in PlanSys2
        self.setup_plansys2_problem(target_rooms)
        
        # 3. Recupera il Dominio in formato stringa PDDL
        self.get_logger().info("Retrieving PDDL domain string from domain_expert...")
        domain_res = self.call_service(self.get_domain_cli, GetDomain.Request())
        if not domain_res or not domain_res.success:
            response.success = False
            response.message = f"Failed to retrieve PDDL domain: {domain_res.error_info if domain_res else 'No response'}"
            return response

        # 4. Recupera il Problema in formato stringa PDDL
        self.get_logger().info("Retrieving PDDL problem string from problem_expert...")
        problem_res = self.call_service(self.get_problem_cli, GetProblem.Request())
        if not problem_res or not problem_res.success:
            response.success = False
            response.message = f"Failed to retrieve PDDL problem: {problem_res.error_info if problem_res else 'No response'}"
            return response
        
        # 5. Richiedi il piano a POPF passando le stringhe reali
        self.get_logger().info("Sending PDDL domain and problem to POPF planner...")
        plan_req = GetPlan.Request()
        plan_req.domain = domain_res.domain
        plan_req.problem = problem_res.problem
        plan_res = self.call_service(self.get_plan_cli, plan_req)
        
        if not plan_res or not plan_res.success:
            response.success = False
            response.message = f"Failed to generate a plan: {plan_res.error_info if plan_res else 'No response'}"
            return response
            
        plan_str = "\n".join([f"{item.time}: {item.action} [{item.duration}]" for item in plan_res.plan.items])
        self.get_logger().info(f"Generated Plan:\n{plan_str}")
        
        # 6. Esegui il piano in modo asincrono
        self.get_logger().info("Sending plan to executor...")
        exec_goal = ExecutePlan.Goal()
        exec_goal.plan = plan_res.plan
        self.exec_plan_cli.wait_for_server()
        self.exec_plan_cli.send_goal_async(exec_goal)
        
        response.success = True
        response.message = f"Plan generated for rooms: {', '.join(target_rooms)} and execution started."
        response.plan = plan_str
        return response

    def setup_plansys2_problem(self, target_rooms):
        # Ripulisci la conoscenza precedente
        self.call_service(self.clear_kb_cli, ClearProblemKnowledge.Request())
        
        # Aggiungi le istanze
        self.add_instance('go2', 'robot')
        self.add_instance('lab_robotics', 'room')
        self.add_instance('corridor', 'room')
        self.add_instance('openspace', 'room')
        
        # Stato iniziale: il robot parte da lab_robotics
        self.add_predicate('robot_at', [('go2', 'robot'), ('lab_robotics', 'room')])
        
        # Definizione della topologia
        self.add_predicate('connected', [('lab_robotics', 'room'), ('corridor', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('openspace', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('lab_robotics', 'room')])
        self.add_predicate('connected', [('openspace', 'room'), ('corridor', 'room')])
        
        # Configurazione del Goal Tree per PlanSys2
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
        self.get_logger().info(f"Received action event: {data}")
        parts = data.split(':')
        if len(parts) >= 5 and parts[0] == "failure" and parts[1] == "navigate":
            from_room = parts[3]
            to_room = parts[4]
            self.get_logger().warn(f"Navigation failure detected from {from_room} to {to_room}. Initiating replanning recovery...")
            threading.Thread(target=self.handle_navigation_recovery, args=(from_room, to_room)).start()

    def handle_navigation_recovery(self, from_room, to_room):
        # 1. Attendere la terminazione del piano corrente
        time.sleep(2.0)
        
        # 2. Richiedere spiegazione del fallimento a Gemma
        system_prompt = (
            "You are an onboard AI assistant on a Unitree Go2 quadruped robot. "
            f"While navigating to the room '{to_room}' from '{from_room}', the path planner failed due to a physical obstruction. "
            "Explain the failure to the user in a concise, natural-language sentence, in first person (e.g. 'I cannot reach the openspace because...'). "
            "Speak in first person. Return ONLY the explanation sentence."
        )
        user_prompt = f"Failed to navigate from {from_room} to {to_room}."
        explanation = self.query_gemma_text(system_prompt, user_prompt)
        if not explanation:
            explanation = f"I failed to navigate from {from_room} to {to_room} because of a path blockage."
        
        explain_msg = String()
        explain_msg.data = explanation
        self.explain_pub.publish(explain_msg)
        self.get_logger().info(f"[EXPLAIN] Published explanation: \"{explanation}\"")
        
        # 3. Aggiornare il problema PDDL: aggiungere il predicato blocked
        self.get_logger().info(f"Adding blocked predicate: {from_room} <-> {to_room}")
        self.add_predicate('blocked', [(from_room, 'room'), (to_room, 'room')])
        self.add_predicate('blocked', [(to_room, 'room'), (from_room, 'room')])
        
        # 4. Ripristinare lo stato di posizione del robot al nodo sorgente
        self.get_logger().info(f"Resetting robot position to {from_room}")
        self.add_predicate('robot_at', [('go2', 'robot'), (from_room, 'room')])
        
        # 5. Determinare i goal non ancora completati
        inspected_rooms = []
        predicates_res = self.call_service(self.get_predicates_cli, GetStates.Request())
        if predicates_res and predicates_res.success:
            for state in predicates_res.states:
                if state.name == 'inspected' and state.parameters:
                    room_param = state.parameters[0].name
                    inspected_rooms.append(room_param)
        self.get_logger().info(f"Inspected rooms so far: {inspected_rooms}")
        
        remaining_rooms = [r for r in self.target_rooms if r not in inspected_rooms]
        self.get_logger().info(f"Remaining target rooms to inspect: {remaining_rooms}")
        
        if not remaining_rooms:
            self.get_logger().info("All target rooms already inspected. No replanning needed.")
            return
            
        # 6. Configurare i nuovi goal
        self.get_logger().info(f"Setting remaining goals: {remaining_rooms}")
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
        
        # 7. Ottenere dominio e problema aggiornati
        domain_res = self.call_service(self.get_domain_cli, GetDomain.Request())
        problem_res = self.call_service(self.get_problem_cli, GetProblem.Request())
        
        if not domain_res or not domain_res.success or not problem_res or not problem_res.success:
            self.get_logger().error("Failed to retrieve updated domain/problem.")
            return
            
        # 8. Eseguire la ripianificazione
        plan_req = GetPlan.Request()
        plan_req.domain = domain_res.domain
        plan_req.problem = problem_res.problem
        plan_res = self.call_service(self.get_plan_cli, plan_req)
        
        if not plan_res or not plan_res.success:
            # Ripianificazione fallita - nessuna strada alternativa
            no_path_prompt = (
                "You are an onboard AI assistant on a Unitree Go2 quadruped robot. "
                "The path planning system failed to find a valid alternative route to the remaining rooms "
                f"({', '.join(remaining_rooms)}) because the path from {from_room} to {to_room} is blocked and "
                "there are no other connections. Explain to the user in a natural, polite sentence "
                "in first person that you are completely stuck and cannot reach the destinations."
            )
            no_path_explanation = self.query_gemma_text(no_path_prompt, "No plan found.")
            if not no_path_explanation:
                no_path_explanation = f"I cannot find any alternative route to reach the remaining rooms: {', '.join(remaining_rooms)}."
                
            explain_msg = String()
            explain_msg.data = no_path_explanation
            self.explain_pub.publish(explain_msg)
            self.get_logger().error(f"[REPLAN FAILURE] Published explanation: \"{no_path_explanation}\"")
            return
            
        plan_str = "\n".join([f"{item.time}: {item.action} [{item.duration}]" for item in plan_res.plan.items])
        self.get_logger().info(f"New Plan Generated:\n{plan_str}")
        
        # 9. Eseguire il nuovo piano
        self.get_logger().info("Executing new plan...")
        exec_goal = ExecutePlan.Goal()
        exec_goal.plan = plan_res.plan
        self.exec_plan_cli.wait_for_server()
        self.exec_plan_cli.send_goal_async(exec_goal)
        
    def add_instance(self, name, type_name):
        req = AffectParam.Request()
        req.param.name = name
        req.param.type = type_name
        res = self.call_service(self.add_inst_cli, req)
        if res is None or not res.success:
            self.get_logger().error(f"Failed to add instance {name} of type {type_name}: {res.error_info if res else 'No response'}")
        
    def add_predicate(self, name, params):
        req = AffectNode.Request()
        req.node.node_type = PlanNode.PREDICATE
        req.node.name = name
        for p_name, p_type in params:
            param = Param(name=p_name, type=p_type)
            req.node.parameters.append(param)
        res = self.call_service(self.add_pred_cli, req)
        if res is None or not res.success:
            self.get_logger().error(f"Failed to add predicate {name} with params {params}: {res.error_info if res else 'No response'}")

    def remove_predicate(self, name, params):
        req = AffectNode.Request()
        req.node.node_type = PlanNode.PREDICATE
        req.node.name = name
        for p_name, p_type in params:
            param = Param(name=p_name, type=p_type)
            req.node.parameters.append(param)
        res = self.call_service(self.remove_pred_cli, req)
        if res is None or not res.success:
            self.get_logger().error(f"Failed to remove predicate {name} with params {params}: {res.error_info if res else 'No response'}")

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