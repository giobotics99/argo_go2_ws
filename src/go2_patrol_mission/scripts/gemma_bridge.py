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
        self.state_lock = threading.Lock()  # Lock per la concorrenza dei thread[cite: 6]
        
        # Dichiarazione parametri LLM[cite: 6]
        self.declare_parameter('gemma_api_base', 'http://127.0.0.1:8000/v1')
        self.declare_parameter('gemma_model_id', 'google/gemma-4-E2B-it')
        self.gemma_api_base = self.get_parameter('gemma_api_base').value
        self.gemma_model_id = self.get_parameter('gemma_model_id').value
        
        # Stato interno[cite: 6]
        self.target_rooms = []
        
        self.srv = self.create_service(
            GemmaCommand, 
            '/gemma_interface/send_command', 
            self.command_callback,
            callback_group=self.cb_group
        )
        
        # Sottoscrizione eventi azioni per recupero/replanning[cite: 6]
        self.events_sub = self.create_subscription(
            String,
            '/patrol/action_events',
            self.events_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Publisher per spiegazioni in linguaggio naturale[cite: 6]
        self.explain_pub = self.create_publisher(String, '/supervisor/explanation', 10)
        
        # Client PlanSys2[cite: 6]
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
        
        # Action client per eseguire il piano[cite: 6]
        self.exec_plan_cli = ActionClient(self, ExecutePlan, '/execute_plan', callback_group=self.cb_group)
        
        self.get_logger().info("Gemma Bridge started. Waiting for commands on /gemma_interface/send_command")

    def call_service(self, client, request, timeout_sec=5.0):
        """Esegue una chiamata di servizio sicura con timeout per evitare stalli."""[cite: 6]
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
        self.get_logger().info("Verifica dello stato di PlanSys2...")[cite: 6]
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

    def command_callback(self, request, response):
        command_text = request.command.lower()
        self.get_logger().info(f"Received natural command: {command_text}")
        
        system_prompt = (
            "You are a robotic task planning assistant for a Unitree Go2 quadruped robot. "
            "Your job is to parse a natural language command from a human operator and extract the target rooms that the robot needs to inspect. "
            "The available room IDs in the PDDL domain are: 'lab_robotics', 'corridor', 'openspace'. "
            "Map semantic descriptions to these rooms: "
            "- 'lab_robotics' is the robotics lab, where robots are developed/built, or the lab. "
            "- 'corridor' is the hallway, main corridor, or path connecting the rooms. "
            "- 'openspace' is the open area, open space, or shared office. "
            "You must output a JSON list containing the room IDs in the order they should be visited / inspected. "
            "Return ONLY the raw JSON format matching this exact schema:\n"
            "{\n"
            "  \"target_rooms\": [\"room_id_1\", \"room_id_2\", ...]\n"
            "}\n"
            "Do not include markdown formatting or backticks."
        )[cite: 6]
        
        parsed = self.query_gemma_json(system_prompt, f"Command: '{command_text}'")
        target_rooms = []
        if parsed and isinstance(parsed.get("target_rooms"), list):
            for room in parsed["target_rooms"]:
                if room in ['lab_robotics', 'corridor', 'openspace']:
                    target_rooms.append(room)
        
        if not target_rooms:
            for room in ['lab_robotics', 'corridor', 'openspace']:
                if room in command_text:
                    target_rooms.append(room)
                    
        if not target_rooms:
            response.success = False
            response.message = "No known rooms found in the command."
            return response
            
        with self.state_lock:
            self.target_rooms = target_rooms
        self.get_logger().info(f"Final parsed target rooms: {target_rooms}")

        if not self.wait_for_plansys2_active():
            response.success = False
            response.message = "Impossibile completare l'operazione: PlanSys2 non è attivo."
            return response

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
            
        exec_goal = ExecutePlan.Goal()
        exec_goal.plan = plan_res.plan
        self.exec_plan_cli.send_goal_async(exec_goal)
        
        response.success = True
        response.message = f"Plan generated for rooms: {', '.join(target_rooms)} and execution started."
        response.plan = plan_str
        return response

    def setup_plansys2_problem(self, target_rooms):
        self.call_service(self.clear_kb_cli, ClearProblemKnowledge.Request())
        
        self.add_instance('go2', 'robot')
        self.add_instance('lab_robotics', 'room')
        self.add_instance('corridor', 'room')
        self.add_instance('openspace', 'room')
        
        self.add_predicate('robot_at', [('go2', 'robot'), ('lab_robotics', 'room')])
        
        self.add_predicate('connected', [('lab_robotics', 'room'), ('corridor', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('openspace', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('lab_robotics', 'room')])
        self.add_predicate('connected', [('openspace', 'room'), ('corridor', 'room')])
        
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
        )[cite: 6]
        explanation = self.query_gemma_text(system_prompt, f"Failed to navigate from {from_room} to {to_room}.")
        if not explanation:
            explanation = f"I failed to navigate from {from_room} to {to_room} because of a path blockage."
        
        explain_msg = String()
        explain_msg.data = explanation
        self.explain_pub.publish(explain_msg)
        
        self.add_predicate('blocked', [(from_room, 'room'), (to_room, 'room')])
        self.add_predicate('blocked', [(to_room, 'room'), (from_room, 'room')])
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
            )[cite: 6]
            no_path_explanation = self.query_gemma_text(no_path_prompt, "No plan found.")
            if not no_path_explanation:
                no_path_explanation = f"I cannot find any alternative route to reach: {', '.join(remaining_rooms)}."
                
            explain_msg.data = no_path_explanation
            self.explain_pub.publish(explain_msg)
            return
            
        if not self.exec_plan_cli.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server /execute_plan not available during replanning.")
            return
            
        exec_goal = ExecutePlan.Goal()
        exec_goal.plan = plan_res.plan
        self.exec_plan_cli.send_goal_async(exec_goal)
        
    def add_instance(self, name, type_name):
        req = AffectParam.Request()
        req.param.name = name
        req.param.type = type_name
        self.call_service(self.add_inst_cli, req)[cite: 6]
        
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