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

# Import per interrogare lo stato di attivazione dei Lifecycle Nodes
from lifecycle_msgs.srv import GetState

from go2_interfaces.srv import GemmaCommand
from plansys2_msgs.msg import Node as PlanNode, Param, Tree
from plansys2_msgs.srv import (
    AffectParam, AffectNode, AddProblemGoal, ClearProblemKnowledge, 
    GetPlan, GetDomain, GetProblem
)
from plansys2_msgs.action import ExecutePlan

class GemmaBridge(Node):
    def __init__(self):
        super().__init__('gemma_bridge')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.srv = self.create_service(
            GemmaCommand, 
            '/gemma_interface/send_command', 
            self.command_callback,
            callback_group=self.cb_group
        )
        
        # Client per monitorare se PlanSys2 è pronto ed evitare crash
        self.get_state_cli = self.create_client(GetState, '/problem_expert/get_state', callback_group=self.cb_group)
        
        # Client per recuperare il PDDL reale del dominio e del problema
        self.get_domain_cli = self.create_client(GetDomain, '/domain_expert/get_domain', callback_group=self.cb_group)
        self.get_problem_cli = self.create_client(GetProblem, '/problem_expert/get_problem', callback_group=self.cb_group)
        
        # Client PlanSys2 standard
        self.clear_kb_cli = self.create_client(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge', callback_group=self.cb_group)
        self.add_inst_cli = self.create_client(AffectParam, '/problem_expert/add_problem_instance', callback_group=self.cb_group)
        self.add_pred_cli = self.create_client(AffectNode, '/problem_expert/add_problem_predicate', callback_group=self.cb_group)
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

    def command_callback(self, request, response):
        command_text = request.command.lower()
        self.get_logger().info(f"Received command: {command_text}")
        
        # 1. Regex Fallback Parser
        target_rooms = []
        for room in ['lab_robotics', 'corridor', 'openspace']:
            if room in command_text:
                target_rooms.append(room)
                
        if not target_rooms:
            response.success = False
            response.message = "No known rooms (lab_robotics, corridor, openspace) found in the command."
            return response
            
        self.get_logger().info(f"Parsed target rooms: {target_rooms}")

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