#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import re
import json
import urllib.request
import urllib.error

from go2_interfaces.srv import GemmaCommand
from plansys2_msgs.msg import Node as PlanNode, Param, Tree
from plansys2_msgs.srv import AffectParam, AffectNode, AddProblemGoal, ClearProblemKnowledge, GetPlan
from plansys2_msgs.action import ExecutePlan

class GemmaBridge(Node):
    def __init__(self):
        super().__init__('gemma_bridge')
        
        self.srv = self.create_service(
            GemmaCommand, 
            '/gemma_interface/send_command', 
            self.command_callback
        )
        
        # Clients for PlanSys2 problem expert
        self.clear_kb_cli = self.create_client(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge')
        self.add_inst_cli = self.create_client(AffectParam, '/problem_expert/add_problem_instance')
        self.add_pred_cli = self.create_client(AffectNode, '/problem_expert/add_problem_predicate')
        self.set_goal_cli = self.create_client(AddProblemGoal, '/problem_expert/set_problem_goal')
        self.get_plan_cli = self.create_client(GetPlan, '/planner/get_plan')
        
        # Action client to execute plan
        self.exec_plan_cli = ActionClient(self, ExecutePlan, '/execute_plan')
        
        self.get_logger().info("Gemma Bridge started. Waiting for commands on /gemma_interface/send_command")

    def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {client.srv_name} not available, waiting again...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def command_callback(self, request, response):
        command_text = request.command.lower()
        self.get_logger().info(f"Received command: {command_text}")
        
        # In a real scenario, this is where we would call the Gemma LLM to extract rooms.
        # Example API call structure (commented out as fallback is used):
        # try:
        #     payload = {"prompt": "Extract rooms from: " + command_text}
        #     req = urllib.request.Request("http://localhost:8000/v1", data=json.dumps(payload).encode(), method="POST")
        #     with urllib.request.urlopen(req, timeout=5.0) as resp:
        #         # parse LLM JSON...
        # except Exception as e: ...
        
        # 1. Regex Fallback Parser for our 3 simulated rooms
        target_rooms = []
        for room in ['lab_robotics', 'corridor', 'openspace']:
            if room in command_text:
                target_rooms.append(room)
                
        if not target_rooms:
            response.success = False
            response.message = "No known rooms (lab_robotics, corridor, openspace) found in the command."
            return response
            
        self.get_logger().info(f"Parsed target rooms: {target_rooms}")

        # 2. Setup PlanSys2 Problem
        self.setup_plansys2_problem(target_rooms)
        
        # 3. Get Plan
        plan_req = GetPlan.Request()
        plan_req.domain = 'patrol'
        plan_req.problem = 'patrol_problem'
        plan_res = self.call_service(self.get_plan_cli, plan_req)
        
        if not plan_res.success:
            response.success = False
            response.message = "Failed to generate a plan from PlanSys2."
            return response
            
        plan_str = "\\n".join([f"{item.time}: {item.action} [{item.duration}]" for item in plan_res.plan.items])
        self.get_logger().info(f"Generated Plan:\\n{plan_str}")
        
        # 4. Execute Plan asynchronously
        self.get_logger().info("Sending plan to executor...")
        exec_goal = ExecutePlan.Goal()
        self.exec_plan_cli.wait_for_server()
        self.exec_plan_cli.send_goal_async(exec_goal)
        
        response.success = True
        response.message = f"Plan generated for rooms: {', '.join(target_rooms)} and execution started."
        response.plan = plan_str
        return response

    def setup_plansys2_problem(self, target_rooms):
        # Clear existing knowledge
        self.call_service(self.clear_kb_cli, ClearProblemKnowledge.Request())
        
        # Add instances
        self.add_instance('go2', 'robot')
        self.add_instance('lab_robotics', 'room')
        self.add_instance('corridor', 'room')
        self.add_instance('openspace', 'room')
        
        # Add initial state
        self.add_predicate('robot_at', [('go2', 'robot'), ('lab_robotics', 'room')])
        
        # Add connectivity: lab_robotics -> corridor -> openspace
        self.add_predicate('connected', [('lab_robotics', 'room'), ('corridor', 'room')])
        self.add_predicate('connected', [('corridor', 'room'), ('openspace', 'room')])
        # Add bidirectional for return paths
        self.add_predicate('connected', [('corridor', 'room'), ('lab_robotics', 'room')])
        self.add_predicate('connected', [('openspace', 'room'), ('corridor', 'room')])
        
        # Setup goal tree
        goal_req = AddProblemGoal.Request()
        tree = Tree()
        
        if len(target_rooms) == 1:
            # Single goal predicate
            node = PlanNode(node_type=PlanNode.PREDICATE, name='inspected', parameters=[Param(name=target_rooms[0], type='room')])
            tree.nodes = [node]
        else:
            # AND goal
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
        self.call_service(self.add_inst_cli, req)
        
    def add_predicate(self, name, params):
        req = AffectNode.Request()
        req.node.node_type = PlanNode.PREDICATE
        req.node.name = name
        for p_name, p_type in params:
            param = Param(name=p_name, type=p_type)
            req.node.parameters.append(param)
        self.call_service(self.add_pred_cli, req)

def main(args=None):
    rclpy.init(args=args)
    bridge = GemmaBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
