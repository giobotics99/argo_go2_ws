#!/usr/bin/env python3
"""Run one (or a few) PDDL actions through PlanSys2, on a given knowledge base.

plansys2_terminal on Foxy cannot run a single action ("run (action ...)" is a ToDo in its code):
this tool loads the KB from a problem file, then asks the executor to run a plan made only of the
given actions, in order. Needs plansys2_v2_check.launch.py (or the full v2 launch) running.

Usage:
  python3 tools/run_action.py src/go2_patrol_mission/pddl/problems/p01_message_vito.pddl \
      "(navigate go2 lab_robotics hub_west)"
  python3 tools/run_action.py PROBLEM.pddl "(navigate go2 lab_robotics hub_west)" "(navigate go2 hub_west vito_room)"
The problem only sets the starting state (robot position, battery, ...); its goal is ignored.
"""
import sys

import rclpy
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ActionExecutionInfo, Plan, PlanItem
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetStates
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {ActionExecutionInfo.NOT_EXECUTED: 'NOT_EXECUTED', ActionExecutionInfo.EXECUTING: 'EXECUTING',
          ActionExecutionInfo.FAILED: 'FAILED', ActionExecutionInfo.SUCCEEDED: 'SUCCEEDED',
          ActionExecutionInfo.CANCELLED: 'CANCELLED'}
ACTION_TIME = 120.0   # nominal duration of each action in the one-off plan (s)


def call(node, cli, req, timeout=30.0):
    if not cli.wait_for_service(timeout_sec=15.0):
        raise RuntimeError(f'servizio {cli.srv_name} non disponibile: PlanSys2 è acceso?')
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None:
        raise RuntimeError(f'nessuna risposta da {cli.srv_name}')
    return fut.result()


def robot_state(node, cli):
    preds = call(node, cli, GetStates.Request()).states
    return [f"({p.name} {' '.join(x.name for x in p.parameters)})" for p in preds if p.name == 'robot_at']


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    with open(sys.argv[1]) as f:
        problem = f.read()
    actions = sys.argv[2:]

    rclpy.init()
    node = Node('run_action')
    preds_cli = node.create_client(GetStates, '/problem_expert/get_problem_predicates')
    try:
        call(node, node.create_client(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge'),
             ClearProblemKnowledge.Request())
        res = call(node, node.create_client(AddProblem, '/problem_expert/add_problem'), AddProblem.Request(problem=problem))
        if not res.success:
            print(f'add_problem fallito: {res.error_info}')
            return 1
        print(f'KB caricato da {sys.argv[1]}; stato iniziale: {robot_state(node, preds_cli)}')

        plan = Plan(items=[PlanItem(time=i * (ACTION_TIME + 0.01), action=a, duration=ACTION_TIME)
                           for i, a in enumerate(actions)])
        client = ActionClient(node, ExecutePlan, '/execute_plan')
        if not client.wait_for_server(timeout_sec=15.0):
            print('execute_plan non disponibile')
            return 1
        last = {}

        def on_feedback(msg):
            for info in msg.feedback.action_execution_status:
                if last.get(info.action_full_name) != info.status:
                    last[info.action_full_name] = info.status
                    print(f'  {STATUS.get(info.status, info.status):9s} {info.action_full_name} {info.message_status}')

        goal = client.send_goal_async(ExecutePlan.Goal(plan=plan), feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, goal, timeout_sec=15.0)
        handle = goal.result()
        if handle is None or not handle.accepted:
            print('piano rifiutato dall\'executor')
            return 1
        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result)
        except KeyboardInterrupt:
            print('\nCtrl+C: annullo il piano')
            handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=2.0)
            return 1
        out = result.result().result
        print(f'risultato: success={out.success}')
        for info in out.action_execution_status:
            print(f'  {STATUS.get(info.status, info.status):9s} {info.action_full_name} {info.message_status}')
        print(f'stato finale: {robot_state(node, preds_cli)}')
        return 0 if out.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
