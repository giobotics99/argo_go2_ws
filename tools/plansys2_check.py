#!/usr/bin/env python3
"""Run a problem through PlanSys2 end to end and answer the three step-5 questions.

Needs plansys2_v2_check.launch.py running (PlanSys2 + fake executors). It:
  1. loads the problem file into the problem expert (problem_expert/add_problem);
  2. reads back domain and problem (are fluents and functions with arguments accepted?);
  3. asks the planner for a plan and executes it (execute_plan action);
  4. reads (to_call go2) and (battery go2) before and after, and logs every action status.

Usage: python3 tools/plansys2_check.py src/go2_patrol_mission/pddl/problems/p02_coffee4.pddl
"""
import re
import sys
import time

import rclpy
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ActionExecutionInfo
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge, GetDomain, GetPlan, GetProblem, GetStates
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {ActionExecutionInfo.NOT_EXECUTED: 'NOT_EXECUTED', ActionExecutionInfo.EXECUTING: 'EXECUTING',
          ActionExecutionInfo.FAILED: 'FAILED', ActionExecutionInfo.SUCCEEDED: 'SUCCEEDED',
          ActionExecutionInfo.CANCELLED: 'CANCELLED'}


class Check(Node):

    def __init__(self):
        super().__init__('plansys2_v2_check')
        self.cli = {
            'clear': self.create_client(ClearProblemKnowledge, '/problem_expert/clear_problem_knowledge'),
            'add': self.create_client(AddProblem, '/problem_expert/add_problem'),
            'domain': self.create_client(GetDomain, '/domain_expert/get_domain'),
            'problem': self.create_client(GetProblem, '/problem_expert/get_problem'),
            'functions': self.create_client(GetStates, '/problem_expert/get_problem_functions'),
            'plan': self.create_client(GetPlan, '/planner/get_plan'),
        }
        self.exec_cli = ActionClient(self, ExecutePlan, '/execute_plan')

    def call(self, name, req, timeout=30.0):
        cli = self.cli[name]
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'servizio {cli.srv_name} non disponibile')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            raise RuntimeError(f'nessuna risposta da {cli.srv_name}')
        return fut.result()

    def functions(self):
        """{'(to_call go2)': 4.0, ...} from the problem expert."""
        res = self.call('functions', GetStates.Request())
        out = {}
        for f in res.states:
            out['(' + ' '.join([f.name] + [p.name for p in f.parameters]) + ')'] = f.value
        return out


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    with open(sys.argv[1]) as f:
        problem_text = f.read()

    rclpy.init()
    n = Check()
    try:
        # 1) load the problem
        n.call('clear', ClearProblemKnowledge.Request())
        res = n.call('add', AddProblem.Request(problem=problem_text))
        print(f'add_problem: success={res.success} {res.error_info}')
        if not res.success:
            return 1

        # 2) a) domain and problem read back
        domain = n.call('domain', GetDomain.Request()).domain
        problem = n.call('problem', GetProblem.Request()).problem
        fun_before = n.functions()
        has_fluents = ':fluents' in domain or ':numeric-fluents' in domain
        nested = '(+ (reserve ?r)' in domain.replace('\n', ' ') or 'reserve' in domain
        print('\n=== a) domain expert')
        print(f'  requirements nel dominio caricato: {re.findall(r"[(]:requirements[^)]*[)]", domain)}')
        print(f'  :fluents presente: {has_fluents}; funzione reserve presente: {nested}')
        print(f'  funzioni nel KB: {len(fun_before)} (es. {[k for k in fun_before if "go2" in k]})')
        print(f'  (to_call go2) = {fun_before.get("(to_call go2)")}, (battery go2) = {fun_before.get("(battery go2)")}')

        # 3) plan
        plan_res = n.call('plan', GetPlan.Request(domain=domain, problem=problem), timeout=90.0)
        print(f'\n=== piano: success={plan_res.success} {plan_res.error_info}')
        if not plan_res.success:
            return 1
        for item in plan_res.plan.items:
            print(f'  {item.time:8.3f}: {item.action} [{item.duration:.1f}]')

        # execute, logging status transitions
        if not n.exec_cli.wait_for_server(timeout_sec=10.0):
            print('execute_plan non disponibile')
            return 1
        last = {}
        to_call_at_chat = []

        def on_feedback(msg):
            for info in msg.feedback.action_execution_status:
                if last.get(info.action_full_name) != info.status:
                    last[info.action_full_name] = info.status
                    if info.status in (ActionExecutionInfo.EXECUTING, ActionExecutionInfo.SUCCEEDED,
                                       ActionExecutionInfo.FAILED):
                        print(f'  {time.strftime("%H:%M:%S")} {STATUS[info.status]:9s} {info.action_full_name}')
                    if info.action == 'chat' and info.status == ActionExecutionInfo.EXECUTING:
                        to_call_at_chat.append(True)

        print('\n=== esecuzione (executor finti)')
        goal_fut = n.exec_cli.send_goal_async(ExecutePlan.Goal(plan=plan_res.plan), feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(n, goal_fut, timeout_sec=10.0)
        handle = goal_fut.result()
        if handle is None or not handle.accepted:
            print('piano rifiutato dall\'executor')
            return 1
        result_fut = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(n, result_fut, timeout_sec=600.0)
        except KeyboardInterrupt:
            print('\nCtrl+C: annullo il piano (il goal Nav2 in corso viene annullato dall\'executor)')
            handle.cancel_goal_async()
            rclpy.spin_once(n, timeout_sec=2.0)
            return 1
        result = result_fut.result().result
        print(f'  risultato: success={result.success}')
        for info in result.action_execution_status:
            if info.status != ActionExecutionInfo.SUCCEEDED:
                print(f'  non riuscita: {info.action_full_name} {STATUS.get(info.status)} {info.message_status}')

        fun_after = n.functions()
        tc0, tc1 = fun_before.get('(to_call go2)'), fun_after.get('(to_call go2)')
        b0, b1 = fun_before.get('(battery go2)'), fun_after.get('(battery go2)')
        announces = sum(1 for i in plan_res.plan.items if i.action.startswith('(announce'))
        print('\n=== risposte')
        print(f'  a) dominio con :fluents e funzioni con argomento accettato: {has_fluents and len(fun_before) > 0}')
        print(f'  b) to_call prima {tc0} -> dopo {tc1} ({announces} announce); battery {b0} -> {b1}')
        numeric_ok = (tc1 is not None and tc0 is not None and tc1 < tc0) or \
            (b1 is not None and b0 is not None and b1 < b0)
        print(f'     effetti numerici applicati dall\'executor: {numeric_ok}')
        print(f'  c) chat partita: {bool(to_call_at_chat)} '
              f'({"SUCCEEDED" if last.get(next((k for k in last if "chat" in k), ""), 0) == ActionExecutionInfo.SUCCEEDED else "non riuscita"})')
        return 0 if result.success else 1
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
