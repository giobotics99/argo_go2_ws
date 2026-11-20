#!/usr/bin/env python3
"""Generate a PDDL problem for patrol_domain_v2 from ontology.yaml and a mission description.

Mission description (YAML):
    id: p01                       # problem name
    mission: message              # coffee_call | message | inspect | home
    targets: [vito_room]          # place IDs (ignored for "home")
    ordered: false                # true: visit targets in the given order (precedes chain)
    battery: 100                  # % (default 100)
    start: lab_robotics           # initial node (default: ontology home)
    blocked: [[hub_center, hub_junction]]   # edges without path_free, both ways (default none)
    reserve: 15                   # optional override of the ontology battery reserve (0 = R1 trip home)

Usage:
    python3 tools/gen_problem.py ONTOLOGY MISSION.yaml [-o OUT.pddl]
The same rules are the reference for the KB loader of argo_mission_bt (tutorial step 10).
"""
import argparse
import sys

import yaml

INSPECT_TIME_PER_VIEWPOINT = 20  # s


def fmt(v):
    """Numbers without useless decimals (PDDL accepts both)."""
    return str(int(v)) if float(v).is_integer() else str(v)


def build_problem(onto, mission):
    """Problem as PDDL text."""
    name, header, rid, nodes, init, goal = build_facts(onto, mission)
    lines = [header,
             f'(define (problem {name}) (:domain argo_office)',
             f'  (:objects {rid} - robot',
             f'            {" ".join(nodes)} - location)',
             '  (:init']
    lines += [f'    {fact}' for fact in init]
    lines += ['  )', '  (:goal (and']
    lines += [f'    {g}' for g in goal]
    lines += ['  ))', ')']
    return '\n'.join(lines) + '\n'


def build_terminal(onto, mission):
    """Same facts as plansys2_terminal commands (paste them at the "> " prompt)."""
    name, header, rid, nodes, init, goal = build_facts(onto, mission)
    cmds = [f'set instance {rid} robot'] + [f'set instance {n} location' for n in nodes]
    for fact in init:
        cmds.append(f'set function {fact}' if fact.startswith('(=') else f'set predicate {fact}')
    cmds.append(f'set goal (and {" ".join(goal)})')
    return '\n'.join(cmds) + '\n'


def build_facts(onto, mission):
    hubs, places, classes = onto['hubs'], onto['places'], onto['classes']
    robot = onto['robot']
    rid = robot['id']
    home = onto['home']
    nodes = list(hubs) + list(places)

    kind = mission['mission']
    targets = list(mission.get('targets') or [])
    # Offices are served from their door (privacy); the other rooms are entered through it
    doors = {p['door_of']: name for name, p in places.items()
             if p.get('door_of') and places.get(p['door_of'], {}).get('class') == 'Office'}
    if kind in ('message', 'coffee_call'):
        targets = [doors.get(t, t) for t in targets]
    ordered = bool(mission.get('ordered', False))
    start = mission.get('start', home)
    battery = mission.get('battery', 100)
    reserve = mission.get('reserve', robot.get('reserve', 0))
    blocked = {frozenset(e) for e in (mission.get('blocked') or [])}

    for t in targets + [start]:
        if t not in nodes:
            raise ValueError(f'nodo sconosciuto: {t}')
    if kind not in ('coffee_call', 'message', 'inspect', 'home'):
        raise ValueError(f'missione sconosciuta: {kind}')

    init = [f'(robot_at {rid} {start})']
    # Topology and distances (both ways); path_free unless blocked
    for a, b, d in onto['edges']:
        for x, y in ((a, b), (b, a)):
            init.append(f'(connected {x} {y})')
            if frozenset((a, b)) not in blocked:
                init.append(f'(path_free {x} {y})')
            init.append(f'(= (distance {x} {y}) {fmt(d)})')
    # Rigid properties of places
    for name, p in places.items():
        if classes.get(p['class'], {}).get('camera_allowed', False):
            init.append(f'(camera_allowed {name})')
        if p.get('needs_permission', False):
            init.append(f'(needs_permission {name})')
        if p.get('dock', False):
            init.append(f'(is_dock {name})')
        if p.get('social_spot', False):
            init.append(f'(social_spot {name})')
    # may_enter everywhere except places that need permission (hubs included)
    for n in nodes:
        if not places.get(n, {}).get('needs_permission', False):
            init.append(f'(may_enter {n})')
    # Numeric facts
    init += [f'(= (speed {rid}) {fmt(robot["speed"])})',
             f'(= (drain {rid}) {fmt(robot["drain"])})',
             f'(= (battery {rid}) {fmt(battery)})',
             f'(= (reserve {rid}) {fmt(reserve)})',
             f'(= (chat_time {rid}) {fmt(onto["chat_time"])})']
    for n in nodes:  # defined for every location, so no action is pruned for a missing value
        vps = len(places.get(n, {}).get('viewpoints') or [])
        init.append(f'(= (inspect_time {n}) {INSPECT_TIME_PER_VIEWPOINT * max(1, vps)})')

    # Mission facts and goal (every mission ends at home)
    goal = []
    to_call = 0
    if kind == 'coffee_call':
        for t in targets:
            init.append(f'(needs_call {t})')
            goal.append(f'(announced {t})')
        to_call = len(targets)
        social = [n for n, p in places.items() if p.get('social_spot', False)]
        goal += [f'(socialized {s})' for s in social]
    elif kind in ('message', 'inspect'):
        if ordered and len(targets) > 1:
            init.append(f'(unlocked {targets[0]})')
            init += [f'(precedes {a} {b})' for a, b in zip(targets, targets[1:])]
        else:
            init += [f'(unlocked {t})' for t in targets]
        for t in targets:
            if kind == 'message':
                init.append(f'(has_message {t})')
                goal.append(f'(message_delivered {t})')
            else:
                goal.append(f'(inspected {t})')
    init.append(f'(= (to_call {rid}) {to_call})')
    goal.append(f'(robot_at {rid} {home})')

    name = mission.get('id', 'argo_problem')
    header = (f'; generated by tools/gen_problem.py: {kind} {targets} ordered={ordered} '
              f'battery={battery} reserve={reserve} start={start} blocked={[sorted(b) for b in blocked]}')
    return name, header, rid, nodes, init, goal


def main():
    parser = argparse.ArgumentParser(description='Problem PDDL per patrol_domain_v2')
    parser.add_argument('ontology')
    parser.add_argument('mission')
    parser.add_argument('-o', '--output', help='file di uscita (default: stdout)')
    parser.add_argument('--terminal', action='store_true',
                        help='comandi per plansys2_terminal invece del problem PDDL')
    args = parser.parse_args()
    with open(args.ontology) as f:
        onto = yaml.safe_load(f)
    with open(args.mission) as f:
        mission = yaml.safe_load(f)
    text = build_terminal(onto, mission) if args.terminal else build_problem(onto, mission)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
