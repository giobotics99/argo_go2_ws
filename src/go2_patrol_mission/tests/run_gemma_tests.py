#!/usr/bin/env python3
"""Run test set B against gemma_server (tutorial step 7).

Prints accuracy per category, median and 95th-percentile latency, fallback rate, and every failure.
Needs gemma_server.py running:  ros2 run go2_patrol_mission gemma_server.py

Usage: python3 src/go2_patrol_mission/tests/run_gemma_tests.py [--cases FILE] [--only CAT]
"""
import argparse
import json
import os
import re
import statistics
import time
import unicodedata

import rclpy
import yaml
from go2_interfaces.srv import FormulateGoals, Speak
from rclpy.node import Node

HERE = os.path.dirname(os.path.realpath(__file__))
ONTOLOGY = os.path.join(HERE, '..', 'config', 'ontology.yaml')
NOT_ITALIAN = re.compile(r'\b(você|não|está|usted|the|and|you)\b|[¿¡ãõ]', re.IGNORECASE)


def fold(text):
    return unicodedata.normalize('NFKD', str(text)).encode('ascii', 'ignore').decode().lower()


def check_goals(expect, res, works_here):
    """'' if the answer matches the expectation, otherwise the reason."""
    if expect == 'clarify':
        return '' if (not res.accepted and res.clarify_question) else 'attesa una domanda di chiarimento'
    if expect == 'reject':
        return '' if (not res.accepted and not res.targets) else 'atteso un rifiuto'
    if not res.accepted:
        return f'rifiutato/chiarimento: "{res.reply or res.clarify_question}"'
    if res.mission != expect['mission']:
        return f'missione {res.mission} invece di {expect["mission"]}'
    want = works_here if expect['targets'] == 'ALL' else list(expect['targets'])
    got = list(res.targets)
    if expect.get('ordered'):
        if not res.ordered or got != want:
            return f'ordine: {got} (ordered={res.ordered}) invece di {want}'
    elif set(got) != set(want):
        return f'stanze {sorted(got)} invece di {sorted(want)}'
    if 'message' in expect and fold(expect['message']) not in fold(res.message_text):
        return f'messaggio "{res.message_text}" senza "{expect["message"]}"'
    return ''


def check_speak(expect, res):
    s = res.sentence
    if not s:
        return 'frase vuota'
    if len(s.split()) > 25:
        return f'{len(s.split())} parole'
    if NOT_ITALIAN.search(s):
        return 'non in italiano'
    missing = [f for f in expect['speak'] if fold(f) not in fold(s)]
    return f'mancano {missing}' if missing else ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cases', default=os.path.join(HERE, 'gemma_cases.yaml'))
    parser.add_argument('--only', help='solo una categoria')
    args = parser.parse_args()
    with open(args.cases) as f:
        cases = yaml.safe_load(f)['cases']
    with open(ONTOLOGY) as f:
        onto = yaml.safe_load(f)
    works_here = [p for p, v in onto['places'].items() if v.get('works_here')]
    if args.only:
        cases = [c for c in cases if c['cat'] == args.only]

    rclpy.init()
    node = Node('gemma_tests')
    goals_cli = node.create_client(FormulateGoals, '/argo/gemma/formulate_goals')
    speak_cli = node.create_client(Speak, '/argo/gemma/speak')
    for cli in (goals_cli, speak_cli):
        if not cli.wait_for_service(timeout_sec=15.0):
            print(f'Servizio {cli.srv_name} non disponibile: gemma_server.py è acceso?')
            return 1

    per_cat, latencies, fallbacks, failures = {}, [], 0, []
    for i, case in enumerate(cases, 1):
        if case['cat'] == 'G5':
            cli, req = speak_cli, Speak.Request(event_type=case['event'],
                                               facts_json=json.dumps(case['facts'], ensure_ascii=False))
        else:
            cli, req = goals_cli, FormulateGoals.Request(command=case['command'], context_json='{}')
        t0 = time.time()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=60.0)
        dt = time.time() - t0
        res = fut.result()
        if res is None:
            why = 'nessuna risposta in 60 s'
        elif case['cat'] == 'G5':
            why = check_speak(case['expect'], res)
        else:
            why = check_goals(case['expect'], res, works_here)
        latencies.append(dt)
        fallbacks += int(bool(res is not None and res.used_fallback))
        ok, n = per_cat.get(case['cat'], (0, 0))
        per_cat[case['cat']] = (ok + (why == ''), n + 1)
        label = case.get('command') or f"{case['event']} {case['facts']}"
        fb = ' [fallback]' if res is not None and res.used_fallback else ''
        print(f"{i:2d} {'OK  ' if not why else 'FAIL'} {dt:5.1f}s {case['cat']:11s} {label[:60]}{fb}")
        if why:
            failures.append((case['cat'], label, why))

    print('\n=== accuratezza per categoria')
    for cat, (ok, n) in per_cat.items():
        print(f'  {cat:11s} {ok}/{n}  ({100 * ok / n:.0f}%)')
    total_ok = sum(ok for ok, _ in per_cat.values())
    print(f'  {"TOTALE":11s} {total_ok}/{len(cases)}  ({100 * total_ok / len(cases):.0f}%)')
    lat = sorted(latencies)
    p95 = lat[min(len(lat) - 1, int(round(0.95 * (len(lat) - 1))))]
    print(f'\n=== latenza: mediana {statistics.median(lat):.1f} s, 95° percentile {p95:.1f} s')
    print(f'=== fallback: {fallbacks}/{len(cases)} ({100 * fallbacks / len(cases):.0f}%)')
    if failures:
        print('\n=== casi falliti')
        for cat, label, why in failures:
            print(f'  [{cat}] {label}: {why}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
