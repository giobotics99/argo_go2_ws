#!/usr/bin/env python3
"""Reference optimum (metres) for each test mission, to judge the quality of POPF's plans.

Exact search over visit orders on the topological model of ontology.yaml (shortest paths on the
tree of edges, blocked edges removed). Battery, recharge and waiting times are ignored: this is the
minimum distance the robot must walk.
  - message / inspect: start -> every target (in the given order if ordered) -> home
  - coffee_call:       start -> every target (any order) -> social spot -> home
  - home:              start -> home
Held-Karp dynamic programming, so the 10 rooms of P3 are instant.

Usage: python3 tools/optimal_tours.py ONTOLOGY MISSION.yaml [MISSION.yaml ...]
Prints "id metres" per mission ("-" if some target is unreachable).
"""
import sys
from itertools import combinations

import yaml

INF = float('inf')


def shortest_paths(onto, blocked):
    nodes = list(onto['hubs']) + list(onto['places'])
    d = {a: {b: (0.0 if a == b else INF) for b in nodes} for a in nodes}
    for a, b, w in onto['edges']:
        if frozenset((a, b)) not in blocked:
            d[a][b] = d[b][a] = float(w)
    for k in nodes:  # Floyd-Warshall, 17 nodes
        for i in nodes:
            for j in nodes:
                if d[i][k] + d[k][j] < d[i][j]:
                    d[i][j] = d[i][k] + d[k][j]
    return d


def best_tour(d, start, targets, end):
    """Min length start -> all targets (any order) -> end, Held-Karp."""
    if not targets:
        return d[start][end]
    n = len(targets)
    best = {}
    for i, t in enumerate(targets):
        best[(1 << i, i)] = d[start][t]
    for size in range(2, n + 1):
        for subset in combinations(range(n), size):
            mask = sum(1 << i for i in subset)
            for j in subset:
                prev = mask & ~(1 << j)
                best[(mask, j)] = min(best[(prev, k)] + d[targets[k]][targets[j]]
                                      for k in subset if k != j)
    full = (1 << n) - 1
    return min(best[(full, j)] + d[targets[j]][end] for j in range(n))


def optimum(onto, mission):
    home = onto['home']
    start = mission.get('start', home)
    targets = list(mission.get('targets') or [])
    # Offices are served from their door, as in gen_problem.py
    doors = {p['door_of']: n for n, p in onto['places'].items()
             if p.get('door_of') and onto['places'].get(p['door_of'], {}).get('class') == 'Office'}
    if mission['mission'] in ('message', 'coffee_call'):
        targets = [doors.get(t, t) for t in targets]
    blocked = {frozenset(e) for e in (mission.get('blocked') or [])}
    d = shortest_paths(onto, blocked)
    kind = mission['mission']
    if kind == 'home':
        total = d[start][home]
    elif kind == 'coffee_call':
        social = [n for n, p in onto['places'].items() if p.get('social_spot', False)][0]
        # Visit all targets, then the social spot, then home (best over the last target)
        total = best_tour(d, start, targets, social) + d[social][home]
    elif mission.get('ordered', False):
        path = [start] + targets + [home]
        total = sum(d[a][b] for a, b in zip(path, path[1:]))
    else:
        total = best_tour(d, start, targets, home)
    return None if total == INF else round(total, 1)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    with open(sys.argv[1]) as f:
        onto = yaml.safe_load(f)
    for path in sys.argv[2:]:
        with open(path) as f:
            mission = yaml.safe_load(f)
        value = optimum(onto, mission)
        print(f"{mission.get('id', path)} {'-' if value is None else value}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
