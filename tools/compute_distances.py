#!/usr/bin/env python3
"""Compute real path lengths for the ontology edges from the occupancy map (tutorial step 3).

For every edge [a, b, d] in ontology.yaml:
  - free cells are those >= 250 in the map image; obstacles (and unknown) are inflated by 0.25 m;
  - a and b are snapped to the nearest free cell (warning if farther than 0.3 m);
  - Dijkstra on the 8-connected grid gives the path length, written back as d (1 decimal).
Only the edge lines of ontology.yaml are rewritten: comments and layout stay as they are.
Also checks the tree property: removing any hub-hub edge must split the graph in two.

Usage:
    python3 tools/compute_distances.py src/go2_patrol_mission/config/ontology.yaml [--dry-run]
Only numpy and PIL (scipy is not used, on purpose).
"""
import argparse
import heapq
import math
import os
import re
import sys

import numpy as np
import yaml
from PIL import Image

WS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_MAP = os.path.join(WS, 'src', 'go2_nav', 'maps', 'institute_stiima_cnr.yaml')
INFLATION = 0.25     # m
SNAP_WARN = 0.30     # m
FREE_THRESHOLD = 250
EDGE_RE = re.compile(r'^(\s*-\s*\[\s*)([\w]+)(\s*,\s*)([\w]+)(\s*,\s*)([0-9.]+)(\s*\].*)$')


def load_map(map_yaml):
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(map_yaml), image)
    img = np.array(Image.open(image).convert('L'))
    if meta.get('negate', 0):
        img = 255 - img
    return img, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1]), image


def inflate(free, radius_cells):
    """Grow the non-free area by a disc of radius_cells (numpy shifts, no scipy)."""
    blocked = ~free
    grown = blocked.copy()
    h, w = blocked.shape
    for dv in range(-radius_cells, radius_cells + 1):
        for du in range(-radius_cells, radius_cells + 1):
            if (dv, du) == (0, 0) or dv * dv + du * du > radius_cells * radius_cells:
                continue
            src = blocked[max(0, -dv):h - max(0, dv), max(0, -du):w - max(0, du)]
            grown[max(0, dv):h - max(0, -dv), max(0, du):w - max(0, -du)] |= src
    return ~grown


def dijkstra(free, start, goal, res):
    """Shortest 8-connected path length (m) from start to goal cell, or None."""
    h, w = free.shape
    diag = res * math.sqrt(2.0)
    steps = [(-1, 0, res), (1, 0, res), (0, -1, res), (0, 1, res),
             (-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag)]
    dist = np.full(free.shape, np.inf)
    dist[start] = 0.0
    heap = [(0.0, start)]
    while heap:
        d, (v, u) = heapq.heappop(heap)
        if (v, u) == goal:
            return d
        if d > dist[v, u]:
            continue
        for dv, du, c in steps:
            nv, nu = v + dv, u + du
            if 0 <= nv < h and 0 <= nu < w and free[nv, nu]:
                nd = d + c
                if nd < dist[nv, nu]:
                    dist[nv, nu] = nd
                    heapq.heappush(heap, (nd, (nv, nu)))
    return None


def tree_check(nodes, edges, hubs):
    """Removing each hub-hub edge must leave exactly two connected components."""
    problems = []
    for a, b in edges:
        if a not in hubs or b not in hubs:
            continue
        adj = {n: set() for n in nodes}
        for x, y in edges:
            if {x, y} != {a, b}:
                adj[x].add(y)
                adj[y].add(x)
        seen, comps = set(), 0
        for n in nodes:
            if n in seen:
                continue
            comps += 1
            stack = [n]
            seen.add(n)
            while stack:
                for m in adj[stack.pop()]:
                    if m not in seen:
                        seen.add(m)
                        stack.append(m)
        if comps != 2:
            problems.append(f'togliendo {a}-{b} restano {comps} componenti (attese 2): c\'è un anello')
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('ontology')
    parser.add_argument('--map', default=DEFAULT_MAP, help='map yaml (default institute_stiima_cnr)')
    parser.add_argument('--dry-run', action='store_true', help='print only, do not write the ontology')
    args = parser.parse_args()

    with open(args.ontology) as f:
        text = f.read()
    onto = yaml.safe_load(text)
    hubs, places = onto['hubs'], onto['places']
    poses = {n: (h['x'], h['y']) for n, h in hubs.items()}
    poses.update({n: (p['pose']['x'], p['pose']['y']) for n, p in places.items()})

    img, res, ox, oy, image_path = load_map(args.map)
    h, w = img.shape
    free = inflate(img >= FREE_THRESHOLD, int(round(INFLATION / res)))
    print(f'Mappa {os.path.basename(image_path)} {w}x{h}, res {res} m, origine ({ox}, {oy}), '
          f'ostacoli gonfiati di {INFLATION} m')

    free_cells = np.argwhere(free)

    def to_cell(x, y):
        return h - 1 - int((y - oy) / res), int((x - ox) / res)

    # Snap every node to the nearest free cell
    snapped, warnings = {}, []
    for name, (x, y) in poses.items():
        v, u = to_cell(x, y)
        d = np.hypot(free_cells[:, 0] - v, free_cells[:, 1] - u)
        i = int(np.argmin(d))
        snapped[name] = (int(free_cells[i, 0]), int(free_cells[i, 1]))
        if d[i] * res > SNAP_WARN:
            warnings.append(f'{name} agganciato a {d[i] * res:.2f} m dalla sua posa (> {SNAP_WARN} m)')

    # Path length for every edge
    edges = [(e[0], e[1]) for e in onto['edges']]
    old = {(e[0], e[1]): e[2] if len(e) > 2 else None for e in onto['edges']}
    new, errors = {}, []
    print(f'\n{"arco":42s} {"prima":>6s} {"ora":>6s}')
    for a, b in edges:
        length = dijkstra(free, snapped[a], snapped[b], res)
        if length is None:
            errors.append(f'{a}-{b}: nessun percorso sulla mappa gonfiata')
            print(f'{a + " - " + b:42s} {old[(a, b)]!s:>6s}   ----  NON RAGGIUNGIBILE')
            continue
        new[(a, b)] = round(length, 1)
        print(f'{a + " - " + b:42s} {old[(a, b)]!s:>6s} {new[(a, b)]:6.1f}')

    errors += tree_check(set(poses), edges, set(hubs))
    for wmsg in warnings:
        print(f'ATTENZIONE: {wmsg}')
    if errors:
        print('\nERRORI:')
        for e in errors:
            print(f'  - {e}')
        return 1
    print('\nControllo "niente anelli": OK (ogni arco tra hub divide l\'albero in due)')

    if args.dry_run:
        print('--dry-run: ontology non modificata')
        return 0
    out, changed = [], 0
    for line in text.splitlines(keepends=True):
        m = EDGE_RE.match(line.rstrip('\n'))
        if m and (m.group(2), m.group(4)) in new:
            value = new[(m.group(2), m.group(4))]
            line = f'{m.group(1)}{m.group(2)}{m.group(3)}{m.group(4)}{m.group(5)}{value}{m.group(7)}\n'
            changed += 1
        elif line.lstrip().startswith('# Tree edges'):
            line = '# Tree edges (undirected). Distances in metres from tools/compute_distances.py\n'
        out.append(line)
    if changed != len(edges):
        print(f'ERRORE: aggiornate {changed} righe di archi su {len(edges)}: ontology non scritta')
        return 1
    with open(args.ontology, 'w') as f:
        f.writelines(out)
    print(f'Scritte {changed} distanze in {args.ontology}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
