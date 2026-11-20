#!/usr/bin/env python3
"""Consistency check for config/ontology.yaml (tutorial step 2).

Usage: python3 tools/check_ontology.py src/go2_patrol_mission/config/ontology.yaml
Prints OK, or the list of errors (exit code 1).
"""
import sys
from collections import Counter

import yaml

EXPECTED_NODES = None   # any number of nodes; the tree check needs edges == nodes - 1


def duplicate_keys_loader():
    """SafeLoader that rejects duplicated keys (plain yaml.safe_load silently keeps the last)."""
    class Loader(yaml.SafeLoader):
        pass

    def construct_mapping(loader, node, deep=False):
        keys = [loader.construct_object(k, deep=deep) for k, _ in node.value]
        dups = [k for k, n in Counter(keys).items() if n > 1]
        if dups:
            raise yaml.constructor.ConstructorError(
                None, None, f'ID duplicati: {dups}', node.start_mark)
        return loader.construct_mapping(node, deep)

    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)
    return Loader


def check(onto):
    errors = []
    hubs = onto.get('hubs') or {}
    places = onto.get('places') or {}
    classes = onto.get('classes') or {}
    edges = onto.get('edges') or []

    # Unique IDs across hubs and places
    clash = set(hubs) & set(places)
    if clash:
        errors.append(f'ID usati sia come hub sia come luogo: {sorted(clash)}')
    nodes = set(hubs) | set(places)

    if EXPECTED_NODES is not None and len(nodes) != EXPECTED_NODES:
        errors.append(f'nodi: {len(nodes)} (attesi {EXPECTED_NODES})')
    if len(edges) != len(nodes) - 1:
        errors.append(f'archi: {len(edges)} (attesi {len(nodes) - 1} per {len(nodes)} nodi)')
    # Doors point to an existing office, at most one door per office
    door_of = [p.get('door_of') for p in places.values() if p.get('door_of')]
    for name, p in places.items():
        if p.get('door_of') and p['door_of'] not in places:
            errors.append(f'{name}: door_of "{p["door_of"]}" non è un luogo')
    for office, n in Counter(door_of).items():
        if n > 1:
            errors.append(f'{office}: {n} porte (al massimo 1)')

    # Edge endpoints exist, no self loops, no duplicate edges
    seen = set()
    adjacency = {n: set() for n in nodes}
    for e in edges:
        if not isinstance(e, list) or len(e) not in (2, 3):
            errors.append(f'arco malformato: {e}')
            continue
        a, b = e[0], e[1]
        for end in (a, b):
            if end not in nodes:
                errors.append(f'arco {a}-{b}: estremo inesistente "{end}"')
        if a == b:
            errors.append(f'arco su se stesso: {a}')
        key = frozenset((a, b))
        if key in seen:
            errors.append(f'arco duplicato: {a}-{b}')
        seen.add(key)
        if a in adjacency and b in adjacency:
            adjacency[a].add(b)
            adjacency[b].add(a)

    # Tree = connected with exactly N-1 edges
    if nodes:
        start = next(iter(nodes))
        visited, stack = {start}, [start]
        while stack:
            for nxt in adjacency[stack.pop()]:
                if nxt not in visited:
                    visited.add(nxt)
                    stack.append(nxt)
        if visited != nodes:
            errors.append(f'grafo non connesso: irraggiungibili {sorted(nodes - visited)}')
        if len(seen) != len(nodes) - 1:
            errors.append(f'non è un albero: {len(seen)} archi distinti per {len(nodes)} nodi')

    # Places: entrance exists and is a hub, exactly one edge towards a hub (the entrance), class exists
    home = onto.get('home')
    if home not in places:
        errors.append(f'home "{home}" non è un luogo')
    door_of_room = {p['door_of']: n for n, p in places.items() if p.get('door_of')}
    for name, p in places.items():
        entrance = p.get('entrance')
        neighbours = adjacency.get(name, set())
        if name in door_of_room:
            # A room with a door on the corridor is reached only through that door
            if entrance != door_of_room[name]:
                errors.append(f'{name}: entrance deve essere la sua porta {door_of_room[name]}')
            if neighbours != {door_of_room[name]}:
                errors.append(f'{name}: deve essere collegata solo alla sua porta, ora {sorted(neighbours)}')
        elif p.get('door_of'):
            # A door lies on the corridor: besides its room, 1 or 2 corridor neighbours (hubs or doors)
            corridor = [n for n in neighbours if n in hubs or places.get(n, {}).get('door_of')]
            if len(corridor) not in (1, 2):
                errors.append(f'{name}: {len(corridor)} vicini sul corridoio (attesi 1 o 2): {corridor}')
            if p['door_of'] not in neighbours:
                errors.append(f'{name}: non è collegata alla sua stanza {p["door_of"]}')
            if entrance not in hubs:
                errors.append(f'{name}: entrance "{entrance}" non è un hub esistente')
        else:
            # A room without a door: exactly one edge, to its entrance hub
            if entrance not in hubs:
                errors.append(f'{name}: entrance "{entrance}" non è un hub esistente')
            hub_edges = [n for n in neighbours if n in hubs]
            if len(hub_edges) != 1:
                errors.append(f'{name}: {len(hub_edges)} archi verso un hub (atteso 1): {hub_edges}')
            elif entrance not in hub_edges:
                errors.append(f'{name}: entrance {entrance} non è tra gli hub collegati {hub_edges}')
        if p.get('class') not in classes:
            errors.append(f'{name}: classe "{p.get("class")}" non definita in classes')
        if 'pose' not in p:
            errors.append(f'{name}: manca la pose')
    for name, h in hubs.items():
        if not all(k in h for k in ('x', 'y', 'theta')):
            errors.append(f'hub {name}: pose incompleta')
    return errors


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    try:
        with open(sys.argv[1]) as f:
            onto = yaml.load(f, Loader=duplicate_keys_loader())
    except (OSError, yaml.YAMLError) as e:
        print(f'ERRORE nel leggere il file: {e}')
        return 1
    errors = check(onto)
    if errors:
        print(f'{len(errors)} errori:')
        for e in errors:
            print(f'  - {e}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
