# README_instruction — From design to workspace (Argo v2)

This is the step-by-step guide to turn the **Argo v2 design** into code inside `argo_go2_ws`.
The design comes first and lives in the design doc ("Argo v2 — Planning design").
**Do not start a phase before the previous one passes its check.**

> Status: guide only. Nothing here has been implemented or run yet. The current workspace
> (gemma_bridge + PlanSys2 + Nav2 in Docker + voice) keeps working until Phase 6 swaps it out.

---

## 0. What we are building

Argo is a social assistant robot for the STIIMA floor. It lives in the robotics lab and does
three **missions** on request, plus two **background behaviours**:

| ID | Mission / behaviour | Example | Planner goal |
|----|---------------------|---------|--------------|
| M1 | Coffee call | "Argo, chiama tutti per il caffè" | `announced` for every `works_here` room + `socialized coffee_break_area` + `robot_at go2 lab_robotics` |
| M2 | Messenger | "Argo, vai da Vito e digli che la riunione è alle 15" | `message_delivered vito_room` + `robot_at go2 lab_robotics` |
| M3 | Inspection | "Argo, ispeziona l'openspace e il vision lab" | `inspected` for each room + home |
| R1 | Low battery | SOC < 15 % | not a goal: the BT interrupts, says why, goes home, lies down |
| R2 | Sensor health | LiDAR / camera / localisation / Nav2 / Gemma down | not a goal: the BT says what failed and degrades |

Where each piece of logic lives:

- **Gemma** (`gemma_server.py`): language and social judgement. It understands the request, writes
  everything Argo says, and picks a recovery strategy from a closed list.
- **PlanSys2 / POPF**: the visit order, energy, coffee area last, permission before the director's room.
- **Mission BT** (`argo_mission_bt`, C++, BehaviorTree.CPP v3): control flow, monitoring, interrupts.
- **Executors**: one per PDDL action, refining it into Nav2 goals, speech, YOLO and driver calls.

### Target layout (new or changed items marked)

```
argo_go2_ws/src/
├── go2_interfaces/srv/
│   ├── GemmaCommand.srv            (kept: entry point /argo/command)
│   ├── FormulateGoals.srv          NEW
│   ├── AssessFailure.srv           NEW
│   ├── AppraiseFindings.srv        NEW
│   ├── Speak.srv                   NEW
│   └── Chat.srv                    NEW
├── go2_patrol_mission/
│   ├── config/ontology.yaml        NEW (replaces waypoints.yaml)
│   ├── pddl/patrol_domain_v2.pddl  NEW
│   ├── pddl/problems/p01..p10.pddl NEW (offline tests)
│   ├── scripts/gemma_server.py     NEW (Gemma part of gemma_bridge.py)
│   ├── scripts/health_monitor.py   NEW
│   ├── scripts/waypoint_markers.py (changed: reads ontology.yaml)
│   ├── src/navigate_action_node.cpp (changed: reads ontology.yaml)
│   ├── src/inspect_action_node.cpp  (changed: viewpoints + YOLO + report)
│   ├── src/announce_action_node.cpp NEW
│   ├── src/chat_action_node.cpp     NEW
│   ├── src/deliver_message_action_node.cpp NEW
│   ├── src/ask_permission_action_node.cpp  NEW
│   ├── src/recharge_action_node.cpp NEW
│   ├── src/advance_action_node.cpp  NEW (trivial)
│   ├── tests/gemma_cases.yaml       NEW (test set B)
│   ├── tools/compute_distances.py   NEW
│   └── launch/argo_mission_launch.py NEW
└── argo_mission_bt/                 NEW package
    ├── behavior_trees/mission.xml
    ├── include/argo_mission_bt/*.hpp
    ├── src/{conditions,plansys2_nodes,gemma_nodes,interaction_nodes}.cpp
    └── src/mission_node.cpp
```

Retired from the launch in Phase 6: `gemma_bridge.py`, `go2_supervisor/supervisor_node`.
Kept as they are: Nav2 (Docker, Humble), `go2_driver`, `argo_voice_node.py`, `yolo_detector.py`.

---

## Phase 1 — Design artefacts (no robot needed)

### 1.1 `config/ontology.yaml`: the single source of truth

Gemma (prompts), the BT (PlanSys2 problem), the executors (poses) and the RViz markers all read
this file. It replaces `waypoints.yaml`; the old place IDs stay the same.

```yaml
home: lab_robotics
robot: {id: go2, speed: 0.5, drain: 0.03, reserve: 15}  # m/s, % per metre (calibrate from rosbags), % never planned below
chat_time: 180                                  # s at the coffee area

hubs:                                           # corridor nodes, approximate: CHECK ON SITE
  hub_west:     {x: 6.0,  y: 2.8,  theta: 0.0}
  hub_center:   {x: 11.6, y: 2.85, theta: 0.0}  # old "corridor" waypoint
  hub_junction: {x: 19.0, y: 2.0,  theta: 1.57} # about 1 m from a wall in the first estimate
  hub_north1:   {x: 19.6, y: 8.0,  theta: 1.57}
  hub_north2:   {x: 19.9, y: 14.0, theta: 1.57}
  hub_north3:   {x: 20.6, y: 19.5, theta: 1.57}

places:
  lab_robotics:
    class: Lab
    pose: {x: 1.94, y: -1.0, theta: 0.0}
    occupant: ["Simone", "Arianna"]
    entrance: hub_west
    dock: true
    works_here: true
    viewpoints: [{x: 1.94, y: -1.0}, {x: 3.5, y: -2.0}]
    synonyms: ["laboratorio di robotica", "lab di robotica", "robotics lab", "casa", "cuccia", "Simone", "Arianna"]
  vito_room:
    class: Office
    occupant: ["Vito", "Nicola M"]
    pose: {x: 7.8, y: 0.29, theta: -1.57}       # MOVE TO THE DOORWAY (privacy)
    entrance: hub_west
    works_here: true
    synonyms: ["stanza di Vito", "ufficio di Vito", "Vito", "Nicola M"]
  openspace:
    class: CommonArea
    pose: {x: 12.5, y: -1.13, theta: -1.57}
    entrance: hub_center
    works_here: true
    viewpoints: [{x: 12.5, y: -1.1}, {x: 11.5, y: -3.0}, {x: 13.5, y: -3.0}]
    synonyms: ["open space", "l'openspace"]
  coffee_break_area:
    class: CommonArea
    social_spot: true
    pose: {x: 13.1, y: 3.5, theta: 0.0}
    entrance: hub_center
    synonyms: ["macchinetta del caffè", "zona caffè", "coffee break"]
  administrative_room:
    class: AdminOffice
    occupant: ["Paola", "Marco", "Rosalba", "Daniela"]
    pose: {x: 22.2, y: -1.97, theta: 0.0}
    entrance: hub_junction
    works_here: true
    viewpoints: [{x: 22.2, y: -1.97}]
    synonyms: ["amministrazione", "ufficio amministrativo", "Paola", "Marco", "Rosalba", "Daniela"]
  vision_lab:
    class: Lab
    pose: {x: 23.7, y: 2.75, theta: 0.0}
    entrance: hub_junction
    works_here: true
    viewpoints: [{x: 23.7, y: 2.75}, {x: 25.5, y: 3.5}]
    synonyms: ["laboratorio di visione", "lab di visione", "vision lab"]
  angelo_room:  {class: Office, occupant: ["Angelo","Lia"] , pose: {x: 16.6, y: 6.52, theta: 3.14}, entrance: hub_north1, works_here: true, synonyms: ["stanza di Angelo", "Angelo", "Lia"]}
  max_room:     {class: Office, occupant: ["Max","Teresa", "Mara"],    pose: {x: 23.4, y: 8.31, theta: 0.0},  entrance: hub_north1, works_here: true, synonyms: ["stanza di Max", "Max", "Teresa", "Mara"]}
  gianni_room:  {class: Office, occupant: ["Gianni","Nicola A"], pose: {x: 20.2, y: 16.5, theta: 0.0},  entrance: hub_north2, works_here: true, synonyms: ["stanza di Gianni", "Gianni", "Nicola A"]}
  beppe_room:   {class: Office, occupant: ["Beppe","Michele"],  pose: {x: 26.5, y: 12.8, theta: 0.0},  entrance: hub_north2, works_here: true, synonyms: ["stanza di Beppe", "Beppe", "Michele"]}
  research_director_room:
    class: DirectorOffice
    occupant: ["Tiziana", "Grazia", "Cosimo"]
    needs_permission: true
    pose: {x: 22.8, y: 22.2, theta: 0.0}        # INSIDE the room (entered after ask_permission); offices are served from the doorway
    entrance: hub_north3
    works_here: true
    synonyms: ["ufficio del direttore", "direttore", "director", "Tiziana", "Grazia", "Cosimo"]

# Tree edges (undirected). Distances in metres from tools/compute_distances.py
edges:
  - [hub_west, hub_center, 6.0]
  - [hub_center, hub_junction, 7.8]
  - [hub_junction, hub_north1, 6.3]
  - [hub_north1, hub_north2, 6.6]
  - [hub_north2, hub_north3, 5.6]
  - [lab_robotics, hub_west, 6.7]
  - [vito_room, hub_west, 3.8]
  - [openspace, hub_center, 4.7]
  - [coffee_break_area, hub_center, 1.8]
  - [administrative_room, hub_junction, 9.5]
  - [vision_lab, hub_junction, 7.8]
  - [angelo_room, hub_north1, 4.6]
  - [max_room, hub_north1, 3.7]
  - [gianni_room, hub_north2, 2.4]
  - [beppe_room, hub_north2, 9.3]
  - [research_director_room, hub_north3, 5.0]

classes:          # what each class allows: read by Gemma and by the KB loader
  Lab:            {camera_allowed: true}
  CommonArea:     {camera_allowed: true}
  AdminOffice:    {camera_allowed: true}
  Office:         {camera_allowed: false}       # privacy: never inspected, served from the doorway
  DirectorOffice: {camera_allowed: false}
```

`occupant` may be a list: Gemma maps any listed name to that room ("di' a Lia…" → angelo_room).

Check: every `entrance` and edge endpoint exists; every place has exactly one edge to a hub; the
graph is a tree (17 nodes, 16 edges).

### 1.2 `tools/compute_distances.py`

Offline script. It loads `institute_stiima_cnr_clear.pgm` and the yaml (resolution 0.05, origin
[-2.96, -11.7]), inflates obstacles by 0.25 m, snaps every place and hub to the nearest free cell,
runs Dijkstra (8-connected) from each node, and prints the path length for every edge in
`ontology.yaml`. Run it again whenever a pose changes, and paste the numbers into `edges`.
It also verifies the tree property: blocking each corridor edge must disconnect the graph.
(The numbers above already come from this procedure on the current map.)

### 1.3 `pddl/patrol_domain_v2.pddl`

POPF-compatible: typing, durative actions, numeric fluents; no negative preconditions, no
conditional effects, no PDDL3. Functions take a `robot` argument, to avoid 0-ary functions in the
PlanSys2 parser.

```pddl
(define (domain argo_office)
  (:requirements :strips :typing :durative-actions :fluents)
  (:types robot location)

  (:predicates
    (robot_at ?r - robot ?l - location)
    (connected ?a ?b - location)
    (path_free ?a ?b - location)
    (may_enter ?l - location)
    (needs_permission ?l - location)
    (camera_allowed ?l - location)
    (is_dock ?l - location)
    (social_spot ?l - location)
    (needs_call ?l - location)
    (announced ?l - location)
    (has_message ?l - location)
    (message_delivered ?l - location)
    (inspected ?l - location)
    (socialized ?l - location)
    (unlocked ?l - location)
    (done ?l - location)
    (precedes ?a ?b - location))

  (:functions
    (distance ?a ?b - location)
    (speed ?r - robot)
    (drain ?r - robot)
    (battery ?r - robot)
    (reserve ?r - robot)                ; % of battery a plan must never go below (15; 0 for the R1 trip home)
    (inspect_time ?l - location)
    (chat_time ?r - robot)
    (to_call ?r - robot))

  (:durative-action navigate
    :parameters (?r - robot ?a ?b - location)
    :duration (= ?duration (/ (distance ?a ?b) (speed ?r)))
    :condition (and (at start (robot_at ?r ?a))
                    (at start (connected ?a ?b))
                    (at start (path_free ?a ?b))
                    (at start (may_enter ?b))
                    (at start (>= (battery ?r) (+ (reserve ?r) (* (distance ?a ?b) (drain ?r))))))
    :effect (and (at start (not (robot_at ?r ?a)))
                 (at end (robot_at ?r ?b))
                 (at end (decrease (battery ?r) (* (distance ?a ?b) (drain ?r))))))

  (:durative-action announce                       ; M1
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 8)
    :condition (and (over all (robot_at ?r ?l))
                    (at start (needs_call ?l)))
    :effect (and (at start (not (needs_call ?l)))
                 (at end (announced ?l))
                 (at end (decrease (to_call ?r) 1))))

  (:durative-action chat                           ; M1, only after everyone is called
    :parameters (?r - robot ?l - location)
    :duration (= ?duration (chat_time ?r))
    :condition (and (over all (robot_at ?r ?l))
                    (at start (social_spot ?l))
                    (at start (<= (to_call ?r) 0)))
    :effect (at end (socialized ?l)))

  (:durative-action deliver_message                ; M2
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 12)
    :condition (and (over all (robot_at ?r ?l))
                    (at start (has_message ?l))
                    (at start (unlocked ?l)))
    :effect (and (at start (not (has_message ?l)))
                 (at end (message_delivered ?l))
                 (at end (done ?l))))

  (:durative-action inspect_room                   ; M3
    :parameters (?r - robot ?l - location)
    :duration (= ?duration (inspect_time ?l))
    :condition (and (over all (robot_at ?r ?l))
                    (at start (camera_allowed ?l))
                    (at start (unlocked ?l)))
    :effect (and (at end (inspected ?l))
                 (at end (done ?l))))

  (:durative-action ask_permission                 ; director's room
    :parameters (?r - robot ?l ?h - location)
    :duration (= ?duration 15)
    :condition (and (over all (robot_at ?r ?h))
                    (at start (connected ?h ?l))
                    (at start (needs_permission ?l)))
    :effect (at end (may_enter ?l)))

  (:durative-action recharge                       ; energy (fixed-duration version)
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 60)
    :condition (and (over all (robot_at ?r ?l))
                    (at start (is_dock ?l))
                    (at start (<= (battery ?r) 50)))
    :effect (at end (increase (battery ?r) 50)))

  (:durative-action advance                        ; order from language
    :parameters (?p ?n - location)
    :duration (= ?duration 0.01)
    :condition (and (at start (done ?p))
                    (at start (precedes ?p ?n)))
    :effect (at end (unlocked ?n)))
)
```

### 1.4 Problems P1–P10 and the offline check

Write one problem file per test (`pddl/problems/p01_message_vito.pddl`, …). Every mission goal ends
with `robot_at go2 lab_robotics`.

| ID | Mission | Problem (start at the lab unless stated) | Expected plan property |
|----|---------|------------------------------------------|------------------------|
| P1 | M2 | Message to Vito | lab → hub_west → vito_room, deliver, back home |
| P2 | M1 | Coffee call: vito_room, openspace, vision_lab, angelo_room | all four announced; `chat` at coffee_break_area last; then home |
| P3 | M1 | Coffee call to all 10 `works_here` rooms | coffee area still last; record planning time (scaling) |
| P4 | M2 | "Prima da Max, poi da Vito" (`precedes max_room vito_room`) | max_room delivered before vito_room although Vito is closer |
| P5 | M3 | Inspect openspace, vision_lab, administrative_room | three `inspect_room`, sensible order |
| P6 | M3 | Same as P5, battery 16 % (only 1 % above the 15 % reserve) | a `recharge` is inserted |
| P7 | M2 | Message to the director | `ask_permission` at hub_north3 before entering |
| P8 | M1 | P2 with hub_center–hub_junction blocked (`path_free` false both ways) | no plan; after dropping the unreachable rooms, vito_room and openspace only |
| P9 | M3 | Inspect vito_room | no plan: offices have no `camera_allowed` (privacy) |
| P10 | R1 | Robot at hub_north3, battery 3 %, any target | no plan within the timeout (below the 15 % reserve) |
| P10b | R1 | Same robot, goal = home only, `reserve = 0` (the R1 trip home) | plan home exists (needs ~1.1 %) |

Reference optimum computed on the topological model (exhaustive search over visit orders): P2 = 99 m,
P3 = 183 m from the lab (196 m from hub_center). Use them to judge the quality of POPF's plans.

The KB loader in Phase 5 generates the same facts, so these files double as its reference output.

```bash
# POPF is built with PlanSys2
ros2 run popf popf pddl/patrol_domain_v2.pddl pddl/problems/p02_coffee4.pddl
```

**Phase 1 check:** P1–P8 solved with the expected structure; P9 and P10 unsolvable. Record makespan,
number of actions and planning time (mean of 10 runs) on the laptop and on the Orin.

Verify here, before writing executors, three PlanSys2 Foxy details:
1. The domain expert accepts `:fluents` and functions with arguments (load the domain with `plansys2_terminal`).
2. The executor applies **numeric** effects to the KB (`to_call`, `battery`). If it does not, the
   `announce` executor decrements `to_call` itself through the ProblemExpertClient, and SyncKB writes
   the real battery before each planning call.
3. `chat`'s check `(<= (to_call go2) 0)` passes at run time after the announcements.

---

## Phase 2 — `gemma_server.py` (the head)

Take the prompts, JSON parsing, proxy handling and fallbacks out of `gemma_bridge.py`. Build every
prompt from `ontology.yaml`, so the room list cannot drift again. Same llama.cpp server
(`http://127.0.0.1:8000/v1`, Gemma 4 E2B, temperature 0.3).

### New services (`go2_interfaces/srv`)

```text
# FormulateGoals.srv   (G1 + G2)
string command
string context_json          # robot room, battery, mission running
---
bool accepted
string mission               # coffee_call | message | inspect | none
string[] targets             # place IDs only
bool ordered
string message_text          # M2 only
string clarify_question      # non-empty -> ask, then call again
string reply                 # sentence to say now
bool used_fallback
```
```text
# AssessFailure.srv   (G3)
string failed_action
string[] action_args
string detections_json
float32 battery
string[] reachable_targets
string[] unreachable_targets
int32 attempts
---
string strategy              # WAIT_RETRY | ASK_CLEAR | DROP_UNREACHABLE | RETURN_HOME | ABORT
string sentence
bool used_fallback
```
```text
# AppraiseFindings.srv   (G4)
string room
string detections_json       # {"person": 3, "laptop": 2, ...}
string time_of_day
---
string verdict               # NONE | REPORT
string sentence
```
```text
# Speak.srv   (G5)
string event_type            # announce | message | inspection | failure | battery_low | sensor_fault | permission
string facts_json
---
string sentence
bool used_fallback
```
```text
# Chat.srv   (G6)
string detections_json
string history
---
string line
```

### Rules for every service

- Validate before returning: IDs must exist in the ontology; `mission` and `strategy` must be in their
  closed sets; for `inspect`, reject targets without `camera_allowed` (privacy) and say why.
- Fallback on timeout (10 s) or invalid JSON: keyword match on IDs, synonyms and occupant names for
  G1; WAIT_RETRY then DROP_UNREACHABLE for G3; a fixed template per `event_type` for G5; a fixed
  list of lines for G6. Set `used_fallback = true`.
- Speech: Italian, first person, at most 25 words, and never change facts (numbers, names, times).
- "Tutti" in M1 means every place with `works_here: true`. "Vai da <name>" uses `occupant`.

### Test set B

`tests/gemma_cases.yaml` holds about 43 cases with the
expected JSON. `tests/run_gemma_tests.py` calls the services and prints accuracy per category, the
median and 95th-percentile latency, and the fallback rate.

Categories (count): M1 phrasings "chiama tutti per il caffè", "pausa caffè!", "coffee time" (6) ·
M1 partial "chiama solo il corridoio nord" (3) · M2 with names, incl. any listed occupant
"di' a Lia che…" (6) · M2 ordered "prima da Max e poi da Gianni" (3) · M3 "ispeziona openspace e
vision lab" (5) · ambiguous "vai nel lab", "dillo a lui" → clarify (4) · privacy "guarda la
scrivania di Vito" → reject (3) · unknown / off-topic "vai da Pippo", "che tempo fa" → reject (5) ·
G5 sentences for inspection, failure, R1, R2: Italian, ≤ 25 words, facts unchanged (8).

**Phase 2 check:** at least 90 % on M1, M2 and M3 phrasings, 100 % on privacy and unknown places,
and the fallback works with llama.cpp stopped.

---

## Phase 3 — Action executors (one per PDDL action)

All of them derive from `plansys2::ActionExecutorClient`, like the two you have. Speaking
executors publish on `/argo/say` and finish only when `/argo/voice_busy` goes back to `false`. The
robot only speaks while standing still, and the domain already guarantees it.

| Action | Executor | What it does | Fails when |
|--------|----------|--------------|------------|
| navigate | `navigate_action_node` (changed) | NavigateToPose to the pose of `?b` (place or hub) from ontology.yaml; keep the voice wait and the BalanceStand unlock | Nav2 aborts → publish `failure:navigate:…` |
| announce | `announce_action_node` | Speak(event=announce) → say "Pausa caffè! …" | Speaker down |
| chat | `chat_action_node` | Every 25 s: Chat(detections) → say, until `chat_time` | Never fails, ends on time |
| deliver_message | `deliver_message_action_node` | Read the text for `?l` from `/argo/messages` (latched JSON written by the BT); say "<name>, ho un messaggio per te: <text>" | Optional: YOLO sees nobody → "recipient absent" (goes to G3) |
| inspect_room | `inspect_action_node` (changed) | For each viewpoint: Nav2 goal, rotate 360° (Nav2 Spin or 4 yaw goals), collect `/yolo/detections`; keep max count per class; save JSON + image under `~/argo_logs/inspections/`; publish `/argo/findings`; Speak(event=inspection) → say | Camera or YOLO silent |
| ask_permission | `ask_permission_action_node` | Speak(permission) → say "Posso entrare?"; wait 30 s for an answer on `/argo/answer` (yes/no from the operator, or speech-to-text later) | No or timeout |
| recharge | `recharge_action_node` | Say that it must charge; call the `Mode` service to lie down; wait until the SOC rises (or `/argo/answer` = "charged" in tests); stand up | Timeout |
| advance | `advance_action_node` | `finish(true)` at once | — |

Test each executor alone with `plansys2_terminal`: set the initial state by hand,
`run (announce go2 vito_room)`, and check the effect in the KB.

**Phase 3 check:** each action runs on the robot once, on its own.

---

## Phase 4 — `health_monitor.py` (R2)

Publishes `/argo/health` (`std_msgs/String`, JSON, 2 Hz):
`{"status": "OK|DEGRADED|FATAL", "faults": [{"source": "camera", "level": "DEGRADED"}]}`

| Source | Signal | Fault if | Level |
|--------|--------|----------|-------|
| LiDAR | `/lidar_points` and `/scan` rate | no message for 1 s | FATAL |
| Localisation | `/amcl_pose` | covariance xx + yy > 1.0 m², or no update for 10 s while moving | FATAL |
| Nav2 | `bt_navigator` lifecycle state | not active | FATAL |
| Camera | `/camera/color/image_raw` | no message for 2 s | DEGRADED |
| YOLO | `/yolo/detections` | no message for 5 s while the camera is fine | DEGRADED |
| Gemma | GET `/v1/models` every 10 s | no answer in 3 s | DEGRADED |
| Voice | `argo_voice` alive | node missing | DEGRADED (log only, it cannot speak) |

The battery is not in here: the BT reads `lowstate` (`unitree_go/LowState`, `bms_state.soc`)
directly for R1.

**Phase 4 check:** kill each node and read `/argo/health` change within the timeout.

---

## Phase 5 — `argo_mission_bt` (the executive)

New `ament_cmake` package. Dependencies: `behaviortree_cpp_v3` (already used by Nav2 on Foxy),
`plansys2_domain_expert`, `plansys2_problem_expert`, `plansys2_planner`, `plansys2_executor`,
`go2_interfaces`, `unitree_go`, `yaml-cpp`.

### BT nodes

| Node | Type | Ports | Implementation |
|------|------|-------|----------------|
| BatteryOK / BatteryLow | Condition | `min_soc` | Cached `lowstate` subscriber |
| SensorsOK | Condition | out `fault` | Cached `/argo/health`; ignores sources already marked degraded |
| MissionActive | Condition | — | Goal non-empty in the problem expert |
| GoalSatisfied | Condition | — | `ProblemExpertClient::isGoalSatisfied` |
| WaitCommand | StatefulAction | out `command` | Server for `/argo/command` (`GemmaCommand.srv`, so existing scripts keep working) |
| GemmaFormulateGoals | StatefulAction | `command` → `mission`, `goals`, `reply` | Async FormulateGoals call; clarification loop |
| SetGoals | SyncAction | `mission`, `goals` | Resets mission facts, then writes them per mission (below) |
| SyncKB | SyncAction | — | `robot_at` = nearest node to the AMCL pose; `battery` from SOC |
| ComputePlan | SyncAction | out `plan` | `PlannerClient::getPlan(domain, problem)` |
| ExecutePlan | StatefulAction | `plan` → `failure` | `ExecutorClient::start_plan_execution`; RUNNING while `execute_and_check_plan()`; on halt `cancel_plan_execution()`; on failure, the failed action from the feedback |
| GemmaAssessFailure | StatefulAction | `failure` → `strategy` | Async AssessFailure; the reachable set is computed on the tree from `path_free` |
| GemmaAppraiseFindings | StatefulAction | — | Reads `/argo/findings`; on REPORT says it aloud and publishes `/argo/report` (never a planning goal: no action could achieve it) |
| GemmaSpeak / Say | StatefulAction | `event` / `text` | Speak srv → `/argo/say` → wait for `voice_busy` false |
| ApplyStrategy | SubTree | `strategy` | Switch over the strategies (see the notes under `mission.xml`) |

`ExecutePlan`, the Gemma nodes and Say must never block the tick: use `StatefulActionNode` (or
`CoroActionNode`, whichever your BT.CPP 3.x version provides) with asynchronous clients.

### KB loader (at start-up, from ontology.yaml)

- Instances: `go2 - robot`; every place and hub `- location`.
- Rigid facts: `connected` both ways for each edge; `distance` both ways; `camera_allowed`,
  `needs_permission`, `is_dock`, `social_spot` from classes and flags; `speed`, `drain`, `reserve`, `chat_time`;
  `inspect_time ?l` = 20 s × number of viewpoints.
- Initial flexible facts: `path_free` on every edge; `may_enter` everywhere except `needs_permission` places.

### SetGoals per mission

| Mission | Facts written | Goal |
|---------|---------------|------|
| coffee_call | `needs_call t` for each target; `to_call go2 = n` | `announced t` (all) ∧ `socialized coffee_break_area` ∧ `robot_at go2 lab_robotics` |
| message | `has_message t`; text on `/argo/messages`; `unlocked` (first target only if ordered, plus `precedes` chain) | `message_delivered t` ∧ `robot_at go2 lab_robotics` |
| inspect | `unlocked` (same rule) | `inspected t` ∧ `robot_at go2 lab_robotics` |

Before writing, SetGoals clears the previous mission's facts (`announced`, `message_delivered`,
`inspected`, `socialized`, `done`, `unlocked`, `precedes`, `needs_call`, `has_message`). It keeps
`path_free`, which a timer resets to true after 5 minutes.

`mission.xml` skeleton (BehaviorTree.CPP v3). Enable Groot monitoring (`BT::PublisherZMQ`) to show
the live tree at the exam.

```xml
<root main_tree_to_execute="Mission">
 <BehaviorTree ID="Mission">
  <ReactiveSequence>
   <ReactiveFallback name="Safe">            <!-- R1 + R2, checked every tick -->
    <ReactiveSequence>
     <BatteryOK min_soc="15"/>
     <SensorsOK fault="{fault}"/>         <!-- from health_monitor -->
    </ReactiveSequence>
    <Fallback>
     <Sequence>
      <BatteryLow min_soc="15"/>
      <SubTree ID="R1_GoHomeRecharge"/>    <!-- say why, plan home, lie down -->
     </Sequence>
     <SubTree ID="R2_Degrade" fault="{fault}"/>  <!-- say what failed, degrade -->
    </Fallback>
   </ReactiveFallback>
   <Fallback name="HasMission">
    <MissionActive/>
    <Sequence>
     <WaitCommand text="{command}"/>
     <GemmaFormulateGoals command="{command}" mission="{mission}" goals="{goals}" reply="{reply}"/>
     <Say text="{reply}"/>
     <SetGoals mission="{mission}" goals="{goals}"/>
    </Sequence>
   </Fallback>
   <Fallback name="MissionDone">
    <GoalSatisfied/>
    <RetryUntilSuccessful num_attempts="3">
     <Fallback>
      <Sequence name="PlanAndAct">
       <SyncKB/>
       <ComputePlan plan="{plan}"/>
       <ExecutePlan plan="{plan}" failure="{failure}"/>
       <GemmaAppraiseFindings/>
      </Sequence>
      <Sequence name="Recover">
       <GemmaAssessFailure failure="{failure}" strategy="{strategy}"/>
       <GemmaSpeak event="{failure}"/>
       <SubTree ID="ApplyStrategy" strategy="{strategy}"/>
       <AlwaysFailure/>  <!-- forces the retry with the updated KB -->
      </Sequence>
     </Fallback>
    </RetryUntilSuccessful>
    <SubTree ID="ReturnHome"/>
   </Fallback>
  </ReactiveSequence>
 </BehaviorTree>
</root>
```

`ApplyStrategy` is a switch on G3's answer: WAIT_RETRY waits 20 s; ASK_CLEAR says "posso passare?",
waits, sets `path_free` back to true; DROP_UNREACHABLE removes the goals behind the blocked edge
(reachability on the tree, computed from the KB); RETURN_HOME replaces the goal with
`robot_at go2 lab_robotics`. R1 (SOC < 15 %) halts ExecutePlan (its halt cancels the PlanSys2 plan), says
why, sets `reserve go2 = 0`, plans home and lies down (missions are planned with `reserve go2 = 15`). R2: LiDAR, localisation or Nav2 faults stop and lie down until the sensor is back;
a camera fault removes the `inspected` goals and marks the camera degraded; a Gemma fault switches
every Gemma node to its fallback.

**Phase 5 check:** run the BT against stub executors (each finishes after 2 s) and walk P1, P2, P8,
R1 and R2 in Groot before touching the robot.

---

## Phase 6 — Launch, pipeline, robot tests

`launch/argo_mission_launch.py` starts:
- `plansys2_bringup_launch_monolithic.py` with `model_file = pddl/patrol_domain_v2.pddl`
- the 8 executors (`action_name` parameter for each)
- `gemma_server.py`, `health_monitor.py`, `waypoint_markers.py`
- `argo_mission_bt/mission_node` (parameters: `ontology_file`, `bt_xml`, `groot: true`)

In `scripts/start_pipeline.sh`, replace the `patrol` window
(`ros2 launch go2_patrol_mission patrol_mission_launch.py`) with `argo_mission_launch.py`. Keep the
llama.cpp, bringup, Nav2 (Docker), YOLO and voice windows as they are. Do not delete
`gemma_bridge.py` or `supervisor_node`: leave them out of the launch, so v1 can still be shown.

Sending a command:
```bash
ros2 service call /argo/command go2_interfaces/srv/GemmaCommand "{command: 'Argo, chiama tutti per il caffè'}"
```

Robot scenarios (at least 5 runs each):

| ID | Scenario | Pass if |
|----|----------|---------|
| S1 | Coffee call to 3 rooms, then chat | every room hears the call; chat at the coffee area; back home |
| S2 | Message to Vito | delivered word for word at the doorway; back home |
| S3 | Inspect openspace and vision lab | detections spoken as a sentence; images and counts saved |
| S4 | People at the coffee area block the corridor | Gemma picks a valid strategy; mission completes or ends safely |
| S5 | Battery threshold raised during S3 (R1) | stops, says why, goes home, lies down |
| S6 | Camera node killed during S3 (R2) | says the camera is down; inspections dropped; goes home |
| S7 | LiDAR node killed during S2 (R2) | stops and lies down within 1 s; says why |
| S8 | Gemma server stopped, then S2 | keyword parsing and fixed sentences; message still delivered |

Record every run:
```bash
ros2 bag record /argo/say /argo/health /argo/findings /patrol/action_events \
  /executor/action_execution_info /amcl_pose /lowstate /yolo/detections
```

**Phase 6 check:** S1–S8, at least 5 runs each, with the metrics table filled in.

---

## Things not to break

- Nav2 runs in a Humble container: keep the NavigateToPose goal without a feedback callback (see the
  comment in `navigate_action_node.cpp`).
- The voice wait and the BalanceStand unlock in `navigate_action_node.cpp` stay.
- Keep the place IDs (`lab_robotics`, `vision_lab`, …): `argo_voice_node.py` and the existing
  scripts use them. The old `corridor` ID becomes `hub_center`.
- Known limitation to state at the exam: every hub is a separate Nav2 goal, so Argo pauses briefly
  at each hub. Fix later with goal tolerances for hubs, or NavigateThroughPoses (available in the
  Humble container).

## Checklist

- [ ] Phase 1: ontology.yaml, distances, domain v2, P1–P10 pass in POPF, PlanSys2 numeric checks
- [ ] Phase 2: gemma_server + 5 srv files, test set B passes
- [ ] Phase 3: 8 executors, each tested alone
- [ ] Phase 4: health_monitor, every fault visible
- [ ] Phase 5: argo_mission_bt against stubs, Groot
- [ ] Phase 6: new launch, S1–S8 recorded
