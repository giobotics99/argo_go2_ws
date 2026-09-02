# Argo GO2 Workspace — Unitree Go2 Autonomous Navigation with Onboard AI

This is a ROS 2 workspace (developed for **ROS 2 Foxy** on Ubuntu 20.04, running on an
**NVIDIA Jetson Orin**) that turns a **Unitree Go2** quadruped robot into a self-driving,
task-planning robot that can patrol rooms, follow people, and explain its own mistakes
out loud using an onboard large-language model.

It combines three pillars of modern robotics software:

| Technology | Role in this project |
|------------|----------------------|
| **Gemma 4 E2B** (2.3B) | Small, locally-served LLM that understands natural-language commands and explains failures in human terms. Run on the robot itself via `llama.cpp`. |
| **PlanSys2** | ROS 2 task planner (PDDL-based) that turns a high-level goal ("inspect the lab, then the corridor") into an ordered sequence of robot actions. |
| **Nav2** | Navigation stack that actually moves the robot — localization, global/local planners, controllers and behavior-tree driven goal execution. |

---

## What does the project do?

The robot can be commanded in **plain natural language** and then carry out the task
autonomously, while **talking back** to explain what it is doing and why. The main
capabilities are:

1. **ML / LLM "brain"** — a Gemma 4 E2B model served locally on the Jetson converts
   spoken/typed commands into structured task descriptions.
2. **Task planning** — PlanSys2 (with a PDDL domain) turns those parsed commands into a
   multi-step plan (e.g. *navigate to lab → inspect → navigate to corridor*).
3. **Motion execution** — Nav2 executes the plan: localize on the map, plan a path,
   avoid obstacles (using a Hesai 3D LiDAR + RealSense camera), and drive to each waypoint.
4. **Explainability & recovery** — when the path planner gets stuck or fails, Gemma
   generates a plain-English, first-person explanation of *why*, and (in autonomous mode)
   automatically picks an alternative waypoint to recover.
5. **Follow-me mode** — an alternate Nav2 behavior tree lets the robot continuously track
   and follow a moving target while keeping a safe distance.

---

## Workspace layout

```
argo_go2_ws/
├── README.md                 # ← this file
├── README_FOLLOW_ME.md       # Guide for the follow-me Navigation strategy
├── go2_nav2.rviz             # RViz configuration
├── map_*.pgm / map_*.yaml    # SLAM maps of the environment
└── src/
    ├── go2_robot/            # Upstream Unitree Go2 ROS 2 integration (base)
    ├── go2_bringup/          # Launch file to start the robot hardware/nav
    ├── go2_description/      # URDF model of the Go2
    ├── go2_driver/           # Low-level comms with the Go2 (odom, joints, modes)
    ├── go2_hardware/         # Hardware interface layer
    ├── go2_simulation/       # Gazebo simulation (optional)
    ├── go2_interfaces/       # Custom ROS 2 service definitions
    ├── go2_nav/              # Nav2 stack + perception scripts (YOLO, follow-me)
    ├── go2_rviz/             # RViz helpers
    ├── go2_supervisor/       # ★ Explainable path-planning supervisor (Gemma)
    ├── go2_patrol_mission/   # ★ PlanSys2 multi-room patrol + Gemma bridge
    ├── cascade_lifecycle/    # Lifecycle orchestration helper
    ├── nav2_bringup/         # Upstream Nav2 launch templates
    ├── unitree_api/          # Unitree communication messages
    ├── unitree_go/           # Unitree Go SDK messages
    └── hesai_ros_driver/     # Hesai 3D LiDAR ROS 2 driver
```

The two packages at the heart of the AI work are:

### `go2_supervisor` — the "explainable" planner
A Python ROS 2 node that sits on top of Nav2 and watches the
`/navigate_to_pose/_action/status` topic:

- When Nav2 **aborts** a goal (planner failure), the supervisor gathers context — the
  failed destination plus the latest **YOLO object detections** (e.g. `person, chair,
  cleaning_cart`) — and sends it to **Gemma 4 E2B**.
- **Autonomous mode** (default): Gemma returns a JSON `{explanation, new_target}`. The
  supervisor publishes the explanation to `/supervisor/explanation` (and to a TTS
  pipeline to be spoken) and immediately dispatches a new Nav2 goal to the chosen waypoint.
- **Manual/voice mode**: Gemma explains the blockage and the supervisor waits for a human
  voice command on `/human/speech_input`. The command is parsed by Gemma using **native
  tool calling** (`navigate_to_waypoint`) to pick the new target.
- Uses a `MultiThreadedExecutor` + thread pool so the blocking HTTP calls to Gemma never
  stall the robot's control loop.
- A companion node (`go2_nav/scripts/yolo_detector.py`) runs GPU-accelerated YOLO on the
  Orin to detect and report nearby objects.

### `go2_patrol_mission` — natural-language multi-room patrol
A C++/Python package that adds **PlanSys2** task planning:

- Defines a PDDL **domain** (`patrol_domain.pddl`) with durative actions `navigate` and
  `inspect` and rooms `lab_robotics`, `corridor`, `openspace` (waypoints in
  `config/waypoints.yaml`).
- Provides PlanSys2 action **executors**:
  - `navigate_action_node.cpp` sends the actual `NavigateToPose` goals to Nav2 for a room.
  - `inspect_action_node.cpp` tilts the Go2 forward (`/euler` pitch), waits 3 s, restores pose.
- `gemma_bridge.py` exposes a `/gemma_interface/send_command` service. You send it a
  sentence like *"Vai a controllare lab_robotics e poi passa da openspace"*, and Gemma
  parses the target room IDs, configures the PlanSys2 problem, generates a plan, and
  executes it automatically. It also listens for navigation failures and uses Gemma to
  explain them and re-plan around blocked edges.

---

## How the pieces connect

```
        Human natural-language command
                    │   (service /gemma_interface/send_command)
                    ▼
        ┌───────────────────────┐        ┌──────────────────────────┐
        │  gemma_bridge (Python)│ ─────► │  PlanSys2 (PDDL planner)  │
        └───────────────────────┘        │  navigate / inspect       │
                    │                     └────────────┬─────────────┘
                    │ task plan                        │ execute_plan
                    ▼                                  ▼
        ┌───────────────────────┐        ┌──────────────────────────┐
        │  supervisor_node      │  ◄──── │  Action executors (C++)  │
        │  (explain + recover)  │        └────────────┬─────────────┘
        └───────────┬───────────┘                     │ NavigateToPose
                    │  /goal_update, /supervisor/     ▼
                    │  explanation, /goal_pose    ┌──────────────────────────┐
                    ▼                             │  Nav2 (AMCL, planners,  │
        ┌───────────────────────┐                 │  DWB, BT navigator)     │
        │  Gemma 4 E2B (LLM)    │ ◄────────────── │        +                │
        │  llama.cpp local      │                 │  YOLO detections        │
        └───────────────────────┘                 └──────────────────────────┘
                                                            │
                                                            ▼
                                               Go2 driver → /cmd_vel motion
```

1. You speak/type a command → `gemma_bridge` asks **Gemma** to turn it into room targets.
2. **PlanSys2** generates a plan (`navigate` / `inspect` actions).
3. The **action executors** drive **Nav2** to each waypoint.
4. Perception (**YOLO** + LiDAR + camera) feeds the state back to the supervisor.
5. If **Nav2 fails**, the **supervisor** asks **Gemma** why, publishes/talks the answer,
   and either auto-picks a new waypoint or listens for your next verbal instruction.

---

## Quick start (summary)

Full detail is in the per-package READMEs. High level:

```bash
# 1. Start the local Gemma 4 E2B server (llama.cpp, OpenAI-compatible API on :8000)
~/llama.cpp/build/bin/llama-server -hf unsloth/gemma-4-E2B-it-GGUF \
  --model "*Q8_0.gguf" --port 8000 --ctx-size 4096 --n-gpu-layers 99 --reasoning off

# 2. Build the workspace
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Bring up the robot + Nav2 with a map
ros2 launch go2_bringup bringup_with_map.launch.py

# 4. Start perception (GPU YOLO)
ros2 run go2_nav yolo_detector.py

# 5. Start the explainable supervisor
ros2 run go2_supervisor supervisor_node --ros-args -p auto_recovery_mode:=true

# 6. Send a natural-language patrol command (PlanSys2 + Gemma bridge)
ros2 service call /gemma_interface/send_command go2_interfaces/srv/GemmaCommand \
  "{command: 'Vai a controllare lab_robotics e poi passa da openspace'}"

# 7. Hear/read what Gemma says
ros2 topic echo /supervisor/explanation
```

Optional **follow-me** mode instead of standard navigation:

```bash
ros2 launch go2_nav navigation.launch.py follow_me:=true
```

---

## Main reference docs

- `src/go2_supervisor/README.md` — full explainable-supervisor guide (modes, topics,
  parameters, waypoint table, Gemma setup).
- `src/go2_patrol_mission/README.md` — PlanSys2 patrol setup and natural-language usage.
- `README_FOLLOW_ME.md` — how the Nav2 follow-me behavior tree works.
- `src/README.md` — the base Unitree Go2 ROS 2 integration (robot bringup, sensors, Nav2).

---

## Notes & environment

- Target hardware: **Unitree Go2** quadruped, **NVIDIA Jetson Orin NX** onboard computer.
- Sensors: **Hesai 3D LiDAR** (`/lidar_points`) + **RealSense D435i** camera
  (`/camera/color/image_raw`).
- ROS 2 distro: **Foxy** (Ubuntu 20.04), per this workspace's configuration.
- Waypoint and room coordinates must be tuned to your SLAM map (see
  `go2_patrol_mission/config/waypoints.yaml` and `OFFICE_WAYPOINTS` in
  `go2_supervisor/go2_supervisor/supervisor_node.py`).


