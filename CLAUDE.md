# CLAUDE.md — Argo workspace (Unitree Go2, ROS 2 Foxy)

Context for Claude Code working inside `~/argo_go2_ws` on the robot. Read this first.

## The project in one paragraph

Argo is a Unitree Go2 that lives in the robotics lab of the STIIMA-CNR floor. It takes requests
in Italian (voice or text), and carries out three missions: **M1 coffee call** (announce the
coffee break in every room, chat at the coffee area, go home), **M2 messenger** (deliver a message
at a person's door), **M3 inspection** (scan a room with YOLO and say what it sees). Two background
behaviours interrupt any mission: **R1 low battery** and **R2 sensor fault**. This is an
Automated Planning exam project: the planning model must stay clean and explainable.

## Roles (do not mix them)

- **Gemma** (`gemma_server.py`, llama.cpp on port 8000): language and judgement only. Every answer
  is a choice from a closed set, validated, with a fallback. Gemma never produces actions or plans.
- **PlanSys2 + POPF**: the plan (room order, energy, coffee area last, permission).
- **argo_mission_bt** (C++, BehaviorTree.CPP v3): all control flow, monitoring, interrupts,
  recovery. It is the only component allowed to start a recovery.
- **Executors**: one PlanSys2 action executor per PDDL action.

## Source of truth

- `README_instruction.md` (workspace root): the full spec — ontology, PDDL domain, services,
  executors, BT nodes, launch. Follow it. If something in it looks wrong, stop and ask.
- `src/go2_patrol_mission/config/ontology.yaml`: places, poses, classes, occupants, edges. Every
  component reads this file. Never hard-code room lists anywhere else.

## Environment facts

- Ubuntu 20.04, ROS 2 **Foxy**, Jetson Orin NX. Source `/opt/ros/foxy/setup.bash` then
  `install/setup.bash`.
- **Nav2 runs in a Humble Docker container.** Do not touch it. The Foxy side only sends
  `NavigateToPose` goals and reads results (no feedback callback: the message differs).
- Map: `src/go2_nav/maps/institute_stiima_cnr.yaml` (frame `map`).
- Topics in use: `/amcl_pose`, `lowstate` (`unitree_go/LowState`, battery = `bms_state.soc`),
  `/lidar_points`, `/scan`, `/camera/color/image_raw`, `/yolo/detections`, `/argo/say`,
  `/argo/voice_busy`, `/patrol/action_events`.
- The robot must not speak while walking (CPU). Speaking executors wait for `/argo/voice_busy`
  to go false; the PDDL domain enforces it with `over all (robot_at …)`.
- Gemma: `http://127.0.0.1:8000/v1/chat/completions`, model `google/gemma-4-E2B-it`, bypass the
  proxy for localhost (see the existing `gemma_bridge.py`).

## Do not modify

- `go2_driver`, `go2_hardware` (except adding scripts), `hesai_ros_driver`, `unitree_*`,
  `nav2_bringup`, the Nav2 Docker setup, `cascade_lifecycle`.
- Keep `gemma_bridge.py`, `supervisor_node.py` and `patrol_mission_launch.py` working: v1 must
  still run for comparison. New things go in new files and a new launch file.
- Keep place IDs (`lab_robotics`, `vision_lab`, `vito_room`, …); the old `corridor` is `hub_center`.

## How to work

- One tutorial step at a time. Propose a short plan, wait for OK, then write code.
- After code: build only what changed, e.g.
  `colcon build --symlink-install --packages-select go2_patrol_mission && source install/setup.bash`
- Then run the step's check and show the output. Do not claim a step works without running it.
- Small, readable code with comments in English; logs and spoken sentences in Italian.
- Never run `rm -rf` on the workspace, never `git push`, never install system packages without asking.
- Suggest a git commit message at the end of each step; commit only when asked.
