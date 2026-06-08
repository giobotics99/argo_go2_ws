# go2_supervisor — Explainable Path Planning Supervisor

ROS 2 Foxy node for the **Unitree Go2** quadruped that turns opaque Nav2
planner failures into spoken, human-understandable explanations and then
negotiates an alternative route with a nearby operator — all powered by a
locally-served **Gemma 4 E2B** (2.3 B effective params) on the onboard
NVIDIA Jetson Orin.

---

## What's Inside

```
go2_supervisor/
├── go2_supervisor/
│   ├── __init__.py              # Package marker
│   └── supervisor_node.py       # Main ROS 2 node (all logic lives here)
├── resource/
│   └── go2_supervisor           # ament index marker
├── package.xml                  # ROS 2 manifest (ament_python)
├── setup.py                     # Entry point declaration
├── setup.cfg                    # ament install paths
└── README.md                    # ← you are here
```

| File | Purpose |
|------|---------|
| `supervisor_node.py` | Subscribes to Nav2 status, YOLO detections, and human speech. On planner abort → queries Gemma 4 for an explanation (Step 1), speaks it via TTS, then parses the human's verbal redirect into a structured waypoint via native tool calling (Step 2), and instantly publishes a new `PoseStamped` goal. |
| `setup.py` | Declares the `supervisor_node` console-script entry point and Python package metadata. All ROS 2 message deps are in `package.xml`, not pip. |
| `package.xml` | Lists `exec_depend` on `rclpy`, `std_msgs`, `geometry_msgs`, `action_msgs`, `nav2_msgs`. |

---

## Topic Interface

| Topic | Type | Direction | Source |
|-------|------|-----------|--------|
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | **subscribe** | Nav2 BT navigator |
| `/yolo/detections` | `std_msgs/String` | **subscribe** | YOLO26-Nano node (see below) |
| `/human/speech_input` | `std_msgs/String` | **subscribe** | Speech-to-text pipeline |
| `/goal_pose` | `geometry_msgs/PoseStamped` | **publish** | → Nav2 planner |

---

## Where Is YOLO26-Nano?

> **YOLO26-Nano is NOT part of this package.**

The supervisor is a **consumer** of YOLO detections — it subscribes to
`/yolo/detections` and expects comma-separated label strings like:

```
"person,cleaning_cart,chair,door"
```

YOLO26-Nano itself is a **separate perception node** that runs natively on
the Jetson Orin. There are several ways to deploy it:

### Option A — Standalone `jetson-inference` container (recommended)

The NVIDIA Jetson AI Lab provides ready-made containers with TensorRT-
optimised YOLO inference. Pull and run alongside this supervisor:

```bash
# Example: a minimal YOLO publisher node (you would write or adapt one)
# that wraps the TensorRT engine and publishes to /yolo/detections.
# This is outside the scope of go2_supervisor.
```

### Option B — Create a `go2_yolo` ROS 2 package

If you want a ROS 2 node that loads a YOLO26-Nano `.engine` (TensorRT) or
`.onnx` model, runs inference on the Go2's camera frames, and publishes
`std_msgs/String` to `/yolo/detections`, create a sibling package:

```
src/
├── go2_supervisor/   ← this package (consumes detections)
└── go2_yolo/         ← new package (produces detections)
    └── yolo_node.py  ← subscribe to camera, run inference, publish labels
```

### Option C — Mock for testing (no GPU needed)

For development and testing **without** an actual YOLO model, simply
publish fake detections from the command line:

```bash
# Continuous mock at 2 Hz
ros2 topic pub /yolo/detections std_msgs/msg/String \
  "{data: 'person,cleaning_cart,chair,door,fire_extinguisher'}" \
  --rate 2

# Single-shot
ros2 topic pub --once /yolo/detections std_msgs/msg/String \
  "{data: 'box,pallet'}"
```

This is sufficient to trigger the full recovery pipeline when combined
with a mocked Nav2 abort (see **Testing** below).

---

## Prerequisites

### 1. Start Gemma 4 E2B via vLLM on Jetson Orin

```bash
sudo docker run -it --rm --pull always \
  --runtime=nvidia --network host \
  -v $HOME/.cache/huggingface:/root/.cache/huggingface \
  ghcr.io/nvidia-ai-iot/vllm:gemma4-jetson-orin \
  vllm serve google/gemma-4-E2B-it \
    --gpu-memory-utilization 0.75 \
    --enable-auto-tool-choice \
    --reasoning-parser gemma4 \
    --tool-call-parser gemma4
```

Verify the model is loaded:

```bash
curl -s http://localhost:8000/v1/models | python3 -m json.tool
```

### 2. Build the package

```bash
cd ~/argo_go2_ws
colcon build --packages-select go2_supervisor --symlink-install
source install/setup.bash
```

---

## Usage

### Launch

```bash
ros2 run go2_supervisor supervisor_node
```

With parameters:

```bash
ros2 run go2_supervisor supervisor_node \
  --ros-args \
  -p gemma_api_base:="http://localhost:8000/v1" \
  -p gemma_model_id:="google/gemma-4-E2B-it" \
  -p gemma_timeout_explain:=8.0 \
  -p gemma_timeout_dispatch:=6.0 \
  -p current_goal_name:="Meeting_Room_A"
```

### Available ROS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gemma_api_base` | `http://localhost:8000/v1` | vLLM OpenAI-compatible endpoint |
| `gemma_model_id` | `google/gemma-4-E2B-it` | Model name as registered in vLLM |
| `gemma_timeout_explain` | `8.0` | HTTP timeout (s) for explanation query |
| `gemma_timeout_dispatch` | `6.0` | HTTP timeout (s) for tool-calling query |
| `current_goal_name` | `Meeting_Room_A` | The initial navigation target name |

---

## Testing Without Hardware

You can exercise the **entire pipeline** from the command line using three
terminals (no YOLO model, no Nav2 stack, no microphone needed):

```bash
# Terminal 1 — Launch supervisor
ros2 run go2_supervisor supervisor_node

# Terminal 2 — Feed fake YOLO detections (keep running)
ros2 topic pub /yolo/detections std_msgs/msg/String \
  "{data: 'person,cleaning_cart,chair'}" --rate 2

# Terminal 3 — Trigger a Nav2 abort
ros2 topic pub --once /navigate_to_pose/_action/status \
  action_msgs/msg/GoalStatusArray \
  "{status_list: [{status: 6, goal_info: {goal_id: {uuid: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]}}}]}"

# Wait for the supervisor to log the explanation, then:
ros2 topic pub --once /human/speech_input std_msgs/msg/String \
  "{data: 'Take the lab shortcut instead'}"

# Terminal 4 — Verify the new goal
ros2 topic echo /goal_pose --once
```

Expected final goal: **Lab_Shortcut** → `(x=3.20, y=1.50, θ=0.00)`.

---

## Hardcoded Waypoints

| Name | x | y | θ (rad) |
|------|---|---|---------|
| Lab_Shortcut | 3.20 | 1.50 | 0.00 |
| Main_Corridor | 0.00 | 5.00 | 1.57 |
| Kitchen | -2.10 | 3.80 | 3.14 |
| Meeting_Room_A | 5.60 | -1.20 | -1.57 |
| Meeting_Room_B | 7.00 | 2.00 | 0.78 |
| Reception | -4.00 | 0.50 | 3.14 |
| Emergency_Exit | 1.00 | -6.00 | -1.57 |
| Server_Room | 4.50 | 4.50 | 0.00 |
| Elevator | -1.50 | -3.00 | 1.57 |
| Staircase | 2.00 | -4.50 | -0.78 |

Edit these in `supervisor_node.py` → `OFFICE_WAYPOINTS` to match your
actual map coordinates.

---

## License

Apache-2.0
