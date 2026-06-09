# go2_supervisor — Explainable Path Planning Supervisor

ROS 2 Foxy package for the **Unitree Go2** quadruped that turns opaque Nav2 planner failures into spoken/readable explanations, and resolves recovery trajectories using a locally served **Gemma 4 E2B** model (2.3B parameters) on the onboard NVIDIA Jetson Orin.

---

## What's Inside

```
go2_supervisor/
├── go2_supervisor/
│   ├── __init__.py              # Package marker
│   └── supervisor_node.py       # Main supervisor logic
├── resource/
│   └── go2_supervisor           # ament index marker
├── package.xml                  # ROS 2 manifest (ament_python)
├── setup.py                     # Entry point declaration
├── setup.cfg                    # ament install paths
└── README.md                    # ← you are here
```

Additionally, this repository contains a GPU-accelerated YOLO detector node located in the companion package `go2_nav`:
* **`go2_nav/scripts/yolo_detector.py`**: Runs PyTorch/CUDA-accelerated YOLO inference on the Orin GPU to scan camera frames from the robot and publish objects to `/yolo/detections`.

---

## Topic Interface

| Topic | Type | Direction | Source / Destination |
|-------|------|-----------|----------------------|
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | **subscribe** | Nav2 BT navigator |
| `/yolo/detections` | `std_msgs/String` | **subscribe** | GPU YOLO node (`yolo_detector.py`) |
| `/human/speech_input` | `std_msgs/String` | **subscribe** | Speech-to-text pipeline (in Manual mode) |
| `/supervisor/explanation` | `std_msgs/String` | **publish** | Published natural-language explanation |
| `/goal_pose` | `geometry_msgs/PoseStamped` | **publish** | Re-dispatched recovery goal to Nav2 |

---

## Core Features & Modes

### 1. Autonomous Recovery Mode (Default)
* **Parameter**: `auto_recovery_mode := true`
* **How it works**: When Nav2 fails, the supervisor compiles the current blocked environment context and sends a single-turn request to Gemma. Gemma returns a unified JSON containing both the failure explanation and the best waypoint choice. The supervisor immediately publishes the text explanation to `/supervisor/explanation` and dispatches the goal to `/goal_pose`. No human verbal input or typing is required.

### 2. Manual/Interactive Voice Mode
* **Parameter**: `auto_recovery_mode := false`
* **How it works**: When Nav2 fails, the supervisor queries Gemma to describe the obstruction and publishes it. It then stops and waits for a text instruction on `/human/speech_input`. Once the user dictates a redirection (e.g., *"Take the lab shortcut"*), Gemma parses this intent using native tool-calling (`navigate_to_waypoint`) and commands the robot to go there.

---

## Pre-requisites & Installation

### 1. Install YOLO GPU Dependencies (on Jetson Orin)
Ensure the PyTorch CUDA build is installed. Then install `ultralytics` for GPU-accelerated object detection:
```bash
pip install ultralytics
```

### 2. Sourcing & Building the Workspace
Build both the navigation stack and the supervisor packages:
```bash
cd ~/argo_go2_ws
colcon build --packages-select go2_nav go2_supervisor --symlink-install
source install/setup.bash
```

---

## Step-by-Step Execution Guide

### Step 1: Start Gemma 4 via llama.cpp Server 
Ensure your Jetson Orin is connected to the internet for the first automated model download from Hugging Face. We use the optimized GGUF weights to run efficiently within the remaining 10Gi hardware memory budget of the Jetson Orin NX.

Run the compiled high-performance local server using all available hardware resources:
```bash
~/llama.cpp/build/bin/llama-server \
  -hf unsloth/gemma-4-E2B-it-GGUF \
  --model "*Q8_0.gguf" \
  --port 8000 \
  --ctx-size 4096 \
  --n-gpu-layers 99 \
  --reasoning off
```

```bash
~/llama.cpp/build/bin/llama-server   -m ~/models/gemma-4-E2B-it-Q8_0.gguf   --port 8000   --ctx-size 4096   --n-gpu-layers 99   --reasoning off

```



Real-Time Latency Note: The --chat-template-kwargs '{"enable_thinking":false}' flag is mandatory. It forces Gemma 4 to bypass its internal reasoning tokens and generate the text/JSON layout directly, dropping inference latency to real-time constraints required by the quadruped path-planner.

Verify that the local OpenAI-compatible API is online and responding:
```bash
curl http://localhost:8000/v1/models
```

### Step 2: Start Nav2 with a Map
Start your standard robot navigation stack (which publishes the `/camera/color/image_raw` color frames):
```bash
source ~/argo_go2_ws/install/setup.bash
ros2 launch go2_bringup bringup_with_map.launch.py
```

### Step 3: Launch YOLO GPU Detections
Start the GPU-accelerated object detector:
```bash
source ~/argo_go2_ws/install/setup.bash
ros2 run go2_nav yolo_detector.py
```

### Step 4: Start the Supervisor Node
```bash
source ~/argo_go2_ws/install/setup.bash
ros2 run go2_supervisor supervisor_node --ros-args -p auto_recovery_mode:=true
```

### Step 5: Read Explanations
In a separate monitoring terminal, echo the topic to read what Gemma is saying:
```bash
ros2 topic echo /supervisor/explanation
```

---

## Testing Without Hardware (CLI Mocking)

If you need to test the pipeline manually without running the camera or Nav2:

1. **Launch the supervisor in Autonomous Mode**:
   ```bash
   ros2 run go2_supervisor supervisor_node --ros-args -p auto_recovery_mode:=true
   ```
2. **Publish mock detections**:
   ```bash
   ros2 topic pub /yolo/detections std_msgs/msg/String "{data: 'person,cleaning_cart,chair'}" --rate 2
   ```
3. **Trigger a mock Nav2 abort**:
   ```bash
   ros2 topic pub --once /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray "{status_list: [{status: 6, goal_info: {goal_id: {uuid: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]}}}]}"
   ```
4. **Observe the result**:
   Check the `/supervisor/explanation` topic and `/goal_pose` goal redirection.

---

## ROS Parameters (supervisor_node)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `auto_recovery_mode` | `true` | When true, skips human voice input and dispatches goals automatically. |
| `gemma_api_base` | `http://localhost:8000/v1` | Local vLLM endpoint. |
| `gemma_model_id` | `google/gemma-4-E2B-it` | Model identifier string. |
| `gemma_timeout_explain` | `8.0` | Timeout in seconds for explanation / auto queries. |
| `gemma_timeout_dispatch` | `6.0` | Timeout in seconds for manual waypoint parses. |
| `current_goal_name` | `Meeting_Room_A` | Target navigation goal name. |

---

## Hardcoded Waypoints

Edit the coordinates inside `supervisor_node.py` under the `OFFICE_WAYPOINTS` dictionary to match your SLAM maps:

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

---

## License

Apache-2.0
