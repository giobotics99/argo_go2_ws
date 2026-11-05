#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Explainable Path Planning Supervisor Node for Unitree Go2.

Monitors Nav2 action status for planner failures, queries a local
Gemma 4 E2B model (served via vLLM on the Jetson Orin) for a
human-readable explanation and an alternative waypoint, then
re-dispatches a new navigation goal.

Architecture:
    ┌──────────────────────────────────────────────────────┐
    │  Jetson Orin (Ubuntu 20.04 / ROS 2 Foxy)            │
    │                                                      │
    │  ┌──────────────┐   HTTP POST    ┌───────────────┐  │
    │  │ supervisor   │ ──────────────►│  vLLM Docker  │  │
    │  │ _node.py     │◄────────────── │  Gemma 4 E2B  │  │
    │  │ (ROS2)       │   JSON resp    │  :8000/v1     │  │
    │  └──┬───┬───┬───┘               └───────────────┘  │
    │     │   │   │                                        │
    │  /navigate  /yolo   /human                           │
    │  _to_pose   /det.   /speech_input                    │
    └──────────────────────────────────────────────────────┘

Threading Model:
    Uses rclpy.executors.MultiThreadedExecutor so that
    asynchronous HTTP requests to the Gemma 4 API never
    block the primary ROS2 spin thread. All LLM calls are
    dispatched to a dedicated ThreadPoolExecutor to protect
    /tf processing and state estimation.

Author : Giobotics
License: Apache-2.0
"""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# HTTP / JSON – lightweight, no openai SDK dependency on the Jetson
# ---------------------------------------------------------------------------
import urllib.request
import urllib.error

# ===========================================================================
# Constants
# ===========================================================================

# Nav2 GoalStatus codes (action_msgs/msg/GoalStatus)
STATUS_UNKNOWN   = 0
STATUS_ACCEPTED  = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4  # note: value 4 in the enum but naming differs
STATUS_CANCELED  = 5
STATUS_ABORTED   = 6

# The spec says STATUS_ABORTED = 4, but in the *actual* action_msgs
# definition (Foxy / Humble), STATUS_ABORTED = 6.
# The user specification says "value 4" – we honour *both* to be safe.
USER_SPECIFIED_ABORT_VALUE = 4

# vLLM / Gemma 4 E2B endpoint
GEMMA_API_BASE = "http://localhost:8000/v1"
GEMMA_MODEL_ID = "google/gemma-4-E2B-it"

# HTTP timeout for Gemma requests (seconds)
GEMMA_TIMEOUT_EXPLAIN  = 8.0   # Step 1 – explanation
GEMMA_TIMEOUT_DISPATCH = 6.0   # Step 2 – structured waypoint JSON

# Max concurrent LLM requests
LLM_THREAD_POOL_SIZE = 2

# ===========================================================================
# Semantic Waypoint Map – Office Environment
# ===========================================================================
# Dict[str, Tuple[x, y, theta]]  (map frame coordinates)
OFFICE_WAYPOINTS: Dict[str, Tuple[float, float, float]] = {
    "Lab_Shortcut":     ( 3.20,  1.50,  0.00),
    "Main_Corridor":    ( 0.00,  5.00,  1.57),
    "Kitchen":          (-2.10,  3.80,  3.14),
    "Meeting_Room_A":   ( 5.60, -1.20, -1.57),
    "Meeting_Room_B":   ( 7.00,  2.00,  0.78),
    "Reception":        (-4.00,  0.50,  3.14),
    "Emergency_Exit":   ( 1.00, -6.00, -1.57),
    "Server_Room":      ( 4.50,  4.50,  0.00),
    "Elevator":         (-1.50, -3.00,  1.57),
    "Staircase":        ( 2.00, -4.50, -0.78),
}

WAYPOINT_NAMES = list(OFFICE_WAYPOINTS.keys())


# ===========================================================================
# Placeholder: text-to-speech
# ===========================================================================
def trigger_text_to_speech(text: str) -> None:
    """
    Placeholder for the TTS pipeline.

    On production hardware this would feed into a piper / espeak-ng process
    or publish to a /tts/request topic. For now we log the output.
    """
    # In deployment, replace with actual TTS call.
    pass  # The supervisor node logs the text via ROS logger instead.


# ===========================================================================
# Low-level HTTP helper (no SDK dependency)
# ===========================================================================
def _post_json(url: str, payload: dict, timeout: float) -> dict:
    """
    Perform a blocking HTTP POST with JSON body.
    Returns the parsed JSON response dict.
    Raises on network / timeout / JSON errors.
    """
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = resp.read().decode("utf-8")
    return json.loads(body)


# ===========================================================================
# SupervisorNode
# ===========================================================================
class SupervisorNode(Node):
    """
    Explainable Path Planning Supervisor for Unitree Go2.

    Lifecycle:
        1. Subscribes to Nav2 action status, YOLO detections, human speech.
        2. On STATUS_ABORTED → compiles state → queries Gemma 4 E2B for
           explanation + alternative waypoint.
        3. Publishes a new PoseStamped goal to Nav2.
    """

    def __init__(self) -> None:
        super().__init__("explainable_path_supervisor")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("gemma_api_base", GEMMA_API_BASE)
        self.declare_parameter("gemma_model_id", GEMMA_MODEL_ID)
        self.declare_parameter("gemma_timeout_explain", GEMMA_TIMEOUT_EXPLAIN)
        self.declare_parameter("gemma_timeout_dispatch", GEMMA_TIMEOUT_DISPATCH)
        self.declare_parameter("current_goal_name", "Meeting_Room_A")
        self.declare_parameter("auto_recovery_mode", True)

        self._api_base: str = self.get_parameter("gemma_api_base").value
        self._model_id: str = self.get_parameter("gemma_model_id").value
        self._timeout_explain: float = self.get_parameter("gemma_timeout_explain").value
        self._timeout_dispatch: float = self.get_parameter("gemma_timeout_dispatch").value
        self._current_goal_name: str = self.get_parameter("current_goal_name").value
        self._auto_recovery_mode: bool = self.get_parameter("auto_recovery_mode").value

        # ── Concurrency guards ────────────────────────────────────────
        self._llm_pool = ThreadPoolExecutor(
            max_workers=LLM_THREAD_POOL_SIZE,
            thread_name_prefix="gemma_llm",
        )
        self._recovery_in_progress = threading.Event()
        self._lock = threading.Lock()

        # ── Cached state ──────────────────────────────────────────────
        self._last_yolo_labels: List[str] = []
        self._last_nav_status: int = STATUS_UNKNOWN
        self._awaiting_human_input = False
        self._pending_explanation: Optional[str] = None

        # ── Callback groups (prevent executor starvation) ─────────────
        self._nav_cbg   = MutuallyExclusiveCallbackGroup()
        self._yolo_cbg  = ReentrantCallbackGroup()
        self._speech_cbg = MutuallyExclusiveCallbackGroup()

        # ── QoS profiles ──────────────────────────────────────────────
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscribers ──────────────────────────────────────────────
        self._nav_status_sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_nav_status,
            status_qos,
            callback_group=self._nav_cbg,
        )

        self._yolo_sub = self.create_subscription(
            String,
            "/yolo/detections",
            self._on_yolo_detections,
            sensor_qos,
            callback_group=self._yolo_cbg,
        )

        self._speech_sub = self.create_subscription(
            String,
            "/human/speech_input",
            self._on_human_speech,
            status_qos,
            callback_group=self._speech_cbg,
        )

        # ── Publisher ─────────────────────────────────────────────────
        self._goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10,
        )
        self._explain_pub = self.create_publisher(
            String,
            "/supervisor/explanation",
            10,
        )

        # ── Startup telemetry ─────────────────────────────────────────
        self.get_logger().info("=" * 60)
        self.get_logger().info("  Explainable Path Planning Supervisor – ONLINE")
        self.get_logger().info(f"  Gemma API : {self._api_base}")
        self.get_logger().info(f"  Model     : {self._model_id}")
        self.get_logger().info(f"  Waypoints : {len(OFFICE_WAYPOINTS)}")
        self.get_logger().info(f"  Goal      : {self._current_goal_name}")
        self.get_logger().info(f"  Auto Mode : {self._auto_recovery_mode}")
        self.get_logger().info("=" * 60)

    # =====================================================================
    # Subscriber callbacks
    # =====================================================================

    def _on_yolo_detections(self, msg: String) -> None:
        """
        Cache the latest YOLO detection labels.

        Expected format on /yolo/detections:
            "person,chair,cleaning_cart,door"
        (comma-separated label strings from YOLO26-Nano)
        """
        raw = msg.data.strip()
        if not raw:
            return

        labels = [lbl.strip() for lbl in raw.split(",") if lbl.strip()]
        with self._lock:
            self._last_yolo_labels = labels

        self.get_logger().debug(
            f"[YOLO] Cached {len(labels)} labels: {labels}",
            throttle_duration_sec=2.0,
        )

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        """
        Monitor Nav2 action status for planner failures.

        GoalStatusArray.status_list contains one GoalStatus per tracked
        goal.  We inspect the *latest* entry.
        """
        if not msg.status_list:
            return

        latest = msg.status_list[-1]
        status = latest.status

        # Avoid redundant processing
        if status == self._last_nav_status:
            return
        self._last_nav_status = status

        # Convert UUID to hex string safely, handling list/array/bytes/ndarray formats across ROS 2 versions.
        goal_id_hex = bytes(latest.goal_info.goal_id.uuid[:4]).hex()
        self.get_logger().info(
            f"[NAV2] Status changed → {status} "
            f"(goal_id: {goal_id_hex}…)"
        )

        # Check for abort (honour both spec-literal 4 AND actual enum 6)
        # NOTE: only 6 is ABORTED. 4 is SUCCEEDED in action_msgs/GoalStatus,
        # so treating 4 as a failure fired "recovery" after every success.
        if status == STATUS_ABORTED:
            self.get_logger().warn(
                "[NAV2] *** PLANNER FAILURE DETECTED *** – "
                "initiating Gemma 4 recovery pipeline"
            )
            self._initiate_recovery()

    def _on_human_speech(self, msg: String) -> None:
        """
        Handle verbal user input routed through /human/speech_input.

        Only processed when the supervisor is awaiting a human decision
        after explaining the failure.
        """
        if not self._awaiting_human_input:
            self.get_logger().warn(
                "[SPEECH] Received speech input but no recovery is pending – ignoring."
            )
            return

        transcript = msg.data.strip()
        if not transcript:
            return

        self.get_logger().info(f"[SPEECH] Human input received: '{transcript}'")
        self._awaiting_human_input = False

        # Dispatch Step 2 to the LLM thread pool (non-blocking)
        self._llm_pool.submit(self._step2_parse_intent, transcript)

    # =====================================================================
    # Recovery pipeline
    # =====================================================================

    def _initiate_recovery(self) -> None:
        """
        Entry point for the two-step Gemma 4 recovery pipeline.
        Runs Step 1 on the LLM thread pool to avoid blocking the executor.
        """
        if self._recovery_in_progress.is_set():
            self.get_logger().warn(
                "[RECOVERY] Pipeline already in progress – skipping duplicate."
            )
            return

        self._recovery_in_progress.set()
        self._llm_pool.submit(self._step1_explain_failure)

    # ------------------------------------------------------------------
    # Step 1 – Explain the failure
    # ------------------------------------------------------------------

    def _step1_explain_failure(self) -> None:
        """
        Query Gemma 4 E2B to produce an explanation of why the robot is stuck.
        In auto_recovery_mode, it also automatically selects the next waypoint.

        Runs on a background thread – NEVER on the executor.
        """
        t0 = time.monotonic()

        # ── Compile state payload ─────────────────────────────────────
        with self._lock:
            yolo_labels = list(self._last_yolo_labels)  # snapshot

        # Contextual filtering: keep only labels relevant to navigation
        # obstacles (drop background classes)
        irrelevant = {
            "wall", "floor", "ceiling", "sky", "background",
            "light", "window",
        }
        filtered = [l for l in yolo_labels if l.lower() not in irrelevant]
        if not filtered:
            filtered = yolo_labels  # fall back to unfiltered if empty

        state_text = (
            f"Failed target goal: {self._current_goal_name}\n"
            f"Detected objects near the robot (YOLO): {', '.join(filtered) if filtered else 'none'}\n"
            f"Available alternative waypoints: {', '.join(WAYPOINT_NAMES)}"
        )

        if self._auto_recovery_mode:
            self.get_logger().info(f"[AUTO RECOVERY] Compiling autonomous payload:\n{state_text}")

            system_prompt = (
                "You are an onboard AI assistant on a Unitree Go2 quadruped robot. "
                "The robot's path planner has FAILED. "
                "Based on the failed target and the detected obstacles, you must: "
                "1. Explain the failure in first person (e.g., 'I cannot pass because...'). "
                "2. Select the best alternative waypoint from the list of available waypoints "
                "that avoids the obstacles. "
                "You MUST return your answer in raw JSON format matching this exact schema:\n"
                "{\n"
                "  \"explanation\": \"<1-2 sentences explanation>\",\n"
                "  \"new_target\": \"<one of the available waypoints>\"\n"
                "}\n"
                "Do not include any markdown formatting, code block backticks (like ```json), or other text. Return ONLY the raw JSON."
            )

            payload = {
                "model": self._model_id,
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": state_text},
                ],
                "max_tokens": 160,
                "temperature": 0.2,
                "stream": False,
            }

            try:
                url = f"{self._api_base}/chat/completions"
                self.get_logger().info(f"[AUTO RECOVERY] POST → {url}")
                response = _post_json(url, payload, self._timeout_explain)

                content = (
                    response.get("choices", [{}])[0]
                    .get("message", {})
                    .get("content", "")
                    .strip()
                )

                elapsed = time.monotonic() - t0
                self.get_logger().info(f"[AUTO RECOVERY] Gemma response ({elapsed:.1f}s):\n{content}")

                # Clean up any potential markdown code blocks
                if content.startswith("```"):
                    lines = content.splitlines()
                    if lines[0].startswith("```"):
                        lines = lines[1:]
                    if lines[-1].startswith("```"):
                        lines = lines[:-1]
                    content = "\n".join(lines).strip()

                data = json.loads(content)
                explanation = data.get("explanation", "").strip()
                waypoint_name = data.get("new_target", "").strip()

                if not explanation:
                    raise ValueError("Empty explanation in JSON")
                if not waypoint_name:
                    raise ValueError("Empty new_target in JSON")

                # ── Publish explanation ───────────────────────────────
                explain_msg = String()
                explain_msg.data = explanation
                self._explain_pub.publish(explain_msg)
                self.get_logger().info(f"[EXPLAIN] Published explanation to /supervisor/explanation: \"{explanation}\"")

                # ── Trigger TTS ───────────────────────────────────────
                trigger_text_to_speech(explanation)
                self.get_logger().info(f"[TTS] Speaking: \"{explanation}\"")

                # ── Validate and dispatch ─────────────────────────────
                if waypoint_name in OFFICE_WAYPOINTS:
                    self._dispatch_goal(waypoint_name)
                else:
                    self.get_logger().error(
                        f"[AUTO RECOVERY] Waypoint '{waypoint_name}' not in OFFICE_WAYPOINTS. "
                        f"Valid: {WAYPOINT_NAMES}"
                    )

            except Exception as e:
                self.get_logger().error(f"[AUTO RECOVERY] Pipeline failure: {e}")
            finally:
                self._recovery_in_progress.clear()

        else:
            self.get_logger().info(
                f"[STEP1] Compiled state payload:\n{state_text}"
            )

            # ── Build prompt ──────────────────────────────────────────────
            system_prompt = (
                "You are an onboard AI assistant on a Unitree Go2 quadruped robot "
                "navigating an office. The robot's path planner has FAILED. "
                "Given the robot's current sensor data and failed goal, provide "
                "a concise (1-2 sentences), natural-language explanation of why "
                "the robot cannot reach its destination. "
                "Speak in first person as the robot (e.g., 'I cannot pass because…'). "
                "Be specific about which detected objects are likely blocking the path. "
                "Do NOT suggest alternatives – only explain the failure."
            )

            payload = {
                "model": self._model_id,
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": state_text},
                ],
                "max_tokens": 128,
                "temperature": 0.3,
                "stream": False,
            }

            # ── Call Gemma 4 E2B ──────────────────────────────────────────
            try:
                url = f"{self._api_base}/chat/completions"
                self.get_logger().info(f"[STEP1] POST → {url}")

                response = _post_json(url, payload, self._timeout_explain)

                explanation = (
                    response.get("choices", [{}])[0]
                    .get("message", {})
                    .get("content", "")
                    .strip()
                )

                elapsed = time.monotonic() - t0
                self.get_logger().info(
                    f"[STEP1] Gemma response ({elapsed:.1f}s): \"{explanation}\""
                )

                if not explanation:
                    raise ValueError("Empty explanation from Gemma 4 E2B")

            except (urllib.error.URLError, urllib.error.HTTPError) as e:
                self.get_logger().error(
                    f"[STEP1] HTTP error contacting Gemma 4: {e}"
                )
                self._recovery_in_progress.clear()
                return
            except (json.JSONDecodeError, ValueError, KeyError, IndexError) as e:
                self.get_logger().error(
                    f"[STEP1] Failed to parse Gemma 4 response: {e}"
                )
                self._recovery_in_progress.clear()
                return
            except Exception as e:
                self.get_logger().error(
                    f"[STEP1] Unexpected error: {type(e).__name__}: {e}"
                )
                self._recovery_in_progress.clear()
                return

            # ── Publish explanation ───────────────────────────────
            explain_msg = String()
            explain_msg.data = explanation
            self._explain_pub.publish(explain_msg)
            self.get_logger().info(f"[EXPLAIN] Published explanation to /supervisor/explanation: \"{explanation}\"")

            # ── Trigger TTS ───────────────────────────────────────────────
            trigger_text_to_speech(explanation)
            self.get_logger().info(f"[TTS] Speaking: \"{explanation}\"")

            # ── Await human verbal input ──────────────────────────────────
            self._pending_explanation = explanation
            self._awaiting_human_input = True

            self.get_logger().info(
                "[STEP1] ✓ Explanation dispatched to TTS. "
                "Awaiting human speech input on /human/speech_input …"
            )

    # ------------------------------------------------------------------
    # Step 2 – Parse human intent → structured waypoint JSON
    # ------------------------------------------------------------------

    def _step2_parse_intent(self, human_text: str) -> None:
        """
        Query Gemma 4 E2B with the human's verbal command, using
        structured output / tool-calling constraints to extract
        a JSON object:  {"new_target": "WAYPOINT_NAME"}

        Runs on a background thread – NEVER on the executor.
        """
        t0 = time.monotonic()

        self.get_logger().info(
            f"[STEP2] Parsing human intent: \"{human_text}\""
        )

        # ── Build tool definitions for Gemma 4 native tool calling ────
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "navigate_to_waypoint",
                    "description": (
                        "Command the robot to navigate to a named office "
                        "waypoint. Call this function with the waypoint "
                        "that best matches the human operator's request."
                    ),
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "new_target": {
                                "type": "string",
                                "enum": WAYPOINT_NAMES,
                                "description": (
                                    "The exact name of the target waypoint."
                                ),
                            },
                        },
                        "required": ["new_target"],
                    },
                },
            }
        ]

        system_prompt = (
            "You are an onboard navigation assistant on a quadruped robot. "
            "The robot has just explained a path failure to a human operator. "
            "The human will now give you a verbal command indicating where the "
            "robot should go instead. "
            "You MUST call the 'navigate_to_waypoint' function with the "
            "waypoint name that best matches the human's intent. "
            f"Available waypoints: {', '.join(WAYPOINT_NAMES)}. "
            "If the human's request is ambiguous, choose the most likely match. "
            "Do not output any other text – ONLY call the function."
        )

        context = (
            f"Previous failure explanation: {self._pending_explanation}\n"
            f"Human operator says: \"{human_text}\""
        )

        payload = {
            "model": self._model_id,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": context},
            ],
            "tools": tools,
            "tool_choice": {
                "type": "function",
                "function": {"name": "navigate_to_waypoint"},
            },
            "max_tokens": 128,
            "temperature": 0.0,
            "stream": False,
        }

        # ── Call Gemma 4 E2B ──────────────────────────────────────────
        waypoint_name: Optional[str] = None

        try:
            url = f"{self._api_base}/chat/completions"
            self.get_logger().info(f"[STEP2] POST → {url}")

            response = _post_json(url, payload, self._timeout_dispatch)

            # Parse tool call response from vLLM
            waypoint_name = self._extract_waypoint_from_response(response)

            elapsed = time.monotonic() - t0
            self.get_logger().info(
                f"[STEP2] Gemma tool-call result ({elapsed:.1f}s): "
                f"new_target = \"{waypoint_name}\""
            )

        except (urllib.error.URLError, urllib.error.HTTPError) as e:
            self.get_logger().error(
                f"[STEP2] HTTP error contacting Gemma 4: {e}"
            )
        except (json.JSONDecodeError, ValueError, KeyError, IndexError) as e:
            self.get_logger().error(
                f"[STEP2] Failed to parse Gemma 4 response: {e}"
            )
        except Exception as e:
            self.get_logger().error(
                f"[STEP2] Unexpected error: {type(e).__name__}: {e}"
            )

        # ── Validate and dispatch ─────────────────────────────────────
        if waypoint_name and waypoint_name in OFFICE_WAYPOINTS:
            self._dispatch_goal(waypoint_name)
        elif waypoint_name:
            self.get_logger().error(
                f"[STEP2] Waypoint '{waypoint_name}' not in OFFICE_WAYPOINTS. "
                f"Valid: {WAYPOINT_NAMES}"
            )
        else:
            self.get_logger().error(
                "[STEP2] Could not determine waypoint from Gemma response."
            )

        # ── Cleanup ───────────────────────────────────────────────────
        self._pending_explanation = None
        self._recovery_in_progress.clear()

    def _extract_waypoint_from_response(self, response: dict) -> str:
        """
        Extract the waypoint name from Gemma's response.
        Handles both tool_calls format AND plain-text JSON fallback.
        """
        choice = response.get("choices", [{}])[0]
        message = choice.get("message", {})

        # ── Primary path: vLLM tool_calls array ───────────────────────
        tool_calls = message.get("tool_calls")
        if tool_calls and isinstance(tool_calls, list):
            tc = tool_calls[0]
            fn = tc.get("function", {})
            args_raw = fn.get("arguments", "{}")

            # vLLM may return arguments as string or dict
            if isinstance(args_raw, str):
                args = json.loads(args_raw)
            else:
                args = args_raw

            waypoint = args.get("new_target", "").strip()
            if waypoint:
                self.get_logger().debug(
                    f"[STEP2] Extracted via tool_calls: {waypoint}"
                )
                return waypoint

        # ── Fallback: parse JSON from content field ───────────────────
        content = message.get("content", "")
        if content:
            self.get_logger().debug(
                f"[STEP2] No tool_calls; attempting JSON parse from content: "
                f"'{content[:200]}'"
            )
            # Try to find JSON object in the content
            try:
                # Direct parse
                parsed = json.loads(content)
                waypoint = parsed.get("new_target", "").strip()
                if waypoint:
                    return waypoint
            except json.JSONDecodeError:
                pass

            # Try extracting JSON from within text
            import re
            json_match = re.search(r'\{[^{}]*"new_target"\s*:\s*"([^"]+)"[^{}]*\}', content)
            if json_match:
                return json_match.group(1).strip()

        raise ValueError(
            f"Could not extract waypoint from Gemma response. "
            f"Message keys: {list(message.keys())}"
        )

    # =====================================================================
    # Goal dispatch
    # =====================================================================

    def _dispatch_goal(self, waypoint_name: str) -> None:
        """
        Publish a PoseStamped to /goal_pose commanding Nav2 to the
        given waypoint.
        """
        coords = OFFICE_WAYPOINTS[waypoint_name]
        x, y, theta = coords

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert theta to quaternion (yaw only, planar robot)
        import math
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)

        self._goal_pub.publish(goal)

        self._current_goal_name = waypoint_name

        self.get_logger().info(
            f"[DISPATCH] ✓ New Nav2 goal published → {waypoint_name} "
            f"(x={x:.2f}, y={y:.2f}, θ={theta:.2f})"
        )
        self.get_logger().info(
            f"[DISPATCH] Quaternion: z={goal.pose.orientation.z:.4f}, "
            f"w={goal.pose.orientation.w:.4f}"
        )

    # =====================================================================
    # Lifecycle
    # =====================================================================

    def destroy_node(self) -> None:
        self.get_logger().info("[SHUTDOWN] Shutting down LLM thread pool …")
        self._llm_pool.shutdown(wait=False)
        super().destroy_node()


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    node = SupervisorNode()

    # Use MultiThreadedExecutor to prevent blocking the spin thread
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info("[MAIN] Spinning with MultiThreadedExecutor (4 threads)")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[MAIN] KeyboardInterrupt received – shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
