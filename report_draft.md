# Language-Driven Mobile Manipulation: A Husky–UR5 System with LLM Agent and GPU-Accelerated Perception

**Author:** Jun Du
**Course Project — Spring 2026**

---

## 1. Introduction

Service robots that operate in human environments must combine three traditionally separate capabilities: autonomous navigation, dexterous manipulation, and high-level reasoning over natural-language goals. This project implements a complete simulated mobile-manipulation stack that unifies these capabilities on a Clearpath Husky UGV equipped with a Universal Robots UR5 arm and a Robotiq Hand-E gripper. A user issues a free-form English command — for example, *"Go to the kitchen and bring me the bottle on the dining table"* — and the system autonomously decomposes the command into navigation, perception, and grasping primitives, executing them in an AWS RoboMaker small-house Gazebo environment.

The contribution of this project is the integration: rather than a single novel algorithm, the work demonstrates that a Large Language Model (LLM) acting as a tool-using agent can replace hand-coded task planners and orchestrate a conventional ROS Noetic stack (move_base, MoveIt, AMCL/SLAM Toolbox) with GPU-accelerated YOLOv8 perception. The system runs end-to-end in real time on a single workstation.

## 2. System Architecture

The architecture (Figure 1) is layered. At the top, a natural-language command from a Flask web UI is forwarded to a `RobotAgent` built on the OpenAI tool-calling API. The agent has access to four high-level tools — `drive_to_location`, `pick_object`, `place_object`, and `list_locations` — backed by a `MasterControl` Python class that exposes typed ROS-side primitives. `MasterControl` in turn dispatches to three subsystems:

- **Navigation:** `move_base` with a navfn global planner and DWA local planner. Localization uses either AMCL on a pre-built map or SLAM Toolbox for online pose-graph SLAM. State estimation fuses wheel odometry and IMU through `robot_localization`'s EKF.
- **Manipulation:** MoveIt with the OMPL RRTConnect planner, a KDL inverse-kinematics solver, and an OctoMap collision environment built from the depth camera. The Robotiq Hand-E is driven through a joint-trajectory controller.
- **Perception:** A C++ ROS node (`trt_yolo_node`) wraps a YOLOv8m ONNX model compiled to a TensorRT engine. It exposes a `DetectObjects` service over 640×640 RGB frames; 3D object position and diameter are recovered by back-projecting the detection bounding box into the registered depth `PointCloud2`.

A YAML *semantic map* (`config/semantic_map.yaml`) holds roughly fifty named poses across five rooms (bedroom, living room, kitchen, dining room, study), giving the LLM a stable vocabulary of locations to reason about without ever exposing raw `(x, y, yaw)` coordinates.

## 3. Method

### 3.1 Robot Description and Stability

The URDF (`urdf/husky_ur5.urdf.xacro`) composes the Husky base, a SICK LMS1XX LiDAR at 25.7 cm, the UR5 arm on a top plate, and the Hand-E at `tool0`. A 30 kg virtual ballast was added below the chassis to prevent tipping when the arm is fully extended laterally — without it, the arm's 850 mm reach combined with grasp loads pitched the platform forward in Gazebo.

### 3.2 LLM Tool-Calling Loop

The agent uses an OpenAI-compatible client (DeepSeek, Groq, or local Ollama). Each user turn appends a message to a rolling conversation history; the model returns either a textual answer or a structured tool call. Tool calls are executed synchronously by `MasterControl`, the result is appended back into the history as a `tool` message, and the loop repeats until the model emits a final answer. This pattern keeps planning policy entirely in-prompt — no PDDL, no behavior tree — and lets the user re-task the robot mid-execution.

### 3.3 Perception-to-Grasp Pipeline

When `pick_object(class_name)` is invoked, the arm first moves to a "look" pose. A YOLO request returns the highest-confidence bounding box for the target class. The 2D box center is unprojected against the latest filtered `PointCloud2` to recover a 3D centroid and an approximate object diameter; the diameter parameterizes the Hand-E target width with a small safety margin. MoveIt then plans a top-down approach, the gripper closes, and a retreat trajectory lifts the object before navigation resumes.

### 3.4 User Interface

The Flask UI at `localhost:5000` exposes `/api/command`, `/api/map`, `/api/save_pose`, and `/api/estop`. Operators can teach new semantic locations on the fly by driving the robot (manually or via a navigation command) and POSTing the current TF pose under a chosen name; the YAML semantic map is rewritten atomically.

## 4. Evaluation

The system was evaluated qualitatively in the AWS RoboMaker small-house world. End-to-end tasks of the form *"go to room X, pick object Y, place it at location Z"* succeed reliably for objects the YOLOv8m COCO classes recognize (bottle, cup, book, apple, sports ball). YOLO inference runs at ~12 ms per frame on TensorRT FP16, giving headroom for closed-loop visual servoing. The dominant failure modes observed were (a) MoveIt IK rejections when the chosen object lies outside the UR5's reachable workspace from the current base pose — partially mitigated by a heuristic that re-positions the base when planning fails — and (b) AMCL pose drift in long corridors, addressed by switching to SLAM Toolbox localization mode.

## 5. Discussion and Future Work

The project demonstrates that a modern LLM with tool-calling can act as a competent task planner for a non-trivial mobile manipulator without bespoke symbolic infrastructure, provided the action space is exposed as well-typed Python tools and the world is described through a curated semantic vocabulary. The most promising extensions are: (1) replacing the COCO-trained YOLO with an open-vocabulary detector (e.g., Grounding DINO) so the LLM is no longer constrained to the 80 COCO classes; (2) closing the loop on grasping with a learned 6-DoF grasp predictor instead of the current top-down heuristic; and (3) porting the stack to a real Husky+UR5 platform, where sim-to-real transfer of the perception module is expected to be the principal challenge.

## 6. Conclusion

A complete language-driven mobile-manipulation pipeline was built on ROS Noetic, integrating LLM-based task planning, MoveIt motion planning, the ROS Navigation Stack, and TensorRT YOLOv8 perception, all coordinated through a single `MasterControl` API and a Flask web UI. The system executes natural-language pick-and-place tasks in a realistic simulated home and is structured so that each subsystem — agent, planner, perception — can be swapped independently for future research.

---

*Code and demo videos: see project repository `learning_ws/`.*
