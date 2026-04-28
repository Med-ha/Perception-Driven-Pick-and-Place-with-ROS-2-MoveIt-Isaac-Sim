# Perception-Driven-Pick-and-Place-with-ROS-2-MoveIt-Isaac-Sim

## 📌 Overview

This project implements a **perception-driven robotic manipulation pipeline** using:

* ROS 2 (Jazzy)
* Isaac Sim (5.1)
* MoveIt 2
* SO101 robotic arm

The system detects a **red cup using RGB-D perception**, plans motion using MoveIt, grasps it, and places it into a box — all orchestrated through a Behavior Tree.

⚠️ **Note:**
This project was developed as a **rapid prototype (~40 hours)** and is not a production-ready system. The goal was to demonstrate **end-to-end robotics integration under time constraints**, not perfect robustness.

---

## 🎯 What This Project Demonstrates

* End-to-end robotics pipeline: perception → planning → execution
* Integration of ROS 2, MoveIt, and Isaac Sim
* Behavior Tree-based task orchestration
* Debugging and handling real-world integration challenges

---

## 🏗 System Architecture

```text
Camera → perception_node → /red_cup_pose
        ↓
movegroup_executor (target generation)
        ↓
MoveIt planning
        ↓
trajectory_to_isaac
        ↓
Robot execution in Isaac Sim
```

Supporting components:

* `isaac_state_relay` → syncs simulation joint states to ROS
* `bt_node` → controls full task execution

---

## 🔁 Behavior Tree Flow

```text
WaitForSystemReady
→ WaitForRedCup
→ safe_home
→ open
→ plan_to_cup
→ align_to_grasp
→ close
→ attach
→ lift_cup
→ above_box
→ drop_in_box
→ open
→ detach
→ above_box
→ safe_home
```

---

## 🎥 Demo

📹 Demo video: `demo/Demo.mp4`

> The demo shows approximate end-to-end execution.
> The system is functional but not fully stable due to simulation and timing constraints.

---

## ⚙️ Setup & Run

### 1. Start Isaac Sim

```bash
cd ~/IsaacSim
./isaac-sim.sh
```

### 2. Launch MoveIt

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fireloop_assignment/so-arm/so101_ws
source install/setup.bash

ros2 launch so101_bringup bringup_moveit.launch.py use_fake_hardware:=true use_sim_time:=false
```

### 3. Run Full Pipeline

```bash
ros2 launch so101_state_machine so101_pipeline.launch.py
```

---

## 🧪 Useful Debug Commands

```bash
ros2 topic echo /red_cup_pose

ros2 topic pub --once /executor_command std_msgs/msg/String "{data: safe_home}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: plan_to_cup}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: close}"
ros2 topic pub --once /executor_command std_msgs/msg/String "{data: attach}"
```

---

## ⚠️ Known Limitations

* Perception uses color segmentation → sensitive to lighting
* Grasping relies on reference joint offsets (not fully generalizable)
* No collision-aware planning (MoveIt planning scene not used)
* Behavior Tree lacks recovery/retry logic
* Grasp attachment can be slightly unstable in simulation
* Execution depends on timing (e.g., joint states availability)

---

## 💡 Key Learnings

* Integrating perception with motion planning requires careful TF handling
* Isaac Sim grasping is non-trivial; physics joints can break simulation
* MoveIt depends heavily on accurate joint state feedback
* Behavior Trees are powerful but require failure handling for robustness
* System-level integration is significantly harder than individual modules

---

## 🚀 Future Improvements

* Add collision objects and planning scene integration
* Improve perception robustness (ML-based detection)
* Implement grasp pose estimation instead of joint offsets
* Add recovery behaviors in Behavior Tree
* Improve attach/detach stability

---

## 📌 Why This Project

This project was built to explore **full-stack robotics integration** — combining perception, motion planning, and simulation into a single working pipeline.

While not perfect, it demonstrates:

* Strong system-level understanding
* Ability to debug across multiple subsystems
* Rapid prototyping under real constraints

---

## 📄 License

MIT License
