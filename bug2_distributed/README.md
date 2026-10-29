# Distributed Bug2 Navigation Architecture for iRobot Create 3

**Author:** Group 5  
**Course:** Introduction Project — M.Sc. Sensor System Engineering / Robotics, Hanze University of Applied Sciences  
**Platform:** iRobot Create 3  
**ROS Distribution:** ROS 2 Jazzy Jalisco  

---

## 1. Project Overview

This repository implements a modular, distributed **Bug2 autonomous maze navigation algorithm** for the iRobot Create 3 platform. 

Instead of a monolithic node, the system is decomposed into a **distributed multi-node architecture** communicating over standard **ROS 2 Actions**. This decoupling provides:
- **Separation of Concerns:** Motion control, sensor processing, and high-level strategy run in dedicated nodes.
- **Hardware Agnosticism (Adapter Pattern):** Interchangeable wall-following backends supporting both an external **2D LiDAR** and the robot's **native IR bumper/wall sensors**.
- **Asynchronous Non-Blocking Execution:** Central state machine coordinates long-running actions with real-time feedback and preemption.
- **Fail-Safe Graceful Shutdown:** Full cancellation of internal actuators and zero-velocity burst on `SIGINT` (`Ctrl+C`).

---

## 2. System Architecture

The system consists of three logical layers:

```
                          ┌────────────────────────┐
                          │   Launch Parameters    │
                          │ (goal_x, goal_y, type) │
                          └───────────┬────────────┘
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │     bug2_coordinator      │
                        │ (Brain & Bug2 State Mach.)│
                        └──────┬─────────────┬──────┘
                               │             │
           /go_to_goal Action  │             │  /wall_follow Action
       (create3_interfaces)    │             │  (create3_interfaces)
                               ▼             ▼
          ┌─────────────────────────┐   ┌────────────────────────────────┐
          │   go_to_goal_server     │   │     Selected Wall Follower     │
          │ (Straight Nav & Hit Det)│   │  (LiDAR or IR Adapter Server)  │
          └────────────┬────────────┘   └───────────────┬────────────────┘
                       │                                │
                       ▼                                ▼
          ┌──────────────────────────────────────────────────────────────┐
          │                    iRobot Create 3 Platform                  │
          │         Topics: /{ns}/odom, /{ns}/scan, /{ns}/cmd_vel        │
          └──────────────────────────────────────────────────────────────┘
```

### Node Descriptions

1. **`bug2_coordinator` (Action Client & State Machine):**
   - Records the robot's starting pose ($S$) and computes the global world target point ($G$) from relative offsets (`goal_x`, `goal_y`).
   - Computes the continuous equation of the **M-line** ($A x + B y + C = 0$).
   - Coordinates the Bug2 cycle: transitions between `go_to_goal` and `wall_follow` until $G$ is reached.

2. **`go_to_goal_server` (Action Server):**
   - Action: `/go_to_goal` (`create3_interfaces/action/GoToGoal`)
   - Drives straight along the heading toward the target point with proportional steering.
   - Monitors the forward LiDAR sector ($-20^\circ$ to $+20^\circ$) and corner sector ($-50^\circ$ to $-20^\circ$).
   - If an obstacle is detected within clearance threshold, halts the robot, records the **Hit Point** ($H$), and returns `obstacle_detected = True`.

3. **`wall_follow_server` (LiDAR-Based Wall Follower):**
   - Action: `/wall_follow` (`create3_interfaces/action/Bug2WallFollow`)
   - Sensor: 2D LaserScan (`/{ns}/scan`).
   - Uses a robust 4-state controller with forward-arc cornering specifically tuned for 90-degree orthogonal maze corners with $5\text{ cm}$ robot body clearance.
   - Monitors the leave condition and returns `success = True` once the M-line is re-encountered with forward progress.

4. **`wall_follow_ir_server` (Create 3 Native IR Adapter):**
   - Action: `/wall_follow` (`create3_interfaces/action/Bug2WallFollow`)
   - Sensor: Native Create 3 infrared proximity and wall sensors.
   - **Adapter Pattern:** Serves as an action server to `bug2_coordinator` while acting as an action client to Create 3's internal action server (`/{ns}/wall_follow`, type: `irobot_create_msgs/action/WallFollow`).
   - Monitors odometry while the robot tracks the wall using its native internal controller, canceling the internal goal and stopping the robot the moment a valid Bug2 leave point is detected.

---

## 3. Custom Action Interfaces (`create3_interfaces`)

### `GoToGoal.action`
```text
# Goal
geometry_msgs/Point target_point
---
# Result
bool reached_goal
bool obstacle_detected
geometry_msgs/Point hit_point
string message
---
# Feedback
float64 distance_to_target
```

### `Bug2WallFollow.action`
```text
# Goal
geometry_msgs/Point start_point
geometry_msgs/Point target_point
geometry_msgs/Point hit_point
int8 follow_side               # -1: Right side (default Bug2), +1: Left side
---
# Result
bool success
string message
---
# Feedback
float64 distance_to_m_line
float64 distance_to_target
```

---

## 4. Bug2 Algorithmic Implementation Details

### 4.1 M-Line Formulation
The M-line connects start point S(x_s, y_s) and goal point G(x_g, y_g):
```text
Δx = x_g - x_s
Δy = y_g - y_s

A = -Δy
B =  Δx
C =  Δy·x_s - Δx·y_s
```

Perpendicular distance from the robot's current pose (x_r, y_r) to the M-line:
```text
d_m_line = |A·x_r + B·y_r + C| / √(A² + B²)
```

### 4.2 Leave Condition
The robot leaves wall-following and resumes direct navigation if and only if **all three** conditions are met:
1. **Hit Clearance:** The robot has moved away from the initial hit point (`d_hit ≥ 0.30 m`) to avoid immediately leaving at the boundary.
2. **On M-Line:** Perpendicular distance to M-line is within threshold (`d_m_line ≤ 0.15 m`).
3. **Forward Progress:** Distance to goal has progressed (`d_goal ≤ d_hit_goal - 0.02 m`).

---

## 5. Wall-Following Controller Strategies

### Strategy A: 2D LiDAR 4-State Controller (`wall_follow_type:=lidar`)
Calibrated for the robot's physical dimensions (34 cm × 34 cm round chassis, LiDAR mounted 20 cm from front bumper and 16 cm from lateral sides, targeting 5 cm body-to-wall clearance):

1. **Inner Corner / Dead End Ahead (`Front < 0.28m` or `FrontRight < 0.28m`):** In-place counter-clockwise rotation (linear = 0.0 m/s, angular = +0.50 rad/s).
2. **Too Close to Wall (`Right < 0.18m`):** Forward motion with gentle left steering (linear = 0.12 m/s, angular = +0.25 rad/s).
3. **Outer Corner / Wall Ended (`Right > 0.26m`):** **Forward-arc clockwise turning** (linear = 0.096 m/s, angular = -0.35 rad/s). Moving forward while turning guarantees the robot sweeps around convex maze corners without getting trapped in pure in-place spinning.
4. **Sweet Spot (`0.18m ≤ Right ≤ 0.26m`):** Pure forward driving along the wall (linear = 0.12 m/s, angular = 0.0 rad/s).

### Strategy B: Native IR Adapter Controller (`wall_follow_type:=ir`)
- Delegates motor control directly to Create 3's internal microcontroller behavior tree.
- The wrapper tracks odometry at 10 Hz and monitors the M-line.
- When the leave point is reached, the adapter issues an asynchronous cancellation to the internal firmware and publishes a burst of zero-velocity commands to prevent overshoot.

---

## 6. Build and Installation

### 6.1 Prerequisites
- Ubuntu 24.04 LTS (or target robot OS)
- ROS 2 Jazzy Jalisco
- iRobot Create 3 messages: `sudo apt install ros-jazzy-irobot-create-msgs`

### 6.2 Building from Source
In your ROS 2 workspace:
```bash
cd ~/icreate3_ws/src
# Clone or place group5_ros2_project here

cd ~/icreate3_ws
# 1. Build interfaces first:
colcon build --packages-select create3_interfaces

# 2. Build the distributed package:
colcon build --packages-select bug2_distributed

# 3. Source the workspace:
source install/setup.bash
```

> [!NOTE]
> If adding the package to an existing workspace session, ensure the install prefix is added permanently to `~/.bashrc`:
> ```bash
> echo 'export AMENT_PREFIX_PATH=~/icreate3_ws/install/bug2_distributed:$AMENT_PREFIX_PATH' >> ~/.bashrc
> source ~/.bashrc
> ```

---

## 7. How to Run

The entire architecture is started via a single unified launch file:

### Mode 1: Using 2D LiDAR (Recommended for Labeled Maze Navigation)
```bash
ros2 launch bug2_distributed bug2_distributed_launch.py wall_follow_type:=lidar goal_x:=2.0 goal_y:=0.0
```

### Mode 2: Using Native IR Sensors (Recommended when LiDAR is unavailable)
```bash
ros2 launch bug2_distributed bug2_distributed_launch.py wall_follow_type:=ir goal_x:=2.0 goal_y:=0.0
```

### Configurable Launch Arguments

| Argument | Default | Description |
| :--- | :---: | :--- |
| `namespace` | `robot_9` | Robot namespace prefix |
| `wall_follow_type`| `lidar` | Backend selector: `lidar` or `ir` |
| `goal_x` | `2.5` | Target X coordinate relative to robot start (meters) |
| `goal_y` | `0.0` | Target Y coordinate relative to robot start (meters) |
| `linear_speed` | `0.15` | Forward cruise speed (m/s) |
| `wall_speed` | `0.12` | Wall-following forward speed (m/s) |
| `angular_speed` | `0.50` | In-place turning speed (rad/s) |

---

## 8. Graceful Preemption & Safety

All nodes are instrumented with clean shutdown handling:
- Pressing `Ctrl+C` immediately invokes active goal cancellation on both custom action servers and underlying Create 3 firmware actions.
- Each node sends a 5-packet burst of zero-twist commands (`geometry_msgs/Twist`) on shutdown to prevent lingering motion due to dropped DDS packets.

