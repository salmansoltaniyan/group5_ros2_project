# Group 5 — Autonomous Robotics Project (Hanze UAS)

Master of Science in Sensor System Engineering / Robotics  
**Topic:** Autonomous Bug2 Maze Solver on iRobot Create 3 (ROS 2 Jazzy)

---

## Repository Structure

```
group5_ros2_project/
├── create3_interfaces/       # Custom ROS 2 Actions (GoToGoal, Bug2WallFollow)
├── bug2_distributed/        # Complete 3-node distributed Bug2 architecture
│   └── README.md             # In-depth architectural & algorithmic documentation
├── bug2_solver/             # Standalone baseline single-node navigators (LiDAR & IR)
└── create3_pkg/              # Test scripts and utility nodes
```

---

## Quick Start (Distributed Bug2)

For full architectural details, equations, and design diagrams, see [`bug2_distributed/README.md`](bug2_distributed/README.md).

### 1. Build
```bash
cd ~/icreate3_ws
colcon build --packages-select create3_interfaces bug2_distributed
source install/setup.bash
```

### 2. Run
```bash
# Option A: 2D LiDAR wall following (Custom 4-State Controller)
ros2 launch bug2_distributed bug2_distributed_launch.py wall_follow_type:=lidar goal_x:=2.0 goal_y:=0.0

# Option B: Built-in IR wall following (Create 3 Adapter Pattern)
ros2 launch bug2_distributed bug2_distributed_launch.py wall_follow_type:=ir goal_x:=2.0 goal_y:=0.0
```
