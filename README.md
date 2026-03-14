# SLAM Navigation Stack — UCR Robotics Project

Full SLAM and navigation stack on TurtleBot3 in Gazebo simulation and 
real-world testbed — covering mapping, localization, path planning, and 
Nav2 integration.

---

## Key Results

- 15% map accuracy improvement via Nav2 frontier exploration parameter tuning
- 10Hz real-time mapping performance
- Stable localization over 10m trajectories
- Successful autonomous exploration of unknown indoor environments

---

## Stack Overview

| Component | Implementation |
|-----------|---------------|
| Mapping | SLAM Toolbox, Gmapping, Cartographer |
| Localization | AMCL with adaptive resampling |
| Global Planning | A* |
| Local Planning | DWB, TEB |
| Navigation | Nav2 |
| Sensor | LiDAR (LaserScan) |
| Platform | TurtleBot3 Burger |

---

## Repo Layout
```
slam_nav_tb3/
├── src/          # ROS2 packages and nodes
├── launch/       # Bringup and Nav2 launch files
├── config/       # Costmap, planner, controller parameters
├── maps/         # Saved maps (YAML + PGM)
├── scripts/      # Helper scripts (bag-to-map, teleop logging)
├── media/        # GIF and MP4 demos
└── docs/         # Project report and diagrams
```

---

## Quick Start
```bash
# Launch simulation
ros2 launch slam_nav_tb3 slam_sim.launch.py

# Run Nav2 with saved map
ros2 launch slam_nav_tb3 nav2.launch.py map:=maps/ucr_lab.yaml

# Build workspace
cd ~/slam_ws
colcon build
source install/setup.bash
```

---

## Key Components

### SLAM Mapping
- SLAM Toolbox for real-time occupancy grid mapping
- Frontier-based autonomous exploration via Nav2
- Parameter tuning achieved 15% accuracy improvement over baseline

### Localization
- AMCL particle filter for pose estimation
- Adaptive resampling for dynamic environments
- Validated over 10m trajectories in UCR lab

### Path Planning
- Global: A* for optimal path generation
- Local: DWB for real-time obstacle avoidance
- Velocity constraints tuned for safe indoor navigation

---

## Configuration

Key parameter files in config/:
- costmap_params.yaml — Obstacle inflation, footprint
- planner_params.yaml — A* and DWB tuning
- controller_params.yaml — Velocity limits and constraints
- slam_params.yaml — SLAM Toolbox configuration

---

## Troubleshooting

Map not building:
```bash
ros2 topic echo /scan        # Verify LiDAR data
ros2 topic echo /odom        # Verify odometry
ros2 run tf2_tools view_frames  # Check TF tree
```

Nav2 not navigating:
```bash
ros2 topic list | grep nav   # Check Nav2 topics active
ros2 node list               # Verify all nodes running
```

Localization drift:
- Increase AMCL particle count in config
- Verify map matches environment
- Check odometry calibration

---

## Notes

- Test in simulation before real hardware deployment
- Save maps after successful exploration runs
- Tune local planner parameters for different environments
- Source workspace before running any ROS2 commands
