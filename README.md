# SLAM and Autonomous Exploration on TurtleBot3

A TurtleBot3 that maps an unknown indoor space on its own. SLAM Toolbox builds the map, Nav2 plans with A* and avoids obstacles with DWB, and frontier exploration decides where to go next. Built at UC Riverside.

<p align="center">
  <img src="docs/Image%201.jpeg" alt="RViz occupancy grid map built during autonomous exploration" width="560">
</p>

## Results

| | |
|---|---|
| Map accuracy improvement from tuning frontier exploration | **15%** |
| Real-time mapping rate | 10 Hz |
| Localization | Stable over 10 m trajectories |

## Stack

| Component | Implementation |
|---|---|
| Mapping | SLAM Toolbox |
| Localization | AMCL |
| Global planning | A* |
| Local planning | DWB |
| Navigation | Nav2 |
| Sensor | 2D LiDAR |
| Platform | TurtleBot3 Burger, Gazebo |

## What's in this repo

| Path | Contents |
|---|---|
| `launch/slam_sim.launch.py` | Starts Gazebo, the robot, and SLAM Toolbox |
| `launch/nav2.launch.py` | Starts Nav2 with the tuned parameters |
| `config/slam_params.yaml` | SLAM Toolbox parameters |
| `config/nav2_params.yaml` | Costmap, planner, and controller parameters |
| `docs/` | Map screenshots |

## Run it

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch launch/slam_sim.launch.py
ros2 launch launch/nav2.launch.py
```

## License

MIT
