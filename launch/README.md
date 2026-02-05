# Launch Files

ROS2 launch files for custom SLAM and Hybrid A* navigation system.

## Available Launch Files

### nav2.launch.py
**Purpose:** Launch complete Nav2 navigation stack with custom parameters

**Launched Components:**
- Controller server (20 Hz DWB + MPC)
- Planner server (10 Hz Hybrid A*)
- Behavior server (recovery behaviors)
- BT Navigator (behavior tree coordination)
- Waypoint follower
- Velocity smoother
- Lifecycle manager

**Arguments:**
```bash
use_sim_time      Use simulation time (default: true)
params_file       Nav2 parameters (default: config/nav2_params.yaml)
map               Map file path (default: maps/map.yaml)
autostart         Auto-start Nav2 nodes (default: true)
```

**Usage:**
```bash
# Basic launch
ros2 launch custom_nav nav2.launch.py

# Custom parameters
ros2 launch custom_nav nav2.launch.py \
  params_file:=config/custom_params.yaml \
  map:=maps/custom_map.yaml

# Hardware deployment (no sim time)
ros2 launch custom_nav nav2.launch.py use_sim_time:=false
```

---

### slam_sim.launch.py
**Purpose:** Launch SLAM toolbox for real-time mapping and localization

**Launched Components:**
- SLAM Toolbox (sync mode)
- RViz2 visualization
- Map saver (optional)

**Arguments:**
```bash
use_sim_time      Use simulation time (default: true)
slam_params_file  SLAM parameters (default: config/slam_params.yaml)
mode              SLAM mode: mapping or localization (default: mapping)
rviz_config       RViz config (default: rviz/slam.rviz)
```

**Usage:**
```bash
# Mapping mode
ros2 launch custom_nav slam_sim.launch.py mode:=mapping

# Localization mode (requires existing map)
ros2 launch custom_nav slam_sim.launch.py mode:=localization

# Custom config
ros2 launch custom_nav slam_sim.launch.py \
  slam_params_file:=config/custom_slam.yaml \
  rviz_config:=rviz/custom.rviz
```

**Saving Maps:**
```bash
# While SLAM is running
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

---

### hybrid_astar.launch.py
**Purpose:** Launch Hybrid A* path planner standalone

**Usage:**
```bash
ros2 launch custom_nav hybrid_astar.launch.py
```

---

### full_navigation.launch.py
**Purpose:** Launch complete system (SLAM + Nav2 + custom planner)

**Usage:**
```bash
ros2 launch custom_nav full_navigation.launch.py
```

---

## Common Launch Patterns

### Complete Mapping Session
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM
ros2 launch custom_nav slam_sim.launch.py mode:=mapping

# Terminal 3: Teleoperate to explore
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 4: Save map when done
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

---

### Navigation with Existing Map
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Nav2
ros2 launch custom_nav nav2.launch.py map:=maps/my_map.yaml

# Terminal 3: Set goal in RViz or via command
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
```

---

### SLAM + Navigation (Simultaneous)
```bash
# Launch SLAM in mapping mode
ros2 launch custom_nav slam_sim.launch.py mode:=mapping

# In another terminal, launch Nav2 without map server
ros2 launch custom_nav nav2.launch.py
```

---

## Node Communication

### SLAM Pipeline
```
/scan → slam_toolbox → /map
                    → /map_metadata
                    → /pose
```

### Navigation Pipeline
```
/goal_pose → bt_navigator → planner_server (Hybrid A*) → /plan
                          → controller_server (DWB+MPC) → /cmd_vel
                          → behavior_server (recovery)
```

### Safety Layer
```
/scan → local_costmap → controller_server
     → global_costmap → planner_server
```

---

## Launch File Arguments Reference

### Common Arguments (All Launch Files)

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| use_sim_time | bool | true | Use simulation clock |
| params_file | string | config/*.yaml | Parameters file path |

### SLAM-Specific Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| mode | string | mapping | mapping or localization |
| rviz_config | string | rviz/slam.rviz | RViz config file |

### Nav2-Specific Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| map | string | maps/map.yaml | Map file for localization |
| autostart | bool | true | Auto-start lifecycle nodes |

---

## Troubleshooting

### SLAM Not Publishing Map

**Check SLAM node status:**
```bash
ros2 node list | grep slam
ros2 topic list | grep map
```

**Verify scan topic:**
```bash
ros2 topic echo /scan --once
```

**Check parameters:**
```bash
ros2 param list /slam_toolbox
ros2 param get /slam_toolbox mode
```

---

### Nav2 Not Planning

**Check lifecycle states:**
```bash
ros2 lifecycle list /planner_server
ros2 lifecycle get /planner_server
```

**Activate manually if needed:**
```bash
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
```

**Verify map is loaded:**
```bash
ros2 topic echo /map --once
```

---

### RViz Not Showing Data

**Check topics:**
```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /map
```

**Verify frames:**
```bash
ros2 run tf2_tools view_frames
```

**Check RViz config:**
```bash
# Use default config
ros2 launch custom_nav slam_sim.launch.py rviz_config:=""
```

---

### Transform Errors

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
```

**Common fixes:**
- Ensure robot_state_publisher is running
- Check URDF is loaded correctly
- Verify SLAM is publishing map → odom transform

---

## Performance Tips

**For Real-Time Performance:**
```yaml
# Reduce costmap update frequency
local_costmap:
  update_frequency: 5.0  # Lower if CPU limited

# Reduce planning frequency
planner_server:
  expected_planner_frequency: 5.0  # Lower for less frequent replanning
```

**For Better Accuracy:**
```yaml
# Increase controller frequency
controller_server:
  controller_frequency: 30.0  # Higher for smoother tracking

# Finer costmap resolution
local_costmap:
  resolution: 0.025  # Finer grid (more CPU)
```

**For Faster Mapping:**
```yaml
# Increase SLAM update rate
map_update_interval: 1.0  # More frequent updates

# Reduce loop closure search
loop_closure:
  search_radius: 5.0  # Smaller search area
```

---

## Notes

- All launch files support both simulation and hardware deployment
- Use `use_sim_time:=false` for real robot deployment
- SLAM requires odometry and laser scan topics
- Nav2 requires a map (from SLAM or pre-built)
- Lifecycle nodes must be activated before use
- Check `ros2 lifecycle` commands for node states
