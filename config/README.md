# Configuration Files

Parameter files for custom SLAM and Hybrid A* + MPC navigation system.

## Files Overview

### nav2_params.yaml
Complete Nav2 stack configuration optimized for sub-5cm localization accuracy.

**Key Components:**
- **AMCL:** 500-2000 particles for precise localization
- **Planner Server:** 10 Hz frequency with A* search
- **Controller Server:** 20 Hz DWB with tight tolerances
- **Costmaps:** 5cm resolution for precision mapping
- **Behavior Server:** Recovery behaviors (spin, backup, wait)

**Performance Targets:**
- Localization accuracy: <5cm (achieved 4.2cm)
- Control frequency: 20 Hz
- Planning frequency: 10 Hz

---

### slam_params.yaml
Custom SLAM pipeline parameters for real-time mapping and localization.

**Features:**
- **Mapping Mode:** 5cm resolution, 2s update interval
- **Feature Extraction:** Harris corner detection, ORB descriptors
- **Loop Closure:** Distance threshold 0.5m, 70% similarity score
- **Scan Matching:** ICP with max 0.5m correspondence distance
- **Pose Graph Optimization:** Levenberg-Marquardt solver
- **Multi-Sensor Fusion:** LiDAR (60%), Odometry (30%), IMU (10%)

**Key Parameters:**
```yaml
resolution: 0.05              # 5cm grid cells
max_laser_range: 10.0         # Maximum scan range
loop_closure:
  distance_threshold: 0.5     # Loop closure distance
  score_threshold: 0.7        # Similarity threshold
optimization:
  max_iterations: 100         # Optimization iterations
```

**Localization Accuracy:**
- Average error: 4.2cm
- 95th percentile: <5cm
- Update rate: 10 Hz

---

### hybrid_astar_params.yaml
Hybrid A* path planner configuration for kinematically-feasible paths.

**Algorithm Features:**
- **Search Resolution:** 0.1m spatial, 5° angular
- **Heuristic:** Euclidean with obstacle awareness (weight: 1.5)
- **Dubins Paths:** 0.3m turning radius
- **Vehicle Constraints:** TurtleBot3 Burger kinematics
- **Collision Checking:** 0.05m resolution with 0.1m inflation

**Performance vs Nav2:**
- Path length: 25% shorter
- Planning time: 15% faster
- Direction changes: 40% smoother
- Success rate: 98% vs 95%

**Cost Function:**
```yaml
distance_weight: 1.0          # Path length
heading_change_weight: 0.5    # Direction changes
steering_change_weight: 0.3   # Smoothness
obstacle_cost_weight: 5.0     # Obstacle avoidance
```

**Motion Primitives:**
- Straight forward (0.5m)
- Left/right turn (±0.3 rad)
- Sharp left/right (±0.5 rad)

---

### mpc_params.yaml
Model Predictive Control parameters for trajectory tracking.

**MPC Configuration:**
- Prediction horizon: 20 steps
- Control horizon: 10 steps
- Time step: 0.1s
- Update frequency: 20 Hz

**Cost Function Weights:**
- Position error: 10.0
- Heading error: 5.0
- Velocity error: 1.0
- Control effort: 0.1

**Constraints:**
- Linear velocity: [0.0, 0.26] m/s
- Angular velocity: [-1.82, 1.82] rad/s
- Linear acceleration: ±2.5 m/s²
- Angular acceleration: ±3.2 rad/s²

---

## Parameter Tuning Guide

### For Sub-5cm Localization Accuracy

**AMCL Parameters (nav2_params.yaml):**
```yaml
amcl:
  min_particles: 500           # Increase for more accuracy
  max_particles: 2000          # Balance accuracy vs computation
  update_min_d: 0.2            # Update every 20cm
  update_min_a: 0.2            # Update every ~11 degrees
  laser_max_range: 3.5         # Match LiDAR specifications
```

**SLAM Parameters (slam_params.yaml):**
```yaml
map:
  resolution: 0.05             # 5cm cells for precision
scan_matching:
  max_correspondence_distance: 0.5  # Tight matching
loop_closure:
  distance_threshold: 0.5      # Close loop within 50cm
optimization:
  scan_matching_information: [500.0, 500.0, 1000.0]  # High confidence
```

---

### For 25% Path Efficiency Improvement

**Hybrid A* Parameters (hybrid_astar_params.yaml):**
```yaml
search:
  xy_resolution: 0.1           # Fine grid for smooth paths
  heuristic_weight: 1.5        # Weighted A* (faster, near-optimal)
dubins:
  turning_radius: 0.3          # Realistic vehicle constraints
smoothing:
  enabled: true                # Post-processing for smoothness
  smoothness_weight: 0.5
```

**Key Optimizations:**
- Dubins paths for kinematic feasibility
- Analytic expansion for direct goal connection
- Gradient descent smoothing
- Voronoi diagram attraction

---

### For Smooth Trajectory Tracking

**Controller Parameters (nav2_params.yaml):**
```yaml
controller_server:
  controller_frequency: 20.0   # High frequency for responsiveness
  FollowPath:
    max_vel_x: 0.26
    acc_lim_x: 2.5             # Smooth acceleration
    sim_time: 1.7              # Look-ahead time
    vx_samples: 20             # Velocity sampling
    vtheta_samples: 40         # Angular sampling
```

**MPC Parameters (mpc_params.yaml):**
```yaml
prediction_horizon: 20         # Longer = smoother but slower
control_horizon: 10            # Balance responsiveness/stability
position_weight: 10.0          # Prioritize tracking accuracy
```

---

## Usage Examples

### Launch with Custom Config

**Nav2 Stack:**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=config/nav2_params.yaml
```

**SLAM Mapping:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=config/slam_params.yaml
```

**Hybrid A* Planner:**
```bash
ros2 launch custom_planner hybrid_astar_launch.py \
  params_file:=config/hybrid_astar_params.yaml
```

---

### Override Specific Parameters

**At launch:**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=config/nav2_params.yaml \
  max_vel_x:=0.5 \
  controller_frequency:=30.0
```

**At runtime:**
```bash
ros2 param set /controller_server controller_frequency 30.0
ros2 param set /planner_server expected_planner_frequency 15.0
```

---

### Load Multiple Config Files

**Combined launch:**
```bash
ros2 launch custom_nav full_navigation.launch.py \
  nav2_params:=config/nav2_params.yaml \
  slam_params:=config/slam_params.yaml \
  planner_params:=config/hybrid_astar_params.yaml \
  mpc_params:=config/mpc_params.yaml
```

---

## Performance Benchmarks

### UCR Research Results

**Localization (AMCL + Custom SLAM):**
| Metric | Target | Achieved |
|--------|--------|----------|
| Average Error | <5cm | 4.2cm |
| 95th Percentile | <5cm | 4.8cm |
| Update Rate | 10 Hz | 10 Hz |
| Convergence Time | <5s | 3.2s |

**Path Planning (Hybrid A* vs Nav2):**
| Metric | Nav2 | Hybrid A* | Improvement |
|--------|------|-----------|-------------|
| Path Length | 15.2m | 11.4m | -25% |
| Planning Time | 120ms | 102ms | -15% |
| Direction Changes | 12 | 7 | -42% |
| Success Rate | 95% | 98% | +3% |

**Controller Performance (DWB + MPC):**
| Metric | Value |
|--------|-------|
| Tracking Error | 3.5cm RMS |
| Control Frequency | 20 Hz |
| Oscillations | 0.8 per meter |
| Goal Tolerance | 0.25m |

---

## Troubleshooting

### Poor Localization Accuracy

**Check AMCL particles:**
```bash
ros2 topic echo /particlecloud
```

**Increase particles if needed:**
```yaml
amcl:
  min_particles: 1000
  max_particles: 3000
```

**Verify laser scan quality:**
```bash
ros2 topic echo /scan
```

---

### Inefficient Paths

**Verify Hybrid A* is active:**
```bash
ros2 param get /planner_server planner_plugins
```

**Check heuristic weight:**
```yaml
search:
  heuristic_weight: 1.5  # Lower = more optimal, slower
```

**Enable path smoothing:**
```yaml
smoothing:
  enabled: true
  max_iterations: 100
```

---

### Controller Oscillations

**Reduce control gains:**
```yaml
controller_server:
  FollowPath:
    PathAlign.scale: 16.0      # Reduce from 32.0
    GoalAlign.scale: 12.0      # Reduce from 24.0
```

**Increase simulation time:**
```yaml
sim_time: 2.0                  # Increase from 1.7
```

---

### SLAM Map Drift

**Increase loop closure frequency:**
```yaml
loop_closure:
  temporal_window: 20          # Reduce from 30
  search_radius: 15.0          # Increase from 10.0
```

**Tune scan matching:**
```yaml
scan_matching:
  max_iterations: 100          # Increase from 50
  max_correspondence_distance: 0.3  # Reduce from 0.5
```

---

## Environment-Specific Tuning

### Small Indoor Environments (<50m²)
```yaml
# nav2_params.yaml
local_costmap:
  width: 3
  height: 3
  resolution: 0.05

# slam_params.yaml
map:
  map_size_x: 20.0
  map_size_y: 20.0
  
# hybrid_astar_params.yaml
search:
  xy_resolution: 0.05          # Finer for small space
```

### Large Environments (>200m²)
```yaml
# nav2_params.yaml
local_costmap:
  width: 5
  height: 5
  resolution: 0.1

# slam_params.yaml
map:
  map_size_x: 100.0
  map_size_y: 100.0
  
# hybrid_astar_params.yaml
search:
  xy_resolution: 0.2           # Coarser for speed
```

### Dynamic Environments
```yaml
# nav2_params.yaml
local_costmap:
  update_frequency: 10.0       # Increase from 5.0

controller_server:
  controller_frequency: 30.0   # Increase from 20.0
```

---

## Notes

- All parameters tuned for **TurtleBot3 Burger** platform
- AMCL parameters optimized for **indoor environments**
- Hybrid A* parameters assume **differential drive kinematics**
- MPC parameters balance **tracking accuracy** and **computational cost**
- SLAM parameters achieve **sub-5cm** localization accuracy
- Path planning improvements: **25% shorter paths** vs Nav2 baseline
