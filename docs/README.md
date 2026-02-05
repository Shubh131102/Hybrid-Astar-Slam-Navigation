# Documentation

Project documentation, reports, and visualizations for Custom SLAM and Hybrid A* Navigation system.

## Contents

**Project_Report.pdf**
- Complete research project report
- Custom SLAM pipeline implementation
- Hybrid A* + MPC navigation stack
- Experimental results and analysis
- Sub-5cm localization accuracy validation
- 25% path efficiency improvement over Nav2

**Presentation.pdf** (or .pptx)
- Project overview slides
- System architecture
- Algorithm comparisons
- Key results and benchmarks

**slam_mapping_rviz.png** (or Image_1.jpeg)
- RViz visualization of SLAM mapping
- Shows occupancy grid map (5cm resolution)
- Green outline indicates mapped boundaries
- Gray areas are free space, black are obstacles
- Demonstrates real-time mapping capability

## Project Overview

**UCR Robotics Research Internship**
- Custom SLAM pipeline with multi-sensor fusion
- Hybrid A* path planning with Dubins paths
- Model Predictive Control for trajectory tracking
- Sub-5cm localization accuracy
- 25% path efficiency improvement over Nav2 baseline

## Key Achievements

**Localization:**
- Average error: 4.2cm
- 95th percentile: <5cm
- Update rate: 10 Hz
- Multi-sensor fusion (LiDAR, Odometry, IMU)

**Path Planning:**
- 25% shorter paths vs Nav2
- 15% faster planning time
- 40% smoother (fewer direction changes)
- 98% success rate

**SLAM Mapping:**
- 5cm resolution occupancy grid
- Real-time loop closure detection
- Pose graph optimization
- Sub-5cm map accuracy

## System Components

### Custom SLAM Pipeline
1. **Feature Extraction:** Harris corner detection + ORB descriptors
2. **Scan Matching:** ICP with 0.5m correspondence distance
3. **Loop Closure:** Distance threshold 0.5m, similarity score 0.7
4. **Pose Graph Optimization:** Levenberg-Marquardt solver
5. **Multi-Sensor Fusion:** LiDAR (60%), Odometry (30%), IMU (10%)

### Hybrid A* Path Planner
1. **Search Algorithm:** Weighted A* with Euclidean heuristic
2. **Kinematic Constraints:** Dubins paths with 0.3m turning radius
3. **Motion Primitives:** 5 primitives (straight, left, right, sharp turns)
4. **Path Smoothing:** Gradient descent with obstacle avoidance
5. **Collision Checking:** 0.05m resolution with 0.1m inflation

### Model Predictive Control
1. **Prediction Horizon:** 20 steps (2.0s)
2. **Control Horizon:** 10 steps
3. **Update Rate:** 20 Hz
4. **Cost Function:** Position, heading, velocity, control effort
5. **Constraints:** Vehicle kinematic and dynamic limits

## RViz Visualization

The included screenshot (Image_1.jpeg) shows:
- **Resolution:** 0.05m (5cm grid cells)
- **Map Size:** ~10m x 10m indoor environment
- **Update Rate:** Real-time mapping
- **Color Scheme:**
  - Gray: Free space (navigable)
  - Black: Obstacles (walls, furniture)
  - Green: Map boundary
  - Darker gray: Unknown/unmapped areas

**Visualization Parameters:**
```yaml
Map:
  Topic: /map
  Color Scheme: map
  Resolution: 0.05
  Alpha: 0.7
```

## Comparison with Nav2 Baseline

| Metric | Nav2 | Custom System | Improvement |
|--------|------|---------------|-------------|
| Localization Error | 8.5cm | 4.2cm | -51% |
| Path Length | 15.2m | 11.4m | -25% |
| Planning Time | 120ms | 102ms | -15% |
| Direction Changes | 12 | 7 | -42% |
| Success Rate | 95% | 98% | +3% |
| Map Resolution | 0.05m | 0.05m | Same |
| Loop Closure Rate | 65% | 78% | +20% |

## Technical Details

**Hardware:**
- Platform: TurtleBot3 Burger
- LiDAR: 360° laser scanner (0.12m - 3.5m range)
- IMU: 100 Hz orientation data
- Odometry: Differential drive encoders

**Software Stack:**
- ROS2 Humble
- Custom SLAM implementation (C++ / Python)
- Hybrid A* planner
- MPC controller
- Nav2 integration

**Algorithms:**
- SLAM: Graph-SLAM with loop closure
- Planning: Hybrid A* with Dubins paths
- Control: Model Predictive Control
- Localization: AMCL + Custom particle filter

## Related Publications

[Add any papers or conference presentations here]

## Citation
```bibtex
@techreport{jangle2024slam,
  title={Custom SLAM and Hybrid A* Navigation for Indoor Mobile Robots},
  author={Jangle, Shubham},
  institution={University of California, Riverside},
  year={2024}
}
```

## Repository

GitHub: [github.com/Shubh131102/slam-navigation-ucr](link)

## Contact

Shubham Jangle  
UC Riverside - Robotics Research Lab  
sjang041@ucr.edu
