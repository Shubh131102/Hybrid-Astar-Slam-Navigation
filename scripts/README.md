# Scripts Directory

Utility scripts for data collection, training, evaluation, and visualization.

## Structure
```
scripts/
├── data_collection/
│   ├── collect_rosbag.py
│   ├── extract_trajectories.py
│   └── process_slam_data.py
├── evaluation/
│   ├── evaluate_localization.py
│   ├── evaluate_planning.py
│   ├── benchmark_performance.py
│   └── compare_planners.py
├── visualization/
│   ├── plot_trajectories.py
│   ├── plot_localization_error.py
│   ├── plot_path_comparison.py
│   └── generate_heatmaps.py
└── utils/
    ├── map_tools.py
    ├── coordinate_transforms.py
    └── statistics.py
```

## Data Collection Scripts

### collect_rosbag.py
Record ROS2 bag files for evaluation and analysis.

**Usage:**
```bash
python scripts/data_collection/collect_rosbag.py \
  --output data/slam_session1.bag \
  --duration 300 \
  --topics /scan /odom /map /tf
```

**Arguments:**
- `--output`: Output bag file path
- `--duration`: Recording duration in seconds
- `--topics`: List of topics to record

**Example:**
```python
import subprocess

topics = ['/scan', '/odom', '/cmd_vel', '/map', '/tf', '/tf_static']
duration = 300  # 5 minutes

cmd = ['ros2', 'bag', 'record', '-o', 'data/session1'] + topics
subprocess.run(cmd)
```

---

### extract_trajectories.py
Extract robot trajectories from ROS2 bags for analysis.

**Usage:**
```bash
python scripts/data_collection/extract_trajectories.py \
  --input data/nav_session.bag \
  --output results/trajectory.csv
```

**Output format (CSV):**
```csv
timestamp,x,y,theta,v_linear,v_angular
1234567.89,0.0,0.0,0.0,0.1,0.0
1234567.99,0.01,0.0,0.0,0.1,0.0
```

---

### process_slam_data.py
Process SLAM data for ground truth comparison.

**Usage:**
```bash
python scripts/data_collection/process_slam_data.py \
  --input data/slam_session.bag \
  --output results/slam_poses.npy
```

---

## Evaluation Scripts

### evaluate_localization.py
Compute localization accuracy metrics.

**Usage:**
```bash
python scripts/evaluation/evaluate_localization.py \
  --ground_truth data/ground_truth.csv \
  --estimated data/slam_poses.csv \
  --output results/localization_metrics.json
```

**Metrics computed:**
- Mean error
- Standard deviation
- 95th percentile error
- Maximum error
- RMS error

**Output (JSON):**
```json
{
  "mean_error": 0.042,
  "std_dev": 0.015,
  "percentile_95": 0.048,
  "max_error": 0.065,
  "rmse": 0.044
}
```

**Example:**
```python
import numpy as np

def compute_localization_error(ground_truth, estimated):
    """Compute localization error metrics."""
    errors = np.linalg.norm(ground_truth[:, :2] - estimated[:, :2], axis=1)
    
    metrics = {
        'mean_error': float(np.mean(errors)),
        'std_dev': float(np.std(errors)),
        'percentile_95': float(np.percentile(errors, 95)),
        'max_error': float(np.max(errors)),
        'rmse': float(np.sqrt(np.mean(errors**2)))
    }
    
    return metrics
```

---

### evaluate_planning.py
Evaluate path planning performance.

**Usage:**
```bash
python scripts/evaluation/evaluate_planning.py \
  --nav2_path results/nav2_path.csv \
  --custom_path results/hybrid_astar_path.csv \
  --output results/planning_comparison.json
```

**Metrics:**
- Path length
- Planning time
- Number of direction changes
- Smoothness (curvature variance)
- Success rate

**Output:**
```json
{
  "nav2": {
    "path_length": 15.2,
    "planning_time": 0.120,
    "direction_changes": 12,
    "smoothness": 0.85
  },
  "hybrid_astar": {
    "path_length": 11.4,
    "planning_time": 0.102,
    "direction_changes": 7,
    "smoothness": 0.95
  },
  "improvement": {
    "path_length": -25.0,
    "planning_time": -15.0,
    "direction_changes": -41.7,
    "smoothness": 11.8
  }
}
```

---

### benchmark_performance.py
System-wide performance benchmarking.

**Usage:**
```bash
python scripts/evaluation/benchmark_performance.py \
  --duration 300 \
  --output results/benchmark.json
```

**Metrics:**
- Topic frequencies (Hz)
- CPU usage per node
- Memory consumption
- Message latencies
- Network bandwidth

---

### compare_planners.py
Compare multiple path planners side-by-side.

**Usage:**
```bash
python scripts/evaluation/compare_planners.py \
  --planners nav2 hybrid_astar rrt_star \
  --test_cases data/test_scenarios/ \
  --output results/planner_comparison.json
```

---

## Visualization Scripts

### plot_trajectories.py
Visualize robot trajectories on map.

**Usage:**
```bash
python scripts/visualization/plot_trajectories.py \
  --map maps/map.yaml \
  --trajectory results/trajectory.csv \
  --output media/images/trajectory.png
```

**Example:**
```python
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

def plot_trajectory(map_img, trajectory, output_path):
    """Plot trajectory overlay on map."""
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Display map
    ax.imshow(map_img, cmap='gray', origin='lower')
    
    # Plot trajectory
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=2, label='Robot Path')
    ax.scatter(trajectory[0, 0], trajectory[0, 1], c='g', s=100, label='Start')
    ax.scatter(trajectory[-1, 0], trajectory[-1, 1], c='b', s=100, label='Goal')
    
    ax.legend()
    ax.axis('equal')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
```

---

### plot_localization_error.py
Visualize localization error over time.

**Usage:**
```bash
python scripts/visualization/plot_localization_error.py \
  --errors results/localization_errors.csv \
  --output media/images/localization_accuracy.png
```

**Generates:**
- Error vs time plot
- Error distribution histogram
- Cumulative distribution function

---

### plot_path_comparison.py
Compare paths from different planners.

**Usage:**
```bash
python scripts/visualization/plot_path_comparison.py \
  --nav2 results/nav2_path.csv \
  --hybrid_astar results/hybrid_astar_path.csv \
  --map maps/map.yaml \
  --output media/images/path_comparison.png
```

**Features:**
- Overlay multiple paths on map
- Color-coded by planner
- Start/goal markers
- Path length annotations

---

### generate_heatmaps.py
Generate visualization heatmaps.

**Usage:**
```bash
python scripts/visualization/generate_heatmaps.py \
  --data results/costmap_data.npy \
  --output media/images/costmap_heatmap.png
```

**Heatmap types:**
- Costmap visualization
- Occupancy probability
- Visited areas frequency
- Planning time heatmap

---

## Utility Scripts

### map_tools.py
Map manipulation utilities.

**Functions:**
```python
def load_map(yaml_path):
    """Load map from YAML and PGM files."""
    pass

def save_map(map_data, yaml_path):
    """Save map to YAML and PGM files."""
    pass

def crop_map(map_data, x_min, x_max, y_min, y_max):
    """Crop map to region of interest."""
    pass

def merge_maps(map1, map2):
    """Merge two maps."""
    pass
```

---

### coordinate_transforms.py
Coordinate transformation utilities.

**Functions:**
```python
def map_to_world(pixel_coords, resolution, origin):
    """Convert map pixel coordinates to world coordinates."""
    pass

def world_to_map(world_coords, resolution, origin):
    """Convert world coordinates to map pixel coordinates."""
    pass

def transform_pose(pose, transform):
    """Transform pose using TF transform."""
    pass
```

---

### statistics.py
Statistical analysis utilities.

**Functions:**
```python
def compute_statistics(data):
    """Compute comprehensive statistics."""
    return {
        'mean': np.mean(data),
        'std': np.std(data),
        'median': np.median(data),
        'min': np.min(data),
        'max': np.max(data),
        'percentile_25': np.percentile(data, 25),
        'percentile_75': np.percentile(data, 75),
        'percentile_95': np.percentile(data, 95)
    }

def confidence_interval(data, confidence=0.95):
    """Compute confidence interval."""
    pass
```

---

## Complete Evaluation Pipeline

### Step-by-Step Workflow
```bash
# 1. Collect data
python scripts/data_collection/collect_rosbag.py \
  --output data/eval_session.bag \
  --duration 300

# 2. Extract trajectories
python scripts/data_collection/extract_trajectories.py \
  --input data/eval_session.bag \
  --output results/trajectory.csv

# 3. Evaluate localization
python scripts/evaluation/evaluate_localization.py \
  --ground_truth data/ground_truth.csv \
  --estimated results/trajectory.csv \
  --output results/localization_metrics.json

# 4. Evaluate planning
python scripts/evaluation/evaluate_planning.py \
  --nav2_path results/nav2_path.csv \
  --custom_path results/hybrid_astar_path.csv \
  --output results/planning_comparison.json

# 5. Generate visualizations
python scripts/visualization/plot_trajectories.py \
  --map maps/map.yaml \
  --trajectory results/trajectory.csv \
  --output media/images/trajectory.png

python scripts/visualization/plot_path_comparison.py \
  --nav2 results/nav2_path.csv \
  --hybrid_astar results/hybrid_astar_path.csv \
  --map maps/map.yaml \
  --output media/images/path_comparison.png

# 6. Benchmark performance
python scripts/evaluation/benchmark_performance.py \
  --duration 300 \
  --output results/benchmark.json
```

---

## Requirements
```
numpy>=1.19.0
matplotlib>=3.3.0
pandas>=1.2.0
scipy>=1.6.0
pillow>=8.0.0
pyyaml>=5.4.0
rosbag2-py
```

Install: `pip install -r requirements.txt`

---

## Example Results

### UCR Research Results

**Localization Accuracy:**
```json
{
  "mean_error": 0.042,
  "std_dev": 0.015,
  "percentile_95": 0.048,
  "rmse": 0.044
}
```

**Path Planning Comparison:**
```json
{
  "path_length_improvement": -25.0,
  "planning_time_improvement": -15.0,
  "smoothness_improvement": 11.8
}
```

---

## Notes

- All scripts assume ROS2 workspace is sourced
- Data collection requires active ROS2 system
- Visualization scripts generate publication-quality figures
- Use `--help` flag for detailed usage of any script
- Results saved in JSON format for easy parsing
