# Media

Demonstration videos, images, and visualizations for the custom SLAM and navigation system.

## Structure
```
media/
├── videos/
│   ├── slam_mapping_demo.mp4
│   ├── hybrid_astar_navigation.mp4
│   ├── loop_closure_detection.mp4
│   └── comparison_nav2_vs_custom.mp4
├── images/
│   ├── slam_rviz_screenshot.png
│   ├── path_comparison.png
│   ├── localization_accuracy.png
│   └── system_architecture.png
└── gifs/
    ├── mapping_timelapse.gif
    ├── autonomous_navigation.gif
    └── path_smoothing.gif
```

## Videos

### slam_mapping_demo.mp4
- Real-time SLAM mapping demonstration
- Shows 5cm resolution occupancy grid generation
- Loop closure detection in action
- Duration: 2-3 minutes

### hybrid_astar_navigation.mp4
- Autonomous navigation using Hybrid A* planner
- Comparison with baseline Nav2 planner
- Shows 25% path efficiency improvement
- Smooth trajectory tracking with MPC

### loop_closure_detection.mp4
- Loop closure detection and pose graph optimization
- Before/after map correction visualization
- Sub-5cm accuracy demonstration

### comparison_nav2_vs_custom.mp4
- Side-by-side comparison
- Nav2 baseline vs Custom Hybrid A* system
- Path length, smoothness, and planning time metrics

## Images

### slam_rviz_screenshot.png
- RViz visualization of SLAM mapping
- Occupancy grid with 5cm resolution
- Robot pose and trajectory
- Already included: Image_1.jpeg from docs

### path_comparison.png
- Overlay of paths from different planners
- Nav2 (red) vs Hybrid A* (green)
- Shows 25% reduction in path length
- Direction changes comparison

### localization_accuracy.png
- Localization error over time
- Sub-5cm accuracy demonstration
- Comparison across different environments

### system_architecture.png
- Complete system block diagram
- SLAM pipeline components
- Navigation stack integration
- Multi-sensor fusion flow

## GIFs

### mapping_timelapse.gif
- Timelapse of mapping session
- Shows progressive map building
- Loop closure corrections
- ~10 second duration

### autonomous_navigation.gif
- Robot navigating autonomously
- Obstacle avoidance
- Goal reaching behavior
- ~5 second loop

### path_smoothing.gif
- Before/after path smoothing
- Gradient descent optimization
- Voronoi diagram attraction

## Creating Media

### Recording Videos

**Using ROS2 Bag:**
```bash
# Record all topics
ros2 bag record -a -o media/recordings/session1

# Record specific topics
ros2 bag record /map /scan /cmd_vel /tf -o media/recordings/nav_demo
```

**Using screen recording:**
```bash
# SimpleScreenRecorder (Linux)
simplescreenrecorder

# Or FFmpeg
ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
  media/videos/slam_demo.mp4
```

### Taking Screenshots

**RViz:**
```bash
# In RViz: File → Save Image
# Or use keyboard shortcut: Ctrl+Shift+S
```

**Command line:**
```bash
# Full screen
import -window root media/images/screenshot.png

# Specific window
import media/images/rviz_screenshot.png
```

### Creating GIFs

**From video:**
```bash
ffmpeg -i media/videos/navigation.mp4 -vf "fps=10,scale=640:-1" \
  media/gifs/navigation.gif
```

**From images:**
```bash
convert -delay 10 -loop 0 media/images/frame*.png media/gifs/timelapse.gif
```

### Optimizing Media

**Compress videos:**
```bash
ffmpeg -i media/videos/original.mp4 -vcodec libx264 -crf 23 \
  media/videos/compressed.mp4
```

**Resize images:**
```bash
convert media/images/large.png -resize 1920x1080 media/images/resized.png
```

**Optimize GIFs:**
```bash
gifsicle -O3 media/gifs/original.gif -o media/gifs/optimized.gif
```

## Visualization Scripts

### Generate Path Comparison
```python
# scripts/visualize_path_comparison.py
import matplotlib.pyplot as plt
import numpy as np

# Load path data
nav2_path = np.load('results/nav2_path.npy')
custom_path = np.load('results/hybrid_astar_path.npy')

# Plot comparison
plt.figure(figsize=(10, 8))
plt.plot(nav2_path[:, 0], nav2_path[:, 1], 'r-', label='Nav2', linewidth=2)
plt.plot(custom_path[:, 0], custom_path[:, 1], 'g-', label='Hybrid A*', linewidth=2)
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('media/images/path_comparison.png', dpi=300)
```

### Generate Localization Accuracy Plot
```python
# scripts/plot_localization_accuracy.py
import matplotlib.pyplot as plt
import numpy as np

# Load error data
errors = np.load('results/localization_errors.npy')

# Plot
plt.figure(figsize=(12, 6))
plt.plot(errors, linewidth=1.5)
plt.axhline(y=0.05, color='r', linestyle='--', label='5cm threshold')
plt.xlabel('Time (s)')
plt.ylabel('Localization Error (m)')
plt.title('Sub-5cm Localization Accuracy')
plt.legend()
plt.grid(True)
plt.savefig('media/images/localization_accuracy.png', dpi=300)
```

## Media Specifications

### Video Format
- **Format:** MP4 (H.264 codec)
- **Resolution:** 1920x1080 (Full HD)
- **Frame rate:** 30 fps
- **Bitrate:** 5-8 Mbps

### Image Format
- **Format:** PNG (lossless) or JPG (compressed)
- **Resolution:** 1920x1080 or higher
- **DPI:** 300 for publication quality

### GIF Format
- **Resolution:** 640x480 or 800x600
- **Frame rate:** 10 fps
- **Optimization:** Use gifsicle or similar

## Usage in Documentation

### Embedding in README
```markdown
# SLAM Mapping Demo
![SLAM Mapping](media/images/slam_rviz_screenshot.png)

# Path Comparison
![Path Comparison](media/images/path_comparison.png)

# Navigation Demo
![Navigation](media/gifs/autonomous_navigation.gif)
```

### Video Links
```markdown
Watch the full demonstration:
[SLAM Mapping Demo](media/videos/slam_mapping_demo.mp4)
[Navigation Comparison](media/videos/comparison_nav2_vs_custom.mp4)
```

## Performance Visualizations

### Key Metrics to Visualize

**Localization:**
- Error over time
- Error distribution histogram
- Trajectory comparison (ground truth vs estimated)

**Path Planning:**
- Path overlay (multiple planners)
- Path length comparison bar chart
- Planning time comparison
- Direction changes histogram

**System Performance:**
- CPU usage over time
- Memory consumption
- Topic frequency (Hz)
- Latency distribution

## Demo Scenarios

### Scenario 1: SLAM Mapping
1. Start in unknown environment
2. Begin mapping with teleoperation
3. Show real-time map building
4. Demonstrate loop closure
5. Save final map

### Scenario 2: Autonomous Navigation
1. Load pre-built map
2. Set navigation goal
3. Show path planning (Hybrid A*)
4. Demonstrate obstacle avoidance
5. Reach goal successfully

### Scenario 3: Comparison
1. Run same task with Nav2 and Custom system
2. Record both side-by-side
3. Compare paths, times, smoothness
4. Highlight 25% improvement

## Notes

- All videos recorded in simulation (Gazebo) and real robot
- Screenshots taken from RViz at key moments
- GIFs created for README and documentation
- High-quality images for publications
- Compress large files before committing to Git
- Use Git LFS for files >10MB

## Copyright

All media files are property of the project and should include appropriate attribution when used in publications or presentations.
