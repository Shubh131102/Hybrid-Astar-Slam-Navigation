# Maps Directory

Saved maps from SLAM sessions for navigation and localization.

## Structure
```
maps/
├── map.yaml                 Default map metadata
├── map.pgm                  Default map image
├── test_environment/        Test environment maps
│   ├── map.yaml
│   └── map.pgm
└── lab_environment/         Lab environment maps
    ├── map.yaml
    └── map.pgm
```

## Map File Format

### map.yaml (Metadata)
```yaml
image: map.pgm
resolution: 0.05              # meters per pixel
origin: [-10.0, -10.0, 0.0]  # [x, y, yaw] in meters
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### map.pgm (Image)
- Grayscale image representing occupancy grid
- White (255): Free space
- Black (0): Occupied space
- Gray (127): Unknown space

## Creating Maps

### Method 1: SLAM Mapping
```bash
# Terminal 1: Launch SLAM
ros2 launch custom_nav slam_sim.launch.py mode:=mapping

# Terminal 2: Teleoperate to explore environment
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 3: Save map when mapping is complete
ros2 run nav2_map_server map_saver_cli -f maps/my_map

# This creates:
# - maps/my_map.yaml
# - maps/my_map.pgm
```

### Method 2: Load Existing Map
```bash
# Copy map files to maps directory
cp /path/to/existing/map.yaml maps/
cp /path/to/existing/map.pgm maps/
```

## Using Maps for Navigation

### Load Map with Nav2
```bash
ros2 launch custom_nav nav2.launch.py map:=maps/my_map.yaml
```

### Load Map Manually
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=maps/my_map.yaml
```

## Map Quality Guidelines

### Good Map Characteristics
- **Complete coverage:** All navigable areas mapped
- **Clear boundaries:** Well-defined walls and obstacles
- **Minimal noise:** Few isolated pixels or artifacts
- **Proper scale:** Resolution matches navigation needs

### Recommended Resolutions
- **High precision:** 0.025m (2.5cm) - For tight spaces
- **Standard:** 0.05m (5cm) - Balance of accuracy and size
- **Large areas:** 0.10m (10cm) - For efficiency

### Map Size Considerations
| Environment Size | Resolution | Map File Size |
|------------------|------------|---------------|
| 10m x 10m | 0.05m | ~40 KB |
| 20m x 20m | 0.05m | ~160 KB |
| 50m x 50m | 0.05m | ~1 MB |

## Map Editing

### Using GIMP
```bash
# Open map image
gimp maps/map.pgm

# Edit obstacles/free space
# - Use white (255) for free space
# - Use black (0) for obstacles
# - Use gray (127) for unknown

# Save as PGM
```

### Using ImageMagick
```bash
# Convert format
convert maps/map.pgm maps/map.png

# Resize map
convert maps/map.pgm -resize 50% maps/map_small.pgm

# Clean noise
convert maps/map.pgm -median 1 maps/map_clean.pgm
```

## Map Metadata Parameters

### resolution
- Meters per pixel
- Smaller = higher resolution
- **Example:** 0.05 means each pixel = 5cm

### origin
- Map origin in world coordinates [x, y, yaw]
- Bottom-left corner of the map
- **Example:** [-10.0, -10.0, 0.0] means map starts at (-10, -10)

### negate
- 0: Black = occupied, White = free
- 1: Inverted colors

### occupied_thresh
- Probability threshold for occupied (0.0 - 1.0)
- Pixels above this are considered occupied
- **Recommended:** 0.65

### free_thresh
- Probability threshold for free (0.0 - 1.0)
- Pixels below this are considered free
- **Recommended:** 0.25

## Map Validation

### Check Map Quality
```bash
# Visualize in RViz
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=maps/my_map.yaml

rviz2

# Add Map display:
# Topic: /map
```

### Verify Map Metadata
```bash
# Check file exists
ls -lh maps/my_map.*

# Check YAML syntax
cat maps/my_map.yaml

# Check image dimensions
identify maps/my_map.pgm
```

## Common Issues

### Map Not Loading

**Check file paths in YAML:**
```yaml
image: map.pgm  # Relative path
# OR
image: /absolute/path/to/map.pgm  # Absolute path
```

**Verify files exist:**
```bash
ls maps/my_map.yaml
ls maps/my_map.pgm
```

### Map Alignment Issues

**Check origin:**
- Origin should place map correctly in world frame
- Adjust origin [x, y, yaw] values

**Verify resolution:**
- Must match SLAM resolution used during mapping

### Navigation Failures

**Map quality issues:**
- Incomplete mapping (gaps in walls)
- Too much noise
- Incorrect occupied/free thresholds

**Fix:**
- Remap environment with better coverage
- Edit map to clean noise
- Adjust occupied_thresh and free_thresh

## Example Maps

### Small Test Environment (10m x 10m)
```yaml
# maps/test_environment/map.yaml
image: map.pgm
resolution: 0.05
origin: [-5.0, -5.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**Usage:**
```bash
ros2 launch custom_nav nav2.launch.py \
  map:=maps/test_environment/map.yaml
```

### Large Lab Environment (50m x 50m)
```yaml
# maps/lab_environment/map.yaml
image: map.pgm
resolution: 0.05
origin: [-25.0, -25.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**Usage:**
```bash
ros2 launch custom_nav nav2.launch.py \
  map:=maps/lab_environment/map.yaml
```

## Best Practices

### During Mapping
1. Drive slowly for better scan quality
2. Cover all navigable areas
3. Close loops for better accuracy
4. Avoid dynamic obstacles (people, chairs)
5. Maintain consistent lighting

### Map Saving
1. Save multiple versions during long sessions
2. Use descriptive names (e.g., `lab_floor2_2024.yaml`)
3. Include date and environment info
4. Keep backup copies

### Map Usage
1. Always verify map loads correctly before navigation
2. Test localization in known locations
3. Update maps when environment changes
4. Use appropriate resolution for task

## Map Conversion

### PGM to PNG
```bash
convert maps/map.pgm maps/map.png
```

### PNG to PGM
```bash
convert maps/map.png -type grayscale maps/map.pgm
```

### Rotate Map
```bash
convert maps/map.pgm -rotate 90 maps/map_rotated.pgm

# Update origin in YAML accordingly
```

### Crop Map
```bash
convert maps/map.pgm -crop 500x500+100+100 maps/map_cropped.pgm

# Update origin and size in YAML
```

## Notes

- Maps created with 5cm resolution for sub-5cm localization accuracy
- PGM format is standard for ROS2 maps
- Always keep .yaml and .pgm files together
- Map origin affects initial pose and navigation
- Use SLAM in localization mode with pre-built maps
