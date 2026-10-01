# Otonom — GPU-Accelerated 3D Elevation Mapping & Autonomous Navigation Stack

A ROS2-based robot navigation system providing real-time 3D elevation mapping on GPU, wheel odometry estimation, motor/arm control, ArUco marker localization, and costmap conversion for Nav2 autonomous navigation.

---

## 🏗️ Architecture Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│ Unitree L2 LiDAR │     │ RealSense Camera │     │  RC Transmitter    │
│   (PointCloud)   │     │ (RGB + Depth)    │     │   (Joystick)       │
└─────────┬────────┘     └────────┬─────────┘     └──────────┬──────────┘
          │                       │                           │
          ▼                       ▼                           ▼
┌─────────────────────────────────────────────┐              │
│         elevation_mapping_cupy (GPU)        │              │
│  ┌──────────┐  ┌──────────┐  ┌────────────┐│              │
│  │ PointMap │→ │ Fusion   │→ │ Traversabl.││              │
│  │ (CuPy)   │  │ & Semant.│  │ (Neural    ││              │
│  └──────────┘  └──────────┘  └────────────┘│              │
└──────────────────────┬──────────────────────┘              │
                       │ GridMap (traversability layer)      │
                       ▼                                     │
           ┌───────────────────────┐                          │
           │   costmap_converter    │←── Encoder Data ────────┘
           │  GridMap → OccupancyGrid│    (motor_kontrol)
           └───────────┬───────────┘
                       │ /map (nav_msgs/OccupancyGrid)
                       ▼
              ┌────────────────┐
              │     Nav2       │ ← Odometry (/odom/wheel)
              │  Navigation    │ ← ArUco pose (if deployed)
              └────────────────┘
```

---

## 📦 Modules

| Directory | Type | Description |
|-----------|------|-------------|
| `elevation_mapping_cupy/` | ROS2 Package | GPU-accelerated real-time 3D elevation mapping core (Python/CuPy) |
| `unitree_lidar_ros2/` | ROS2 Package | C++ driver for the Unitree L2 LiDAR sensor |
| `costmap_converter/` | ROS2 Package | C++ node converting GridMap → OccupancyGrid for Nav2 |
| `motor_kontrol/` | ROS2 Package | Python motor controller / robotic arm / joystick nodes |
| `odometry/` | ROS2 Package | Wheel odometry estimation from encoder + IMU fusion |
| `cv/` | ROS2 Package | Computer vision: RealSense camera publisher & ArUco marker detection |

---

## 🚀 Setup & Installation

### Prerequisites

```bash
# OS: Ubuntu 22.04 (Jammy) with ROS2 Humble
sudo apt update

# Core ROS2 packages
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge ros-humble-rviz2 \
  ros-humble-tf2-ros ros-humble-tf-transformations ros-humble-message-filters \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
  ros-hulse-grid-map ros-humble-elevation-map-msgs

# Python dependencies
pip3 install cupy-cuda12x numpy<2 scipy opencv-python simple-parsing shapely ruamel.yaml transforms3d pyrealsense2

# C++ build tools
sudo apt install -y cmake build-essential libpcl-dev libboost-system-dev
```

### Build

```bash
# Create ROS2 workspace
mkdir -p ~/otonom_ws/src && cd ~/otonom_ws/src

# Clone the repository (already here, symlink if needed)
git clone <your-repo-url> otonom  # or cp/symlink existing repo

cd ~/otonom_ws
source /opt/ros/humble/setup.bash

# Build each package
colcon build --packages-select elevation_mapping_cupy
colcon build --packages-select unitree_lidar_ros2
colcon build --packages-select costmap_converter
colcon build --packages-select motor_kontrol
colcon build --packages-select odometry
colcon build --packages-select cv

# Source workspace
source install/setup.bash

### Python Dependencies
```bash
# Install package-level requirements (see requirements.txt for details)
pip3 install -r otonom/requirements.txt
```

> **⚠️ Note on `cupy-cuda12x`**: Listed in `requirements.txt` as-is, but match your actual CUDA version:
> - **CUDA 12.x** → use `cupy-cuda12x` (default, already listed)
> - **CUDA 11.x** → install `cupy-cuda11x` instead
> - **CUDA 11.8** → install `cupy-cuda118`
>
> Verify with `nvcc --version` and replace the package name accordingly.
>
> > **⚠️ Note on `numpy<2.0`**: Pinned because CuPy interop breaks on numpy ≥ 2. This was already enforced in your ROS2 package.xml as `numpy_lessthan_2`. Use this pip requirement file instead of system `numpy`.
```

---

## ▶️ Running the System

### Option A — Full Simulation (Gazebo + TurtleBot3)

```bash
# Terminal 1: Launch Gazebo world with turtlebot3
export TURTLEBOT3_MODEL=waffle_relescope_depth
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch elevation mapping + RViz
ros2 launch elevation_mapping_cupy elevation_mapping_turtle.launch.py use_sim_time:=true
```

### Option B — Real Robot Deployment

**Terminal 1 — LiDAR driver:**
```bash
source ~/otonom_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Terminal 2 — Motor control + joystick:**
```bash
ros2 run motor_kontrol motor          # Motors & encoder reading (60 Hz)
ros2 run motor_kontrol robot_kol      # Robotic arm control (mode=2)
ros2 run motor_kontrol uzaktan_kumanda  # RC receiver → /joystick_cmd
```

**Terminal 3 — Odometry:**
```bash
ros2 run odometry odom
```

**Terminal 4 — Camera & ArUco localization:**
```bash
# (Optional) ArUco camera node for marker-based localization:
ros2 run cv cam_node
```

**Terminal 5 — Costmap converter & RViz:**
```bash
# Start the costmap converter (feeds Nav2):
ros2 run costmap_converter grid_map_to_costmap

# Start RViz for visualization:
rviz2 -d ~/otonom_ws/src/otonom/costmap_converter/rviz/view.rviz
```

---

## 🔧 Elevation Mapping Parameters — Detailed Guide

Core parameters are in `elevation_mapping_cupy/config/core/core_param.yaml`. They are grouped into functional categories below.

### 1. Map Geometry & Resolution

| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | `0.5` m | Cell edge length (m). Lower = higher detail but more GPU memory. Your robot uses **0.5 m**. This gives a `(map_length/resolution)²` grid — e.g., 20×20 for a 10 m map. |
| `map_length` | `10.0` m | Physical extent of the map in each direction (X and Y) from center. A 10 m map with 0.5 m resolution = **20×20 cells**. Larger maps need more GPU memory (~3× per axis). |

> **Memory formula (GPU):** `layers × rows × cols × data_size`
> With 7 layers + float32: `7 × 20 × 20 × 4 B ≈ 112 KB` — negligible. For a 50 m map: `7 × 100 × 100 × 4 → ~28 MB`.

### 2. Sensor Noise & Point Filtering

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sensor_noise_factor` | `0.05` | Weight inverse factor: noise ∝ `sensor_noise_factor × distance²`. Higher values = points receive lower confidence at range → noisier map but fewer false positives. For LiDAR use **smaller** (e.g., 0.01). For depth cameras, use similar to default. |
| `mahalanobis_thresh` | `2.0` | Maximum Mahalanobis distance for a point to be considered valid. Points outside this threshold of any existing cell are marked as outliers. Lower = stricter (fewer points accepted). Higher = more lenient but riskier. |
| `outlier_variance` | `0.01` | Variance value assigned to outlier cells. Low values effectively "hide" those cells. |
| `min_valid_distance` | `0.2` m | Reject points closer than this to the sensor (eliminates robot-body self-reflections). For LiDAR on a rover, set ~**0.3–0.5 m**. |
| `max_height_range` | `10.5` m | Maximum height above sensor before point rejection (prevents ceiling/floor detection for overhead mapping). |

### 3. Ramped Height Rejection

These three parameters define a dynamic vertical filter: reject points where  
**z > max(d − ramped_height_range_b, 0) × ramped_height_range_a + ramped_height_range_c**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ramped_height_range_a` | `0.3` | Slope of rejection cone from sensor. Lower = narrower valid volume, more aggressive near-field filtering. |
| `ramped_height_range_b` | `1.0` m | Distance offset. Controls how far in front the ramp starts sloping. |
| `ramped_height_range_c` | `0.2` m | Minimum vertical height from sensor regardless of distance. Prevents near-field "wall" at base. |

### 4. Drift Compensation (Critical for Nav2)

Cumulative odometry drift corrupts map position over time. These parameters compensate:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_drift_compensation` | `true` | Toggles drift correction on/off. **Always keep on** for mobile robots. |
| `position_noise_thresh` | `0.01` m | Drift compensation triggers only when odometry position change exceeds this. Prevents noise from being treated as drift. |
| `orientation_noise_thresh` | `0.1` rad | Same but for orientation change. Keep low to avoid over-compensating during rapid turning. |
| `drift_compensation_alpha` | `0.1` | Smoothing factor: smaller values = slower, smoother correction (less jarring map shifts). Larger = faster correction but potential jumps. |
| `max_drift` | `0.1` m | Maximum allowable drift per update. Drift beyond this is **discarded for safety** — prevents catastrophic map corruption from bad sensor readings. |
| `drift_compensation_variance_inlier` | `0.05` | Only cells with variance below this value are used as reference during compensation. Lower = more selective (fewer trusted cells). |
| `traversability_inlier` | `0.9` | Minimum traversability score for a cell to be considered an inlier. Higher = only confident traversal areas contribute. |
| `min_height_drift_cnt` | `100` | Minimum number of inlier cells required before drift compensation is triggered. Prevents spurious corrections from very few observations. |

**How it works in practice:**
1. The map checks if odometry has shifted beyond `position_noise_thresh` or `orientation_noise_thresh`.
2. It finds "inlier" cells (low variance + high traversability).
3. Computes the mean error between projected and observed elevations.
4. If total drift < `max_drift`, **adds** a fraction of the correction (`alpha × mean_error`) to the elevation layer.

### 5. Variance & Time Management

| Parameter | Default | Description |
|-----------|---------|-------------|
| `initial_variance` | `1000.0` | Starting confidence (variance) for new map cells. Very high values = cells start very uncertain, allowing fast updates from incoming points. Higher = faster initial convergence but noisier early map. |
| `initialized_variance` | `10.0` | Variance after the map is initialized (first initialization with point cloud). Lower = more confident initial model. |
| `max_variance` | `100.0` | Maximum allowed variance per cell. Acts as a "forgetting cap" — prevents any cell from becoming too uncertain. |
| `time_variance` | `0.0001` | Variance added to valid cells each `update_variance_fps`. Represents temporal uncertainty growth. Lower = older observations stay more confident (better for slow-moving robots). |
| `time_interval` | `0.1` s | Interval at which the time layer increments by this amount. Tracks how "stale" each cell is. |

### 6. Map Update Rates

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_pose_fps` | `60.0` Hz | Rate at which pose updates and map shifting occur. Should match or exceed your odometry/tf broadcast rate (20+ Hz recommended). |
| `update_variance_fps` | `2.0` Hz | Rate for applying temporal variance growth. Not time-critical; lower rates save GPU compute. |
| `map_acquire_fps` | `5.0` Hz | Rate at which map data is copied from GPU → CPU for publishing. Should be **lower** than all update rates. Higher = smoother visualization but more CPU/GPU bandwidth. |

### 7. Traversability & Obstacle Detection

Used by the neural network traversability filter and Nav2 costmap conversion:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dilation_size` | `3` | Kernel size (pixels) for dilating valid cells before traversability filtering. Larger = more context per cell, less "noise" in the output. |
| `safe_thresh` | `0.7` | If a cell's traversability drops below this threshold, it counts as **unsafe** for polygon traversal checks. |
| `safe_min_thresh` | `0.4` | Hard minimum: any single cell below this score makes the entire polygon unsafe (regardless of average). |
| `max_unsafe_n` | `10` | If more than this many cells in a polygon are unsafe, the polygon is rejected. Tolerance for small obstacles within an otherwise traversable area. |

### 8. Wall Sharpening & Cleanup

For producing clean edge detection (vertical walls vs. ground):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_edge_sharpen` | `true` | Enables wall_num_thresh logic: when a cell has more points than the threshold, only points **above** current height are used — making vertical surfaces crisper. |
| `wall_num_thresh` | `20` | Minimum point count in a cell before edge sharpening is activated per that cell. Lower = sharper walls with fewer observations (noisier). Higher = wait for more data. |
| `enable_visibility_cleanup` | `true` | Removes invalid cells behind obstacles using ray tracing from sensor to valid cells. Prevents "ghost" terrain behind walls. |
| `cleanup_step` | `0.1` | Amount subtracted from validity layer during cleanup per iteration. Higher = faster cleanup but may eat into real data. |
| `cleanup_cos_thresh` | `0.1` | Cosine threshold between ray direction and vertical axis for visibility checks. Lower = stricter angle requirement (fewer points cleaned). |

### 9. Overlap Clearance (Multi-Floor)

For robots that move between floors/levels:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_overlap_clearance` | `true` | Clears cells near the robot center that don't match expected height range (prevents old floor layers from corrupting new ones). |
| `overlap_clear_range_xy` | `4.0` m | XY radius around map center where clearance is active. Cells outside this ring are never cleared in one pass. Larger = safer but slightly slower. |
| `overlap_clear_range_z` | `2.0` m | Vertical range from current center-Z. Only cells within ±Z of map center can be cleared. Prevents accidental deletion of far terrain. |

### 10. Topology & Frame Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_frame` | `'map'` | Global reference frame the map sits in. Nav2 expects **`/map`**. |
| `base_frame` | `'base_link'` | Robot base — the map center follows this frame in the odometry chain. Must match your robot description TF tree. |
| `corrected_map_frame` | `'map'` | Frame where drift-compensated map TF is published. Usually same as `map_frame`. |

### 11. Feature Toggles (Performance Settings)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_normal_arrow_publishing` | `false` | Publish normals as RViz visualization markers. Turn on for debugging surface orientation. Off by default — saves compute/bandwidth. |
| `enable_drift_corrected_TF_publishing` | `true` | Publish TF `/map → /robot_base_drift_corrected`. Useful when Nav2 uses the corrected frame for localization. |
| `enable_normal_color` | `false` | Include surface normals as a "color" layer in the map output. Enable if you want 3D visualization with shading. |

### 12. Map Initialization

Controls how the map is seeded on first point cloud receipt:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `initialize_method` | `'linear'` | Interpolation: `'nearest'`, `'linear'`, or `'cubic'`. Cubic = smoothest but slowest. Linear = good balance for rovers. |
| `initialize_frame_id` | `['base_link']` | TF frame at which to initialize. Each value creates a square map section centered on that frame's current pose. Multiple frames allow multi-region initialization (e.g., floor + table top). |
| `dilation_size_initialize` | `2` | Dilation kernel size applied after initialization (smoothes rough initial data). Higher = smoother but may erase small features. |
| `use_initializer_at_start` | `true` | Initialize map immediately on first point cloud, or wait for `/initialize_map` service call. Set to **false** if you want to start empty and seed manually via RViz "2D Nav Goal" → "Publish Initial Pose". |

---

## 🗺️ Plugin Layers (Map Post-Processing)

Defined in `config/core/plugin_config.yaml`. These run sequentially on the raw map before publishing:

```
Raw Map ──→ min_filter ──→ smooth ──→ inpaint ──→ (traversability erosion) ──→ Published Layers
```

| Plugin | Function | Key Parameters |
|--------|----------|----------------|
| `min_filter` | Fills invalid cells with minimum neighboring height | `dilation_size`, `iteration_n` |
| `smooth_filter` | Gaussian-like smoothing of elevation layer | `input_layer_name: min_filter` (processes previous output) |
| `inpainting` | OpenCV inpainting (telea/Navier-Stokes) for gaps | `method: telea \| ns` |
| `erosion` | Dilates/dilates traversability to clean edges | `input_layer_name`, `dilation_size`, `iteration_n` |

---

## 📡 Topics & Services Summary

### Key Topics

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/elevation_mapping_node/elevation_map_raw` | Out (GridMap) | `grid_map_msgs/` | Raw unfiltered map (publisher: elevation_map_raw) |
| `/map` | Out (OccupancyGrid) | `nav_msgs/OccupancyGrid` | Nav2-compatible costmap (from costmap_converter, 30% threshold) |
| `/odom/wheel` | Out (Odometry) | `nav_msgs/Odometry` | Wheel odometry (odometry package) |
| `/encoder_data` | Out | `rover_msgs/EncoderMsg` | Raw wheel speeds → encoder node |
| `/joystick_cmd` | In | `rover_msgs/ControllerMsg` | Joystick command → motor controller |
| `/marker_pose` | Out (PoseStamped) | `geometry_msgs/PoseStamped` | ArUco marker pose in map frame |

---

## 🔍 Tuning Tips

1. **For noisy sensors** → Increase `sensor_noise_factor`, raise `mahalanobis_thresh`
2. **For fast-moving robots** → Keep `drift_compensation_alpha` small (e.g., 0.05) to avoid jumpy corrections
3. **For low-light indoor environments** → Decrease `ramped_height_range_a` to tighten the valid cone below and above the sensor
4. **If Nav2 costmap looks noisy** → Adjust your `costmap_converter` threshold (`val < 0.3f = obstacle`) or increase `dilation_size` in plugins
5. **Memory concerns** → Larger `map_length` with small `resolution` grows quadratically. A 40 m map at 0.1 m resolution = ~400×400 × 7 × 4 bytes ≈ **45 MB on GPU**
