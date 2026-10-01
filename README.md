# Otonom — GPU-Accelerated 3D Elevation Mapping & Autonomous Navigation Stack

A ROS2-based robot navigation system providing real-time 3D elevation mapping on GPU, wheel odometry estimation, motor/arm control, costmap conversion (`GridMap → OccupancyGrid`), and full [Nav2](https://navigation.ros.org/) autonomous navigation stack for outdoor/indoor rover platforms equipped with a Unitree L2 LiDAR.

---

## 🏗️ High-Level Architecture

```
┌───────────────┐                        ┌──────────────────┐
│  Unitree L2   │                        │ RC Transmitter  │
│   (LiDAR)     │                        │  (Joystick)     │
│ PointCloud     │                        │ ControllerMsg   │
└───────┬───────┘                        └────────┬────────┘
        │                                         │
        ▼                                         ▼
┌─────────────────────────────────────────────────────────────┐
│                   elevation_mapping_cupy                    │
│  GPU-accelerated sensor fusion, traversability,               │
│  semantic layers. Input: PointCloud → Output: GridMap         │
└──────────────────────────────┬──────────────────────────────┘
                               │ /elevation_map_raw (GridMap)
                               ▼
                     costmap_converter
           GridMap.traversability → OccupancyGrid (/map)
                               │
            ┌──────────────────┼──────────────────┐
            ▼                  ▼                   ▼
    ┌─────────────┐  ┌─────────────┐   ┌──────────────────┐
    │ Nav2        │  │ Odometry    │   │ Robot Motors     │
    │ (AMCL,      │←│ (/odom/wheel)│   │ (motor_kontrol)  │
    │ Path/Foli-   │  │             │   │ serial → STM32   │
    │ ng stack)   │  └─────────────┘   └──────────────────┘
    └─────────────┘
```

### Data Flow Summary

1. **Unitree L2** publishes a point cloud (`/point_cloud`) at the LiDAR data rate (~10 Hz).
2. **`elevation_mapping_cupy`** ingests the point cloud, updates an on-GPU elevation map with traversability and semantic layers, and publishes a `GridMap` (`/elevation_map_raw`).
3. **`costmap_converter`** subscribes to the GridMap and publishes a `nav_msgs/OccupancyGrid` (`/map`) that feeds Nav2's navigation stack.
4. **Nav2** computes global/local plans and velocity commands, while odometry (built from wheel encoders) closes the pose loop.

---

## 📦 Package List

| Directory | Type | Description |
|-----------|------|-------------|
| `elevation_mapping_cupy/` | ROS2 Package | GPU-accelerated elevation mapping core (Python/CuPy) |
| `unitree_lidar_ros2/` | ROS2 Package | C++ driver for the Unitree L2 LiDAR sensor |
| `costmap_converter/` | ROS2 Package | Converts GridMap→OccupancyGrid for Nav2 (`val < 0.3f → obstacle`) |
| `motor_kontrol/` | ROS2 Package | Motor controller, robotic arm handler, and RC joystick receiver |
| `odometry/` | ROS2 Package | Wheel odometry from encoder data + IMU orientation quaternions |

---

## 🚀 Building & Running

### 0. Prerequisites

```bash
# Ubuntu 22.04 + ROS2 Humble desktop:
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-grid-map-ros \
  ros-humble-grid-map-msgs ros-humble-elevation-map-msgs \
  ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf-transformations \
  ros-humble-message-filters ros-humble-geometry-msgs \
  ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-control-tools \
  ros-humble-image-transport ros-humble-camera-info-manager \
  cmake build-essential libpcl-dev libboost-system-dev

# CUDA toolkit (≥12.0 for cupy-cuda12x) — verify with:
nvcc --version
```

### 1. Workspace Build

```bash
# Clone the repository (already here, symlink if needed)
git clone https://github.com/HU-Rover/otonom.git # or cp/symlink existing repo

cd ~/otonom

# Build each package
colcon build 

# Source workspace
source install/setup.bash
```

### 2. Python Dependencies

See [`requirements.txt`](./requirements.txt) for a pip-compatible list.

```bash
pip3 install -r /home/omer/github/otonom/requirements.txt
```

> **⚠️ CUDA version:** Match `cupy-cudaNNx` to your installed toolkit (use `nvcc --version`).
>
> **⚠️ `numpy<2.0`:** Pinned because CuPy interop breaks at ≥ 2 — already declared in the ROS package.xml as `numpy_lessthan_2`.

### 3. Running on a Real Rover

```bash
# Terminal 1 — LiDAR
source ~/otonom_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# Terminal 2 — Motor controller + encoder reader (60 Hz)
ros2 run motor_kontrol motor

# Terminal 3 — Wheel odometry → /odom/wheel
ros2 run odometry odom

# Terminal 4 — Costmap converter → /map
ros2 run costmap_converter grid_map_to_costmap

# Terminal 5 — Nav2
ros2 launch nav2_bringup navigation.launch.py
```

> To drive manually during testing, `ros2 run motor_kontrol uzaktan_kumanda` listens to the RC receiver (`/dev/ttyUSB0`) and publishes joystick commands to the `motor` node. For robotic-arm handling, `ros2 run motor_kontrol robot_kol` forwards mode-2 servo/gripper commands via serial.

> To visualize, launch RViz with a layout that adds `GridMap`, `OccupancyGrid`, `Odometry`, and Nav2 RViz plugins (e.g., `rviz2 -d /home/omer/github/otonom/costmap_converter/rviz/view.rviz`).

---

## 🔧 Elevation Mapping — In-Depth Parameter Reference

Core parameters live in `config/core/core_param.yaml`. Each section below explains the purpose, a recommended tuning range, and what to watch for when you change it.

### 1. Map Geometry

| Parameter | Default | Notes |
|-----------|---------|------|
| `resolution` | `0.5` m | Cell edge length in metres. Grid size = `(map_length / resolution)²`. Your rover uses **0.5**, giving a 20×20 grid for a 10 m map. Lower → more detail but **quadratic** memory/compute increase. |
| `map_length` | `10.0` m | Full width/height of the map in each direction from centre. Smaller maps save GPU memory at the cost of frequent "edge-of-map" clipping during fast travel. |

> **GPU memory rough formula (float32):** `layers × rows × cols × 4 bytes`. With `7` base layers at 0.5 m / 10 m: `7 × 20 × 20 × 4 ≈ 112 KB` — tiny. The real cost is in temporary CUDA buffers for kernel launches (~tens of MB).

### 2. Sensor Noise & Point Filtering

| Parameter | Default | Tuning Notes |
|-----------|---------|-------------|
| `sensor_noise_factor` | `0.05` | Weight inversely ∝ sensor_noise × distance². **Higher → points accepted with lower confidence at long range.** For the Unitree L2 (solid-state LiDAR), drop to ~**0.01–0.02**. For RGB-D cameras keep closer to the default. |
| `mahalanobis_thresh` | `2.0` | Max Mahalanobis distance from an existing cell's Gaussian before a point is rejected as an outlier. **Lower → stricter map, fewer points accepted.** Raise if you get too-few valid cells on rough terrain; lower for sharper walls and floors. |
| `outlier_variance` | `0.01` | Variance pushed into a cell when the incoming point is flagged outlier. Low = effectively invisible. High = leaves "ghost" data behind. |
| `min_valid_distance` | `0.2` m | Clamp near-range noise (robot body, chassis reflections). **Increase to 0.3–0.5** for rovers with dense LiDAR mounting on sloped bumpers. |
| `max_height_range` | `10.5` m | Upper-bound height clip relative to sensor Z. Prevents ceilings from appearing when looking up. Only relevant if the mount is very close to the ground. |

### 3. Ramped Height Rejection (Cone Filter)

These three values define a conical rejection zone around the sensor:
> Reject points where `z > max(d - ramped_height_range_b, 0) × ramped_height_range_a + ramped_height_range_c`

| Parameter | Default | Intuition |
|-----------|---------|-----------|
| `ramped_height_range_a` | `0.3` (dimensionless) | Slope of the upper rejection boundary — smaller = narrower cone (more aggressive near-field filtering). |
| `ramped_height_range_b` | `1.0` m | Offset along distance. Controls how far in front of the sensor the ramp "starts". |
| `ramped_height_range_c` | `0.2` m | Minimum absolute height from sensor regardless of range (clips floor directly underneath). |

> **Tune tip:** On a rover looking slightly downward, increase `ramped_height_range_b` so the rejection cone starts closer to the robot and avoids falsely marking ground under the chassis as "too high".

### 4. Drift Compensation ⚡

Cumulative odometry drift corrupts map pose over time. This subsystem *proactively* corrects the elevation layer when a large enough motion is detected:

| Parameter | Default | Effect of Changing |
|-----------|---------|--------------------|
| `enable_drift_compensation` | `true` | **Always keep true** on a moving rover. |
| `position_noise_thresh` | `0.01` m | Minimum odometry displacement before compensation is *eligible*. Higher → more selective (won't trigger for tiny jitters). Lower → catches every micro-adjustment, but also more noise-driven corrections. |
| `orientation_noise_thresh` | `0.1` rad (≈ 6°) | Same as above but for angular movement. Keep low to avoid over-correcting during tight turns where LiAD returns are unreliable. |
| `drift_compensation_alpha` | `0.1` | Smoothing factor: `elevation_new = elevation + alpha × mean_error`. **Smaller → slower, gentler correction** (recommended 0.05–0.1). Larger → faster convergence but visible "hops" in the map. |
| `max_drift` | `0.1` m | Safety cap — drift *above* this is discarded entirely. Prevents a single bad measurement from corrupting the whole map. |
| `drift_compensation_variance_inlier` | `0.05` | Only cells with variance ≤ this are trusted as "fixed ground" for the error estimate. Higher → more cells included (more robust). Lower → only highest-confidence cells (less coverage, tighter correction). |
| `traversability_inlier` | `0.9` | Minimum traversability score for an inlier cell. Higher = only truly flat areas contribute to drift estimate. Set lower (~0.5) if you need more reference cells on uneven terrain. |
| `min_height_drift_cnt` | `100` | Minimum number of valid inliers before any correction occurs. Prevents single-cell corrections from "jumping" the map. Raise on sparse maps; reduce for very rich scans. |

**Mechanism (what happens each pose update):**
1. Compare the new odometry pose with the old one. If displacement < `position_noise_thresh` *and* rotation < `orientation_noise_thresh`, **skip compensation entirely**.
2. Collect all "inlier" cells (variance ≤ `drift_compensation_variance_inlier` ∩ traversability ≥ `traversability_inlier`).
3. Compute the mean elevation error between projected LiDAR measurements and existing cells → this is the estimated drift along the sensor's vertical axis.
4. If total drift magnitude < `max_drift`, apply: `elevation += alpha × mean_error`. Cells outside the radius of validity are left alone to avoid "smearing" corrections across large unknown regions.

### 5. Variance & Memory Management

| Parameter | Default | Notes |
|-----------|---------|-------|
| `initial_variance` | `1000.0` | Starting variance for brand-new cells. **Very high** values mean the cell is "open to persuasion" from every incoming point, which gives fast convergence but early-stage noise. Lower (e.g., 10) = more confident initial guess that resists later data changes. |
| `initialized_variance` | `10.0` | Variance *after* map initialization runs (first point-cloud pass). Set lower if you want the initialized model to be authoritative over subsequent noisy data. |
| `max_variance` | `100.0` | Hard cap per cell. Acts as a "forgetting ceiling" — older unobserved cells still gain variance via `time_variance` but never exceed this value. Higher = more conservative map (less update speed). Lower = cells age out faster, potentially losing valid data under the robot. |
| `time_interval` | `0.1` s | Interval at which each cell's time-layer is incremented. Tracks "staleness". Used downstream to age out stale observations via `time_variance`. |
| `update_variance_fps` | `2.0` Hz | How often temporal variance (`time_variance`) is applied. Non-critical — lower saves GPU cycles; higher keeps aging more accurate for fast-moving maps. |

### 6. Map Update Rates (Scheduling)

| Parameter | Default | Recommended Range |
|-----------|---------|-------------------|
| `update_pose_fps` | `60.0` Hz | Match or **exceed** your odometry/tf broadcast rate (20+ is fine; 60 is overkill but safe). Determines how often the map tracks new pose estimates and shifts centre. Keep high if odometry jitter is present — more frequent re-centring = less clipping at map edges. |
| `update_variance_fps` | `2.0` Hz | Low enough to be cheap (non-critical compute), high enough that aging doesn't overshoot between ticks. 1–5 Hz works for most rovers. |
| `map_acquire_fps` | `5.0` Hz | Rate of GPU → CPU map copy for publishing `/elevation_map_raw`. Must be **strictly lower** than all update rates. For real-time RViz, 3–8 Hz is fine. Higher = smoother graphics but more bandwidth overhead. |

### 7. Traversability Thresholds (Nav2 Integration)

Used by the traversability filter neural network and downstream Nav2 costmap thresholds:

| Parameter | Default | Impact |
|-----------|---------|--------|
| `dilation_size` | `3` | Dilation kernel size before the NN filter runs. Larger = more context per cell, fewer "salt-and-pepper" false obstacles. On a 0.5 m map with 20×20 resolution, each pixel spans large terrain patches — 3 is adequate; go up to 5 only for very smooth floors. |
| `safe_thresh` | `0.7` | Traversability below this counts as **unsafe** during polygon traversability checks. Affects Nav2's obstacle costmap (`costmap_converter` threshold of 0.3 maps directly here — anything with a traversability score < 0.3 is treated as an occupied cell at value 100). |
| `safe_min_thresh` | `0.4` | **Absolute floor** — any single cell below this kills the entire polygon regardless of its average traversability. Tighten to 0.5 if you want Nav2 to avoid even marginal terrain. |
| `max_unsafe_n` | `10` | Tolerance for small obstacles within an otherwise-safe polygon. If more than 10 *unsafe cells* fall inside a proposed route segment, reject the whole thing. Raise (e.g., 20) on rough terrain where Nav2 can plausibly cross; lower (e.g., 5) if you want conservative routing. |

### 8. Wall Sharpen & Visibility Cleanup

| Parameter | Default | Effect |
|-----------|---------|--------|
| `enable_edge_sharpen` | `true` | When ≥ `wall_num_thresh` points fall into a cell, only points **above** the current height are kept → makes walls and steps crisp. Keep true for rover terrain with curbs/boxes/etc. Flip off on very flat warehouse floors. |
| `wall_num_thresh` | `20` | Minimum points-per-cell to activate sharpening per cell. Lower = crisper edges with fewer observations but also more noise (each stray point can flip a cell). Raise on noisy sensors (e.g., 40–50 for high-density LiDAR). |
| `enable_visibility_cleanup` | `true` | Removes "behind-obstacle" ghosts using ray-tracing from sensor through valid cells into empty space. Keeps the map honest under shelves/underpasses. Disable if you experience excessive data loss behind dense structures. |
| `cleanup_step` | `0.1` | Validity-subtraction per cleanup iteration. Higher = faster erosion of ghost cells (fewer iterations needed) but more aggressive — risk of deleting real terrain near obstacles. 0.05–0.1 is safe for ground mapping. |
| `cleanup_cos_thresh` | `0.1` | Cosine threshold on ray direction vs. vertical axis; controls the cone of visibility. Lower = tighter cone (more points rejected as "looking off-angle"). Keep default unless you have a wide-cone sensor like a camera or RGB-D unit. |

### 9. Overlap Clearance (Multi-Floor)

For robots that encounter elevated surfaces (shelves, ramps):

| Parameter | Default | Notes |
|-----------|---------|-------|
| `enable_overlap_clearance` | `true` | Clears cells near map centre that exceed ±`overlap_clear_range_z` from expected floor height, preventing lower-floor ghosts above the robot. On single-floor operation this is mostly harmless; keep true. |
| `overlap_clear_range_xy` | `4.0` m | XY ring radius where clearance can fire. Cells outside are never cleared in any pass — prevents the ring-edge effect (half a map wiped during clearance). Larger = safer but slightly slower per frame. 4–6 m works for typical rovers. |
| `overlap_clear_range_z` | `2.0` m | Z-range from centre used to identify "suspicious" cells. Raise to ~3-4 m if you have tall objects (pallet racks, etc.) that span the full clearance window and get erroneously wiped. |

### 10. Frame / TF Configuration

| Parameter | Default | Notes |
|-----------|---------|-------|
| `map_frame` | `'map'` | Nav2's global frame — **must** be `/map`. Changing it requires updating Nav2 `global_frame` parameter and all costmap layers. |
| `base_frame` | `'base_link'` | Frame the map center orbits (the robot body). Must exist in your TF tree and match your URDF/SRDF base link name. |
| `corrected_map_frame` | `'map'` | Drift-corrected transform published as `/corrected_map_frame`. For most rover setups this is identical to `map_frame`; only differ if you use a separate drift-compensated localization system (e.g., LiDAR odometry + map matching). |

### 11. Feature Toggles

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `enable_normal_arrow_publishing` | `false` | Surface normal arrows as RViz markers. Turn **on** for debugging surface tilt; off by default to save bandwidth. |
| `enable_drift_corrected_TF_publishing` | `true` | Publishes TF from corrected map origin. Useful if Nav2 uses this corrected frame as its global reference instead of the odometry-derived `/map`. |
| `enable_normal_color` | `false` | Includes surface normals as a "color" layer in published maps. Enables 3D shading in visualisers like CloudCompare or RTAB-Map. Leave off unless you're debugging terrain normals. |

### 12. Map Initialization

| Parameter | Default | Intuition |
|-----------|---------|-----------|
| `initialize_method` | `'linear'` | Interpolation for the first point-cloud seeding pass. `'cubic'` is smoothest but slow; `'nearest'` is fastest but blocky; `'linear'` (default) is a balanced sweet-spot for rover ground maps. |
| `initialize_frame_id` | `['base_link']` | TF frame used to seed the initial map section. Every entry creates one square centred on that frame. Use multiple entries (`[base_link, front_laser]`) to pre-seed terrain from several known sensor viewpoints. |
| `dilation_size_initialize` | `2` post-init smoothing kernel. Higher → smoother initial model at the cost of erasing small features (e.g., ground seams, minor curbs). 2-3 is adequate for most floor surfaces. |
| `use_initializer_at_start` | `true` | Auto-seed on first point cloud received. Set to **false** if you instead seed manually via RViz "2D Nav Goal → Publish Initial Pose" or programmatically calling `/initialize_map`. |

---

## 🗺️ Plugin Layers (Post-Processing Pipeline)

Plugins run sequentially in `config/core/plugin_config.yaml` **after** sensor fusion but **before** publishing:

```
Raw GPU Map  →  min_filter  →  smooth  →  inpaint  →  erosion  →  Published grid layers
```

| Plugin | Function | Key Params |
|--------|----------|------------|
| `min_filter` | Fill invalid/nan cells with the **minimum** height in the dilation radius neighbourhood. Acts as a first-pass gap filler. | `dilation_size`, `iteration_n` |
| `smooth_filter` | Simple smoothing (averaging) over the previous layer (`input_layer_name`). Reduces surface noise produced by point clouds with high variance. | — |
| `inpainting` | OpenCV inpainting of remaining nan regions using either the `telea` or `ns` (Navier-Stokes) algorithm. Produces visually clean edges but adds CPU overhead. | `method: telea\|ns` |
| `erosion` | Dilates traversability (or any selected layer) to smooth out small "islands" of unsafe value and widen safe corridors by the specified radius. Set `reverse=true` for a **closing** operation that shrinks traversable regions; use `false` for **opening** (expands them). | `dilation_size`, `iteration_n` |

---

## 📡 Key ROS2 Topics & Services

| Topic / Service | Direction | Type | Purpose |
|-----------------|-----------|------|---------|
| `/point_cloud` | In | `sensor_msgs/PointCloud2` | Raw sensor data from the Unitree L2 (or any compatible source) subscribed by the elevation map node. |
| `/elevation_mapping_node/elevation_map_raw` | Out | `grid_map_msgs/GridMap` | Raw GPU map layers — unfiltered, published at 5 Hz. |
| `/map` | Out | `nav_msgs/OccupancyGrid` | Costmap for Nav2 (30% traversability threshold maps below that to obstacle=value 100). Publishes transiently/local at QoS(1) so nav stack re-subscribes on connection and retains the latest published map. |
| `/odom/wheel` | Out | `nav_msgs/Odometry` | Wheel odometry (published by the odometry package from encoder readings + IMU quaternions). |
| `/encoder_data` | Out | `rover_msgs/EncoderMsg` | Raw left-right / front-rear encoder speeds (60 Hz) read over serial and published by **motor_kontrol**. Feeds into odometry. |
| `/joystick_cmd` | In | `rover_msgs/ControllerMsg` | Joystick commands from the RC receiver (**uzaktan_kumanda**) → forwarded to motor controller for manual driving. |

---

## 🔧 Nav2 Integration — Quick Reference

[Nav2](https://navigation.ros.org/) is the standard ROS2 navigation stack providing global/local path planning, obstacle avoidance, and recovery behaviours on top of the costmap produced by `costmap_converter`.

### Pipeline Overview

1. **`/map` (OccupancyGrid):** Published by your `costmap_converter` node from traversability data — Nav2 reads this as its static map layer.
2. **`/odom/wheel`:** Odometry feed for localisation within the map + robot-motion estimation during trajectory execution.
3. **`/cmd_vel`:** Nav2's output velocity commands forwarded to your rover's low-level controller.

### Typical Nav2 Parameter Files

Nav2 expects a set of YAML parameter files that define controllers, planners, costmaps, and behaviours. Your stack uses:

| Component | Nav2 Package | Key File / Namespace |
|-----------|-------------|---------------------|
| AMCL (localiser) | `nav2_amcl` | Parameters under `amcl` namespace — configure `odom_frame_id`, `map_frame_id`, `base_frame_id`, and initial pose estimate. |
| NavFn / DWB planner | `nav2_navfn_planner`, `nav2_dwb_local_planner` | Global path planning uses `nav_core2::NavFn`, local tracking with `dwb_controller::DWBLocalPlanner`. Tune the controller's `min_x_vel_bytes`, `max_x_vel_bytes`, and acceleration limits to your rover's dynamics. |
| Costmap layer | `nav2_costmap_2d` | The nav2_costmap node subscribes `/map` + `/odom/wheel` → produces a dynamic footprint-aware costmap for local navigation. The **Global costmap** reads from the static map; the **Local costmap** fuses sensor data and obstacle avoidance data dynamically at runtime. |
| Recovery & BT tree | `nav2_behaviors`, `behavior_tree/` | Recovery behaviours (e.g., spin, retract) and navigation behaviour trees define what happens when path planning fails or a localisation issue is detected. |

### Launching Nav2 with Your Stack

```bash
# 1. Ensure costmap_converter + odometry are running (see Running on a Real Rover above).

# 2. Launch Nav2 bringup (adapt to your robot's type — here we show the standard nav2_bringup):
ros2 launch nav2_bringup navigation_launch.py

# 3. RViz for visualisation:
rviz2 ...   # Add plugins: NavPose, TF, Map (subscribe to /map), Path, LaserScan
```

### Common Tuning Points in Your Set-Up

| Issue | Likely Cause | Fix |
|-------|-------------|-----|
| Nav2 sees the entire map as obstacles | `costmap_converter` threshold too low (`val < 0.3`). **Lower threshold** (e.g., `0.15`) if traversability scores are generally high, or increase them in the elevation map's traversability parameters so more cells become "safe" for Nav2 to traverse. |
| Path planning takes huge detours around your own shape | The robot footprint / costmap inflation radius doesn't account for your rover's size. Set `inflation_radius` and `cost_scaling_factor` in the param file to match your vehicle dimensions. |
| Robot "drifts" from planned path | Wheel odometry error or LiDAR sensor noise causing inaccurate localisation. Tune AMCL's `resample_interval` and `initial_pose_*`; for the costmap, check that `resolution` (0.5 m in your config) gives enough cells to distinguish narrow obstacles. |
| Nav2 gets stuck behind curbs/steps | Traversability threshold + robot height limits. If your rover can't climb >~5 cm curbs but the costmap marks them as safe, lower the traversability cutoff or add a `max_z` check in the map's layer configuration so that elevated regions are treated as obstacles rather than traversable terrain. |

### Nav2 + `corrected_map_frame`

If you enable `enable_drift_corrected_TF_publishing`, Nav2 can optionally use the corrected frame for its global reference. Set `global_frame: <corrected_frame_name>` and `map_frame: <corrected_frame_name>` in your navigation params so that the localiser doesn't drift away from the costmap as odometry degrades over time.

---

## 🔍 Tuning Quick-Reference

| Condition | Parameter to Adjust | Direction |
|-----------|-------------------|-----------|
| Noisy / sparse sensor data | `sensor_noise_factor`, `mahalanobis_thresh` | ↑ both for stricter filtering |
| Fast-moving rover, map "jumps" | `drift_compensation_alpha` | ↓ (e.g. 0.05) |
| Near-field floor reflected by LiDAR under chassis | `ramped_height_range_b`, `min_valid_distance` | ↑ |
| Nav2 costmap looks too noisy | Traverseability threshold in **costmap_converter** (`val < 0.3f → obstacle`) — increase the cutoff to e.g. `0.45`, or adjust `dilation_size` / plugin layers upstream | |
| Map grows large during multi-floor operation | `enable_overlap_clearance`, `overlap_clear_range_*` | ↑ ranges if floors are far apart |

---

## 📝 Notes on the `motor_kontrol` Module

```
Package                  Node                     Topic In           →  Topic Out / Hardware
----------------------------------------------|--------------------------|---------------------------
motor_kontrol            motor    (joystick_cmd       →   encoder_data    serial writes (STM32)
                         robot_kol  joystick_cmd      →   [serial port]   (robot-arm servos)
                         uzaktan_ RC receiver           →   joystick_cmd    (RC transmitter raw data)
```

All three nodes use `MultiThreadedExecutor` to prevent the joystick subscriber callback from starving the timer-based encoder reading — critical for avoiding stutter in real-time motor commands.

