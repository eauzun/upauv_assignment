# Under Pressure AUV Team: Software Assignments

Weekly software assignments (ROS 2 Humble, Gazebo Fortress, C++17).

## Contents
- [Week 1: Heartbeat and camera bridge](#week-1)
- [Week 2: Red target detection and tracking](#week-2)
  - [Environment and dependencies](#environment-and-dependencies-1)
  - [Build and run](#build-and-run-1)
  - [Nodes, topics and QoS](#nodes-topics-and-qos)
  - [Parameters](#parameters)
  - [Detection algorithm](#detection-algorithm)
  - [Target path](#target-path)
  - [Evaluation](#evaluation)
  - [Demo commands](#demo-commands)
  - [Sources and AI assistance](#sources-and-ai-assistance)
- [Week 3](#week-3)

---

## Week 1

<details>
<summary><b>Week 1 details</b> (click to expand)</summary>

Under Pressure AUV Team, Week 1 Software Assignment.

### Environment & Dependencies
* **OS:** Ubuntu 22.04 LTS
* **ROS:** ROS 2 Humble
* **Simulator:** Gazebo Fortress
* **Dependencies:** `rclcpp`, `std_msgs`, `sensor_msgs`, `ros_gz_sim`, `ros_gz_bridge`, `rqt_image_view`

### Build & Run
```bash
git clone git@github.com:eauzun/upauv_assignment.git
cd upauv_assignment
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch simulation, bridge, and all nodes
ros2 launch up_week1 week1.launch.py
```

*(If using Docker, allow GUI access on host first: `xhost +local:docker`)*

View camera stream in a separate terminal:
```bash
ros2 run rqt_image_view rqt_image_view
```

The launch file takes an optional `world:=<path.sdf>` argument (default: `up_week1/worlds/test_world.sdf`). Week 2 uses it to load its own world.

### Nodes & Topics

| Node | Role | Topic | Message Type |
| :--- | :--- | :--- | :--- |
| `heartbeat_publisher` | Publishes candidate name & sequence number (1 Hz) | `/up/heartbeat` | `std_msgs/msg/String` |
| `heartbeat_subscriber` | Subscribes to `/up/heartbeat` and logs messages | `/up/heartbeat` | `std_msgs/msg/String` |
| `camera_monitor` | Reports width, height, encoding & frame count (1 Hz) | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `parameter_bridge` | Bridges Gazebo camera feed to ROS 2 | `/camera/image_raw` | `sensor_msgs/msg/Image` |

### World Objects (`worlds/test_world.sdf`, `up_test_world`)

| Object | Geometry | Position (`x, y, z` m) | Orientation (`r, p, y` rad) |
| :--- | :--- | :--- | :--- |
| `pool` (floor + walls) | Static `box` (`8.4 × 8.4 m`) | `0, 0, -2.1` (floor) | `0, 0, 0` |
| `red_target` | Red `box` (`3.0 × 0.15 × 0.02 m`) | `0, -1.5, -1.99` | `0, 0, 0` |
| `fish_1` | `capsule` + `box` | `-1.5, 1.5, -1.0` | `0, 0, 0.5` |
| `fish_2` | `capsule` + `box` | `1.5, -0.5, -1.5` | `0, 0, -0.8` |
| `block_1` | `cylinder` (`r=0.3, l=0.6`) | `0, 2.0, -1.7` | `0, 0, 0` |
| `fixed_camera` | RGB camera (`640×480`, `10 Hz`) | `0, -3.8, -1.0` | `0, 0.25, 1.5` |

</details>

---

## Week 2

<details open>
<summary><b>Week 2 details</b> (click to collapse)</summary>

Classical (non-deep-learning) red target detection in the Gazebo camera stream, published through ROS 2 interfaces, with a moving target, distractors, an occluding pillar, two lighting setups and a measurement tool.

### Environment and dependencies

* Ubuntu 22.04 LTS, ROS 2 Humble, Gazebo Fortress (`ign gazebo`), C++17, Python 3
* ROS packages: `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `vision_msgs`, `cv_bridge`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, `rqt_image_view` (viewer), `up_week1`
* Libraries: OpenCV (via `cv_bridge`), `ignition-transport11`, `ignition-msgs8`

```bash
sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-rqt-image-view \
  ros-humble-ros-gz libignition-transport11-dev libignition-msgs8-dev
```

### Build and run

`up_week1` and `up_week2` live in the same repository. Always build from the workspace root.

```bash
git clone git@github.com:eauzun/upauv_assignment.git
cd upauv_assignment
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch up_week2 week2.launch.py
```

Launch arguments:

| Argument | Values | Default | Effect |
| :--- | :--- | :--- | :--- |
| `lighting` | `normal`, `dim` | `normal` | Selects `worlds/test_world_w2.sdf` or `test_world_w2_dim.sdf` and the matching detector yaml |
| `use_mover` | `true`, `false` | `true` | `false` keeps the target at its fixed position (0, 0, -1.3) |
| `params_file` | path | per lighting | Detector parameter file (`config/detector.yaml` or `config/detector_dim.yaml`) |

The launch file includes the Week 1 launch (Gazebo, camera bridge, heartbeat nodes, camera monitor) with `world:=<week2 world>`, then starts a `/clock` bridge, `red_target_detector` and `target_mover`. Viewers are started separately:

```bash
ros2 run rqt_image_view rqt_image_view /vision/debug_image
```

The dim world differs from the normal world only in the sun light (diffuse 0.6 → 0.3, specular 0.1 → 0.05). The detector code is identical; only the parameter file differs.

### Nodes, topics and QoS

| Node | Role |
| :--- | :--- |
| `target_mover` | Moves `red_target` along the path in `config/target_path.yaml` by calling the Gazebo `/world/up_test_world/set_pose` service; publishes ground truth |
| `red_target_detector` | HSV-based red target detection, publishes detections, flag, offset and debug image |
| `evaluate.py` | Measures detection rate, false detection rate and center error against ground truth |
| Week 1 nodes | `heartbeat_publisher`, `heartbeat_subscriber`, `camera_monitor` (unchanged) |

| Topic | Type | Publisher | Notes |
| :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | bridge | Input (`rgb8` or `bgr8`), 10 Hz |
| `/vision/detections` | `vision_msgs/msg/Detection2DArray` | detector | Pixel center and size, sorted by area (descending); empty array when nothing is found |
| `/vision/target_detected` | `std_msgs/msg/Bool` | detector | Debounced presence of the largest valid candidate |
| `/vision/target_offset` | `geometry_msgs/msg/PointStamped` | detector | x, y: normalized offset from image center in [-1, 1] (right/down positive); z: target area / image area. Not published when there is no detection |
| `/vision/debug_image` | `sensor_msgs/msg/Image` | detector | `bgr8`, bounding box and center marker |
| `/target_mover/ground_truth` | `geometry_msgs/msg/PointStamped` | mover | World position of the target, 20 Hz, used by the evaluator |
| `/clock` | `rosgraph_msgs/msg/Clock` | bridge | Sim time for the mover |

All outputs copy `header.stamp` and `frame_id` from the processed input image.

**QoS:** input subscription `SensorDataQoS` (best effort, keep last 1) so latency never accumulates; `/vision/detections`, `/vision/target_detected`, `/vision/target_offset` reliable, keep last 10; `/vision/debug_image` reliable, keep last 1.

### Parameters

Loaded from `config/detector.yaml` (or `detector_dim.yaml`) and changeable at runtime with `ros2 param set /red_target_detector <name> <value>`. Invalid values are rejected. Use matching types (`s_min 200` for ints, `min_area 50.0` for doubles).

| Parameter | Type | Default | Meaning |
| :--- | :--- | :--- | :--- |
| `h1_low`, `h1_high` | int | 0, 6 | Lower red hue range (OpenCV H, 0-180) |
| `h2_low`, `h2_high` | int | 174, 180 | Upper red hue range (red wraps around 180) |
| `s_min` | int | 120 (dim: 110) | Minimum saturation (0-255) |
| `v_min` | int | 70 (dim: 30) | Minimum value/brightness (0-255) |
| `min_area` | double | 80.0 (dim: 60.0) | Smallest accepted contour area in px², must be ≥ 0 |
| `max_area_ratio` | double | 0.5 | Largest accepted area as a fraction of the image, in (0, 1] |
| `morph_kernel` | int | 3 | Odd kernel size (1-31) for opening/closing |
| `confirm_frames` | int | 3 | Consecutive frames with a candidate needed to set `target_detected` true |
| `lost_frames` | int | 3 | Consecutive frames without a candidate needed to set it false |
| `stream_timeout` | double | 1.0 | Seconds without images before the node reports "no target" |
| `publish_debug` | bool | true | Enables `/vision/debug_image` |

### Detection algorithm

1. Convert the image to `bgr8` (handles `rgb8` and `bgr8` input), then to HSV.
2. Threshold two hue ranges (red wraps around hue 0/180) with the saturation and value minimums, and OR the masks.
3. Morphological opening (removes speckle) and closing (fills gaps) with an elliptical kernel.
4. Find external contours; keep those with `min_area ≤ area ≤ max_area_ratio × image area`.
5. Sort candidates by area (descending); the largest is the target.
6. Debounce: `confirm_frames` consecutive hits set `target_detected` true, `lost_frames` consecutive misses set it false, so a single noisy frame does not change the state.
7. Publish detections, flag, offset (only when a candidate exists) and debug image.
8. A watchdog sets `target_detected` false if no image arrives for `stream_timeout` seconds; processing resumes by itself when the stream returns. Empty or corrupt images are ignored.

The node logs once per second: processed frames, detection state, center (px), area and average processing time (ms).

### Target path

Defined in `config/target_path.yaml` (waypoints with `move_time` and `hold_time`), repeated identically every run. Positions are in world coordinates.

TODO: paste the table from `target_path.yaml` (label, x, y, z, move_time, hold_time) and the total cycle time. The path must contain: hold in view (≥ 5 s), displacement ≥ 1 m in view, occlusion behind the pillar, out of view (≥ 3 s), re-entry.

Scene objects: red target (`red_target`), blue sphere `object_blue` (clearly different colour distractor), orange cube `object_orange` (near-red distractor), `occlusion_pillar` (cylinder r=0.24, at (-0.65, -1.2)), plus fish and block from Week 1.

### Evaluation

Ground truth is the target's world position published by the mover, projected into the image with the camera geometry from the SDF. A frame counts as *visible* when the target is fully inside the image and not behind the pillar, and as *invisible* when it is outside the image or centred behind the pillar. Partially occluded frames and the first `settle_frames` (default 4) frames after each visible/invisible change are skipped, because ground truth is asynchronous and the detector is intentionally debounced.

```bash
ros2 run up_week2 evaluate.py --ros-args -p lighting:=normal -p frames:=600 -p out_dir:=docs
```

| Lighting | Frames | Detection rate (≥ 90 %) | False detection (≤ 5 %) | Mean center error (≤ 10 px) |
| :--- | :--- | :--- | :--- | :--- |
| normal | TODO | TODO | TODO | TODO |
| dim | TODO | TODO | TODO | TODO |

Distractor check with default parameters: the orange cube is not detected. Raising `h1_high` to 25 makes it a false detection (screenshot in `docs/screenshots/`).

### Demo commands

```bash
ros2 launch up_week2 week2.launch.py                      # everything
ros2 launch up_week2 week2.launch.py lighting:=dim        # dim lighting
ros2 launch up_week2 week2.launch.py use_mover:=false     # static target
ros2 topic echo /vision/target_detected
ros2 topic echo /vision/target_offset
ros2 param set /red_target_detector s_min 200
ros2 param set /red_target_detector min_area -5.0         # rejected
```

### Sources and AI assistance

- OpenCV HSV thresholding and contour documentation, ROS 2 Humble and Gazebo Fortress documentation.
- AI assistance (Claude, Anthropic) was used for code drafts, debugging and the evaluation tool. TODO: list which parts, and confirm you can explain all of them.

</details>

---

## Week 3

<details>
<summary><b>Week 3</b> (click to expand)</summary>

Not started yet.

</details>
