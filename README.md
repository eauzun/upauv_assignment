# up_week1

Under Pressure AUV Team — Week 1 Software Assignment (ROS 2 Humble & Gazebo Fortress, C++17).

## Environment & Dependencies
* **OS:** Ubuntu 22.04 LTS
* **ROS:** ROS 2 Humble
* **Simulator:** Gazebo Fortress
* **Dependencies:** `rclcpp`, `std_msgs`, `sensor_msgs`, `ros_gz_sim`, `ros_gz_bridge`, `rqt_image_view`

## Build & Run
```bash
# Clone into workspace and build
git clone <repo-url>
cd upauv_assignment
source /opt/ros/humble/setup.bash
colcon build --packages-select up_week1
source install/setup.bash

# Launch simulation, bridge, and all nodes
ros2 launch up_week1 week1.launch.py
```

*(If using Docker, allow GUI access on host first: `xhost +local:docker`)*

View camera stream in a separate terminal:
```bash
ros2 run rqt_image_view rqt_image_view
```

## Nodes & Topics

| Node | Role | Topic | Message Type |
| :--- | :--- | :--- | :--- |
| `heartbeat_publisher` | Publishes candidate name & sequence number (1 Hz) | `/up/heartbeat` | `std_msgs/msg/String` |
| `heartbeat_subscriber` | Subscribes to `/up/heartbeat` and logs messages | `/up/heartbeat` | `std_msgs/msg/String` |
| `camera_monitor` | Reports width, height, encoding & frame count (1 Hz) | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `parameter_bridge` | Bridges Gazebo camera feed to ROS 2 | `/camera/image_raw` | `sensor_msgs/msg/Image` |

## World Objects (`worlds/test_world.sdf` - `up_test_world`)

| Object | Geometry | Position (`x, y, z` m) | Orientation (`r, p, y` rad) |
| :--- | :--- | :--- | :--- |
| `pool` (floor + walls) | Static `box` (`8.4 × 8.4 m`) | `0, 0, -2.1` (floor) | `0, 0, 0` |
| `red_target` | Red `box` (`3.0 × 0.15 × 0.02 m`) | `0, -1.5, -1.99` | `0, 0, 0` |
| `fish_1` | `capsule` + `box` | `-1.5, 1.5, -1.0` | `0, 0, 0.5` |
| `fish_2` | `capsule` + `box` | `1.5, -0.5, -1.5` | `0, 0, -0.8` |
| `block_1` | `cylinder` (`r=0.3, l=0.6`) | `0, 2.0, -1.7` | `0, 0, 0` |
| `fixed_camera` | RGB camera (`640×480`, `10 Hz`) | `0, -3.8, -1.0` | `0, 0.25, 1.5` |
