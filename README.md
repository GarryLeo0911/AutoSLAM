ROS 2 Jazzy package for Freenove 4WD Smart Car

Nodes
- motor_node: subscribes `cmd_vel` (geometry_msgs/Twist) and drives motors.
- led_node: subscribes `led_color` (std_msgs/ColorRGBA) to set all LEDs.
- ultrasonic_node: publishes `range` (sensor_msgs/Range) at given rate.
- camera_node: publishes `image_raw` (sensor_msgs/Image) using Picamera2.
- odom_integrator_node: open-loop odometry by integrating `cmd_vel` to publish `odom` and TF `odom->base_link`.

Launch
`ros2 launch ros2_freenove_4wd bringup.launch.py`

SLAM (OAK‑D + RTAB‑Map)
- Purpose: Teleop the car while building a map and getting localization from RTAB‑Map. Nav2 is not required.
- Requirements: OAK‑D and ros-jazzy-depthai-ros installed; its `rtabmap.launch.py` available (usually in package `depthai_ros`).
- Start SLAM teleop bringup:
  - `ros2 launch ros2_freenove_4wd slam_bringup.launch.py`
  - Optional args:
    - `depthai_pkg:=<your_depthai_pkg>` (default `depthai_examples`)
    - `depthai_launch:=rtabmap.launch.py` (relative name under the DepthAI package's `launch/`)
    - `cam_parent_frame:=base_link` (parent frame)
    - `cam_child_frame:=oak-d_frame` (OAK‑D frame published by DepthAI)
    - `cam_x:=0.08 cam_y:=0.0 cam_z:=0.08 cam_roll:=0 cam_pitch:=0 cam_yaw:=0` (static TF from parent to camera)
- Notes:
  - This launch includes `motor_node`, `teleop_wasd`, `robot_state_publisher`, a static TF from `base_link` to the OAK‑D frame, and includes DepthAI’s `rtabmap.launch.py`.
  - If your DepthAI package name or launch path differs, pass the args above, or set them permanently by editing `launch/slam_bringup.launch.py`.
  - Make sure RTAB‑Map is configured to use `base_link` as the base frame and to publish `map->odom` TF. Visual odometry typically publishes `odom->base_link`.

Navigation (Nav2)
- Purpose: Run Nav2 to plan/control and publish `cmd_vel` to the car.
- Prereqs: `nav2_bringup` installed on your system; a map YAML for localization, or run SLAM to create one.
- URDF: installed at `share/ros2_freenove_4wd/urdf/freenove_4wd.urdf` with frames `base_footprint`, `base_link`, `ultrasonic`, `camera`.
- Odom: `odom_integrator_node` provides open-loop odom (drifts). Replace with encoder-based odom + `robot_localization` when available.

Run Nav2
- Start motors on the robot: `ros2 run ros2_freenove_4wd motor_node`
- Launch Nav2 (loads URDF, odom, and Nav2): `ros2 launch ros2_freenove_4wd nav2.launch.py map:=/path/to/map.yaml`
- Optional RViz: `ros2 launch nav2_bringup rviz_launch.py`
- Note: Do not run `teleop_wasd` while Nav2 is active; both publish `cmd_vel`. If needed, add `twist_mux`.

Parameters
- motor_node: `max_duty` (int, default 2000).
- led_node: `count` (int, default 8), `bus` (int, default 0), `device` (int, default 0).
- ultrasonic_node: `trigger_pin` (int, default 27), `echo_pin` (int, default 22), `frame_id` (str, default `ultrasonic`), `rate_hz` (float, default 10.0).
- camera_node: `width` (int, default 640), `height` (int, default 480), `frame_id` (str, default `camera`), `fps` (int, default 15).
- odom_integrator_node: `odom_frame` (str, default `odom`), `base_frame` (str, default `base_link`), `publish_rate_hz` (float, default 50.0).
- Nav2 params: see `share/ros2_freenove_4wd/config/nav2_params.yaml`.

Notes
- Servo and buzzer are intentionally omitted.
- Ensure SPI, I2C, and camera are enabled on the Pi.
- Picamera2 is required for camera_node.
- For obstacle avoidance, add a lidar and enable obstacle layers in Nav2. With only ultrasonic, use static maps or consider a range-sensor layer if supported by your ROS 2 distro.
