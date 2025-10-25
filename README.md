ROS 2 Jazzy package for Freenove 4WD Smart Car

**Overview**
- Drive motors, LEDs, ultrasonic, and a simple camera node.
- Optional SLAM with OAK‑D using RTAB‑Map (point cloud mode).
- Camera runs on the car; mapping and visualization run on the laptop.

**Prerequisites**
- Robot (car/RPi): OAK‑D connected and a camera driver installed. Use one of:
  - `oakd_pcloud` (recommended for point clouds)
  - `depthai_ros_driver` (if you prefer DepthAI’s stack)
- Laptop: `rtabmap_slam`, `rtabmap_odom`, and `rviz2` installed.

**Network Setup (both machines)**
- Same LAN and time sync (NTP). Allow UDP multicast in firewall.
- Set ROS 2 environment:
  - Linux/macOS: `export ROS_DOMAIN_ID=22; export ROS_LOCALHOST_ONLY=0; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - Windows PowerShell: `$Env:ROS_DOMAIN_ID=22; $Env:ROS_LOCALHOST_ONLY=0; $Env:RMW_IMPLEMENTATION='rmw_cyclonedds_cpp'`

**Run On The Robot (camera + actuators)**
- Option A: Include `oakd_pcloud` directly
  - `ros2 launch ros2_freenove_4wd oakd_camera.launch.py camera_launch:=oakd_pcloud.launch.py`
- Option B: Use the existing bringup with a camera package
  - `ros2 launch ros2_freenove_4wd slam_bringup.launch.py include_camera:=true camera_pkg:=oakd_pcloud camera_launch:=oakd_pcloud.launch.py`
- Adjust static TF if needed (camera mount):
  - `cam_parent_frame:=base_link cam_child_frame:=oak-d_frame`
  - `cam_x:=0.08 cam_y:=0.0 cam_z:=0.08 cam_roll:=0 cam_pitch:=0 cam_yaw:=0`

**Run On The Laptop (mapping + RViz)**
- Start RTAB‑Map in scan‑cloud mode, consuming the car’s point cloud:
  - `ros2 launch ros2_freenove_4wd laptop_rtabmap_oakd.launch.py cloud_topic:=/oak/stereo/points`
  - Tip: Find the exact topic from the robot: `ros2 topic list | findstr points`
  - Optional: disable ICP odom if you already publish odom: `start_icp_odom:=false`
- Start RViz with 3D map view:
  - `ros2 launch ros2_freenove_4wd laptop_viz.launch.py rvizconfig:=$((ros2 pkg prefix ros2_freenove_4wd)/share/ros2_freenove_4wd/rviz/oakd_viz.rviz)`

**What You Should See**
- Robot TF and model.
- RTAB‑Map’s 3D map on `/rtabmap/cloud_map` and obstacles on `/rtabmap/cloud_obstacles`.
- The map stays stable in `map` frame while odom may drift.

**Bandwidth Tips**
- Prefer compressed image transport over Wi‑Fi (when using images).
- Reduce camera resolution/FPS and adjust point cloud settings to lower throughput.

**Built‑In Nodes**
- `motor_node`: subscribes `cmd_vel` and drives motors.
- `led_node`: subscribes `led_color`.
- `ultrasonic_node`: publishes `range`.
- `camera_node`: simple Picamera2 publisher (optional if using OAK‑D).
- `odom_integrator_node`: integrates `cmd_vel` to publish `odom` and TF `odom->base_link`.

**Nav2 (optional)**
- Launch: `ros2 launch ros2_freenove_4wd nav2.launch.py map:=/path/to/map.yaml`
- Do not run `teleop_wasd` while Nav2 is active.

**Parameters (common)**
- `motor_node.max_duty` (int, default 2000)
- `ultrasonic_node.{trigger_pin, echo_pin, frame_id, rate_hz}`
- `camera_node.{width,height,frame_id,fps}`
- `odom_integrator_node.{odom_frame, base_frame, publish_rate_hz}`

**Notes**
- Ensure SPI, I2C, and camera are enabled on the Pi.
- For better odometry, replace the integrator with encoders + `robot_localization`.

