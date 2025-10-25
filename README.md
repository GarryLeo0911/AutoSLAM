ROS 2 Jazzy package for Freenove 4WD Smart Car

**Overview**
- Drive motors, LEDs, ultrasonic, and a simple camera node.
- Optional SLAM with OAK‑D using RTAB‑Map (point cloud mode).
- Camera runs on the car; mapping and visualization run on the laptop.

**Prerequisites**
- Robot (car/RPi): OAK‑D connected.
- If you use ROS 2 only: prefer `depthai_ros_driver` (native ROS 2).
- If you want an oakd_pcloud-style interface in ROS 2: use our ROS 2 wrapper `oakd_pcloud_ros2_compat.launch.py` which runs `depthai_ros_driver` and remaps topics to the oakd_pcloud names.
- If you must run the original `oakd_pcloud` (ROS 1): you’ll need a ROS 1 environment and `ros1_bridge`.
- Laptop: `rtabmap_slam`, `rtabmap_odom`, and `rviz2` installed (ROS 2).

**Network Setup (both machines)**
- Same LAN and time sync (NTP). Allow UDP multicast in firewall.
- Set ROS 2 environment:
  - Linux/macOS: `export ROS_DOMAIN_ID=22; export ROS_LOCALHOST_ONLY=0; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - Windows PowerShell: `$Env:ROS_DOMAIN_ID=22; $Env:ROS_LOCALHOST_ONLY=0; $Env:RMW_IMPLEMENTATION='rmw_cyclonedds_cpp'`

**Run On The Robot (camera + actuators)**
- Option A (oakd_pcloud-style in ROS 2):
  - `ros2 launch ros2_freenove_4wd oakd_pcloud_ros2_compat.launch.py`
  - Requires `depthai_ros_driver` and `topic_tools` installed.
  - Publishes PointCloud2 on `/stereo_rgb_node/stereo/points` and rectified images/camera_info with oakd_pcloud-compatible names.
  - Ensure a TF from `base_link` to the camera frame exists. With DepthAI, the point cloud often uses `<camera_name>_right_camera_optical_frame` (e.g., `oak_right_camera_optical_frame`). Set `cam_child_frame` accordingly in `slam_bringup.launch.py` or publish a matching static transform.
- Option B (pure ROS 2 camera, no oakd compatibility): use `depthai_ros_driver` via our bringup.
  - `ros2 launch ros2_freenove_4wd slam_bringup.launch.py include_camera:=true camera_pkg:=depthai_ros_driver camera_launch:=camera.launch.py`
- Option C (original oakd_pcloud - ROS 1) with bridging (advanced):
  1) On the robot (ROS 1 shell):
     - `roslaunch oakd_pcloud stereo_nodelet.launch camera_name:=oak`
     - This publishes `/stereo_rgb_node/stereo/points` (sensor_msgs/PointCloud2) in ROS 1.
  2) On the laptop, run `ros1_bridge` dynamic bridge so ROS 2 can see the point cloud:
     - Ensure both ROS 1 and ROS 2 are installed on the laptop.
     - Source both environments, then run: `ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics`
     - Verify the topic appears in ROS 2 with: `ros2 topic list | findstr stereo_rgb_node/stereo/points`
  3) Keep robot control on ROS 2: run our bringup (motors, TF):
     - `ros2 launch ros2_freenove_4wd slam_bringup.launch.py`
  - Adjust static TF if needed (camera mount):
     - `cam_parent_frame:=base_link cam_child_frame:=oak-d_frame`
     - `cam_x:=0.08 cam_y:=0.0 cam_z:=0.08 cam_roll:=0 cam_pitch:=0 cam_yaw:=0`

**Run On The Laptop (mapping + RViz)**
- Start RTAB‑Map in scan‑cloud mode, consuming the bridged point cloud:
  - `ros2 launch ros2_freenove_4wd laptop_rtabmap_oakd.launch.py`
  - Default `cloud_topic` is `/stereo_rgb_node/stereo/points` (oakd_pcloud). Override if you remap.
  - Tip: Discover topics: `ros2 topic list | findstr points`
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
