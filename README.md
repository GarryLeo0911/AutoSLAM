ROS 2 Jazzy package for Freenove 4WD Smart Car

Nodes
- motor_node: subscribes `cmd_vel` (geometry_msgs/Twist) and drives motors.
- led_node: subscribes `led_color` (std_msgs/ColorRGBA) to set all LEDs.
- ultrasonic_node: publishes `range` (sensor_msgs/Range) at given rate.
- camera_node: publishes `image_raw` (sensor_msgs/Image) using Picamera2.

Launch
`ros2 launch ros2_freenove_4wd bringup.launch.py`

Parameters
- motor_node: `max_duty` (int, default 2000).
- led_node: `count` (int, default 8), `bus` (int, default 0), `device` (int, default 0).
- ultrasonic_node: `trigger_pin` (int, default 27), `echo_pin` (int, default 22), `frame_id` (str, default `ultrasonic`), `rate_hz` (float, default 10.0).
- camera_node: `width` (int, default 640), `height` (int, default 480), `frame_id` (str, default `camera`), `fps` (int, default 15).

Notes
- Servo and buzzer are intentionally omitted.
- Ensure SPI, I2C, and camera are enabled on the Pi.
- Picamera2 is required for camera_node.

