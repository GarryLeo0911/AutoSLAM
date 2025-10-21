import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomIntegratorNode(Node):
    """Integrate `cmd_vel` to publish odom and TF (odom->base_link).

    NOTE: This is open-loop odometry. It will drift without real wheel encoders
    or an external localization source. Prefer encoders + robot_localization or
    visual-inertial odometry for production use.
    """

    def __init__(self) -> None:
        super().__init__('odom_integrator_node')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self.dt = 1.0 / max(rate, 1.0)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f'Odom integrator started: odom_frame={self.odom_frame}, base_frame={self.base_frame}, rate={rate}Hz'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _integrate(self, dt: float) -> None:
        if self.last_cmd is None:
            return
        vx = float(self.last_cmd.linear.x)
        vth = float(self.last_cmd.angular.z)

        # Simple unicycle integration in the base frame
        if abs(vth) < 1e-6:
            dx = vx * dt * math.cos(self.yaw)
            dy = vx * dt * math.sin(self.yaw)
            dth = 0.0
        else:
            # Exact integration for constant velocities
            dth = vth * dt
            r = vx / vth if abs(vth) > 1e-6 else 0.0
            dx = r * (math.sin(self.yaw + dth) - math.sin(self.yaw))
            dy = -r * (math.cos(self.yaw + dth) - math.cos(self.yaw))

        self.x += dx
        self.y += dy
        self.yaw = (self.yaw + dth + math.pi) % (2 * math.pi) - math.pi

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        # Zero command after 0.5s timeout for safety
        if self.last_cmd is not None and (now - self.last_cmd_time) > Duration(seconds=0.5):
            self.last_cmd = Twist()

        self._integrate(self.dt)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if self.last_cmd is not None:
            odom.twist.twist.linear.x = float(self.last_cmd.linear.x)
            odom.twist.twist.angular.z = float(self.last_cmd.angular.z)

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomIntegratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

