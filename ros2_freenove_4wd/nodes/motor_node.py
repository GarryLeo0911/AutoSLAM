import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ..hw.motor import Ordinary_Car


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('max_duty', 2000)
        self.max_duty = int(self.get_parameter('max_duty').get_parameter_value().integer_value)
        self.car = Ordinary_Car()
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.get_logger().info(f'Motor node started. max_duty={self.max_duty}')

    def cmd_vel_cb(self, msg: Twist):
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)
        lin = max(min(lin, 1.0), -1.0)
        ang = max(min(ang, 1.0), -1.0)

        left = lin * self.max_duty - ang * self.max_duty
        right = lin * self.max_duty + ang * self.max_duty

        # Map to 4 wheels: FL, BL, FR, BR
        fl = int(left)
        bl = int(left)
        fr = int(right)
        br = int(right)
        self.car.set_motor_model(fl, bl, fr, br)

    def destroy_node(self):
        try:
            self.car.set_motor_model(0, 0, 0, 0)
            self.car.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

