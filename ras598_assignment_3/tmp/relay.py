# python3 src/ras598_assignment_3/tmp/relay.py
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class Relay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb, 10)

    def cb(self, msg):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = msg
        self.pub.publish(ts)

rclpy.init()
rclpy.spin(Relay())
