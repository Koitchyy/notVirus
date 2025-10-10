#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class ConstantControl(Node):
    def __init__(self):
        super().__init__("constant_control")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Bool, "/kill", self.kill, 10)
        self.timer = self.create_timer(0.2, self.callback)

    def callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.pub.publish(msg)

    def kill(self, msg):
        if msg.data:
            self.timer.cancel()
            self.pub.publish(Twist())



if __name__ == "__main__":
    rclpy.init()
    node = ConstantControl()
    rclpy.spin(node)
