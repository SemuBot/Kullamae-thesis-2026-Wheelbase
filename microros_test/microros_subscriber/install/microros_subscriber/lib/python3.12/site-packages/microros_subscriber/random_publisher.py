#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_random_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.get_logger().info("Publishing random /cmd_vel")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = random.uniform(-1.0, 1.0)
        msg.angular.z = random.uniform(-2.0, 2.0)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

