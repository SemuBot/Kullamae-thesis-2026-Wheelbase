#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray # <--- Changed types
from rclpy.qos import QoSProfile, ReliabilityPolicy

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        
        # Micro-ROS usually needs BEST_EFFORT reliability
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # 1. Match the Topic Name and Type from STM32
        self.motor_sub = self.create_subscription(
            Float32MultiArray,
            '/motors/current',
            self.motor_callback,
            qos
        )
        
        # 2. Match the Topic Name and Type from STM32
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/encoders/absolute',
            self.encoder_callback,
            qos
        )
        
        self.motor_data = []
        self.encoder_data = []
        
        self.get_logger().info('Waiting for Micro-ROS data...')
    
    def motor_callback(self, msg):
        # MultiArray data is stored in msg.data (a list)
        self.motor_data = msg.data
        self.print_status()
    
    def encoder_callback(self, msg):
        self.encoder_data = msg.data
        self.print_status()
    
    def print_status(self):
        # Clear screen (optional) or just print cleanly
        # specific logic to print only when we have both is better for display
        if len(self.motor_data) > 0 and len(self.encoder_data) > 0:
            print(f"--- Incoming Data ---")
            print(f"Motors (Amps): {self.motor_data}")
            print(f"Encoders (Abs): {self.encoder_data}")

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
