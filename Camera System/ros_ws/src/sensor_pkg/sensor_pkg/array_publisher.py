#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class ExtensionArrayPublisher(Node):
    def __init__(self):
        super().__init__('extension_array_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/actuator/extension_array',
            10)
        # Create a timer to call timer_callback every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        # Fill with 10 random floats in [0.0, 1.0)
        msg.data = [random.random() for _ in range(10)]
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ExtensionArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
