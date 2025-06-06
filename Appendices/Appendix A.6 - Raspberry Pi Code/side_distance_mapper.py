#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

class SideSensorMarkerPublisher(Node):
    def __init__(self):
        super().__init__('side_sensor_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'side_sensor/marker', 10)

        # Frame in which to publish the markers
        self.frame_id = "inching_unit_top_2"

        # Default sensor distances (in meters)
        self.left_distance = 1.0
        self.right_distance = 1.0

        # Subscribe to ToF distance topics (in cm)
        self.create_subscription(Float32, '/left_tof/distance', self.left_callback, 10)
        self.create_subscription(Float32, '/right_tof/distance', self.right_callback, 10)

        # Timer to publish markers at 2 Hz
        self.timer = self.create_timer(0.5, self.timer_callback)

    def left_callback(self, msg):
        self.left_distance = msg.data / 30.0  # Convert cm → m

    def right_callback(self, msg):
        self.right_distance = msg.data / 30.0  # Convert cm → m

    def timer_callback(self):
        self.publish_marker(self.left_distance, side='left', marker_id=0, color=(1.0, 1.0, 1.0))   # Red
        self.publish_marker(self.right_distance, side='right', marker_id=1, color=(1.0, 1.0, 1.0))  # Green

    def publish_marker(self, distance, side, marker_id, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"side_sensor_{side}"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position offset: left = +Y, right = -Y
        marker.pose.position.x = 0.0
        marker.pose.position.y = distance if side == 'left' else -distance
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Marker size (in meters)
        marker.scale.x = 0.05
        marker.scale.y = 0.01
        marker.scale.z = 0.05

        # Marker color (RGB + Alpha)
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0

        # Marker lifetime = 0 (forever until overwritten)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.publisher_.publish(marker)
        self.get_logger().info(f"Published {side} marker at {distance:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = SideSensorMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
