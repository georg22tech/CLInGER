#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, ColorRGBA
from visualization_msgs.msg import Marker
import sys, tty, termios, select

class DistanceMarker(Node):
    def __init__(self):
        super().__init__('distance_marker')

        # subscribe to live sensor
        self.create_subscription(Float32, '/distance', self.distance_callback, 10)

        # publisher for the cube marker
        self.marker_pub = self.create_publisher(Marker, '/distance_marker', 10)
        # ← NEW: publisher for the reference distance
        self.ref_pub    = self.create_publisher(Float32, '/gantry_reference_distance', 10)

        self.current_distance = 0.0
        self.saved_distance   = None

        # set up non-blocking stdin
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # poll at 20 Hz
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Press ‘0’ to capture /distance and publish reference + marker.")

    def distance_callback(self, msg: Float32):
        self.current_distance = msg.data

    def timer_callback(self):
        # if you hit ‘0’, capture & publish
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.read(1) == '0':
                self.saved_distance = self.current_distance
                self.get_logger().info(f"📌 Captured distance: {self.saved_distance:.2f} m")

                # 1) publish the reference distance
                ref_msg = Float32()
                ref_msg.data = self.saved_distance
                self.ref_pub.publish(ref_msg)

                # 2) drop the visualization cube
                self.publish_marker()

    def publish_marker(self):
        if self.saved_distance is None:
            return

        m = Marker()
        m.header.frame_id = 'distance_top'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns, m.id        = 'distance', 0
        m.type, m.action  = Marker.CUBE, Marker.ADD

        # place along +Y of the sensor
        m.pose.position.x = 0.0
        m.pose.position.y = float(self.saved_distance)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0

        # size
        m.scale.x, m.scale.y, m.scale.z = 0.25, 0.01, 0.25

        # true‐float color
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = 0.545, 0.271, 0.075, 1.0
        m.color = c

        self.marker_pub.publish(m)
        self.get_logger().info(f"🚩 Marker at {self.saved_distance:.2f} m in ‘distance_top’ frame")

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main():
    rclpy.init()
    node = DistanceMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
