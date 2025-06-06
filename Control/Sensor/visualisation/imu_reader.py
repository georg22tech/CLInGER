import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import serial

class IMUReader(Node):
    def __init__(self):
        super().__init__('imu_reader')

        # Set up serial connection
        self.serial_port = "/dev/ttyACM0"  # Change if needed
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to IMU on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ROS publishers
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self.euler_publisher = self.create_publisher(Vector3Stamped, "/imu/euler", 10)

        # Timer to read serial data
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        """Reads serial data from IMU and publishes both Quaternion & Euler angles."""
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return  # Skip if no data

            # Debug: Print received data to verify format
            self.get_logger().info(f"Received Serial Data: {line}")

            parts = line.split(",")
            if len(parts) != 7:
                self.get_logger().warn(f"Invalid IMU data format: {line}")
                return

            # Extract and convert values
            qx, qy, qz, qw = map(float, parts[0:4])  # Quaternion values
            heading, roll, pitch = map(float, parts[4:7])  # Euler angles in degrees

            # Publish IMU quaternion message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link"
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            self.imu_publisher.publish(imu_msg)
            self.get_logger().info(f"Published IMU Quaternion: {qx}, {qy}, {qz}, {qw}")

            # Publish Euler angles (Degrees)
            euler_msg = Vector3Stamped()
            euler_msg.header.stamp = self.get_clock().now().to_msg()
            euler_msg.header.frame_id = "base_link"
            euler_msg.vector.x = roll    # Roll in degrees
            euler_msg.vector.y = pitch   # Pitch in degrees
            euler_msg.vector.z = heading  # Heading (Yaw) in degrees

            self.euler_publisher.publish(euler_msg)
            self.get_logger().info(f"Published IMU Euler Angles: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={heading:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")

def main():
    rclpy.init()
    node = IMUReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
