import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from transforms3d.quaternions import qinverse, qmult
from transforms3d.euler import quat2euler, euler2quat

class IMUJointPublisher(Node):
    def __init__(self):
        super().__init__('imu_joint_publisher')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        # TF broadcaster for base_link transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to publish joint states and TF at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- Define joint names ---
        # Continuum joints (to be updated based on difference of top and bottom roll)
        self.continuum_joints = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6', 'joint_7'
        ]
        # Static joints from your URDF (remain unchanged)
        self.static_joints = [
            'joint_8',
            'incher1_joint_1', 'incher2_joint_1', 'incher3_joint_1', 'incher4_joint_1',
            'incher1_joint_2', 'incher2_joint_2', 'incher3_joint_2', 'incher4_joint_2',
            'gripper_topL_outside_joint_1', 'gripper_topR_outside_joint_1',
            'gripper_topL__inside_joint_1', 'gripper_topR_inside_joint_1',
            'gripper_bottomL_outside_joint_1', 'gripper_bottomR_outside_joint_1',
            'gripper_bottomL_inside_joint_1', 'gripper_bottomR_inside_joint_1',
            'gripper_topL_outside_joint_2', 'gripper_topR_outside_joint_2',
            'gripper_topL__inside_joint_2', 'gripper_topR_inside_joint_2',
            'gripper_bottomL_outside_joint_2', 'gripper_bottomR_outside_joint_2',
            'gripper_bottomL_inside_joint_2', 'gripper_bottomR_inside_joint_2'
        ]
        self.joint_names = self.continuum_joints + self.static_joints

        # Initialize all joint positions to 0.0
        self.joint_positions = {name: 0.0 for name in self.joint_names}

        # Baseline angles for continuum joints when a full 90° (roll difference) bend is reached.
        self.baseline_90deg = [
            0.100,  # joint_1
            0.168,  # joint_2
            0.211,  # joint_3
            0.222,  # joint_4
            0.265,  # joint_5
            0.286,  # joint_6
            0.319   # joint_7
        ]
        if len(self.baseline_90deg) != len(self.continuum_joints):
            self.get_logger().error("Baseline angles count does not match continuum joints count!")

        # Baseline (neutral) quaternion for the bottom IMU.
        self.baseline_quat = [0.0, 0.0, 0.0, 1.0]

        # Latest values for TF from the bottom IMU.
        self.heading_deg = 0.0      # Yaw in degrees (from bottom IMU)
        self.bottom_roll_rad = 0.0  # Bottom IMU roll in radians

        # Latest top roll input (in degrees), can span -180 to 180.
        self.top_roll_deg = 0.0

        # SUBSCRIPTIONS:
        # Bottom IMU subscription (quaternion) on '/imu/data'
        self.create_subscription(Imu, '/imu/data', self.bottom_imu_callback, 10)
        # Top roll input subscription (Float32 in degrees) on 'roll_angle_input'
        self.create_subscription(Float32, 'roll_angle_input', self.top_roll_callback, 10)

        self.get_logger().info("IMUJointPublisher (fusion-based) initialized.")

    def bottom_imu_callback(self, msg: Imu):
        current_quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        # Compute the difference quaternion: q_diff = inverse(baseline_quat) * current_quat
        q_diff = qmult(qinverse(self.baseline_quat), current_quat)
        # Extract Euler angles (roll, pitch, yaw) in radians
        roll, pitch, yaw = quat2euler(q_diff)
        self.bottom_roll_rad = roll
        self.heading_deg = np.rad2deg(yaw)
        self.get_logger().info(
            f"Bottom IMU: roll={np.rad2deg(roll):.1f}°, yaw={self.heading_deg:.1f}°"
        )

    def top_roll_callback(self, msg: Float32):
        # Accept top roll input (in degrees) in the full range [-180, 180]
        self.top_roll_deg = np.clip(msg.data, -180.0, 180.0)
        self.get_logger().info(f"Top roll input: {self.top_roll_deg:.1f}°")

    def update_continuum_joints(self):
        """
        Update continuum joints based on the difference between the top roll (in degrees)
        and the bottom roll (converted to degrees). The difference is wrapped to [-180, 180],
        then clipped to ±90°, and normalized (±90° maps to ±1). This normalized value
        is then multiplied by the baseline continuum joint values.
        """
        bottom_roll_deg = np.rad2deg(self.bottom_roll_rad)
        # Compute the difference
        roll_diff = self.top_roll_deg - bottom_roll_deg
        # Wrap the difference to [-180, 180]
        roll_diff_wrapped = (roll_diff + 180) % 360 - 180
        # Clip the difference to ±90°
        roll_diff_clipped = np.clip(roll_diff_wrapped, -90, 90)
        # Normalize: ±90° becomes ±1
        norm = roll_diff_clipped / 90.0

        self.get_logger().info(
            f"Top roll: {self.top_roll_deg:.1f}°, Bottom roll: {bottom_roll_deg:.1f}°, "
            f"Difference (wrapped): {roll_diff_wrapped:.1f}°, Clipped: {roll_diff_clipped:.1f}°, norm: {norm:.2f}"
        )

        # Update continuum joints using the normalized difference and baseline angles.
        for i, joint in enumerate(self.continuum_joints):
            self.joint_positions[joint] = norm * self.baseline_90deg[i]

    def publish_joint_states(self):
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = [self.joint_positions[name] for name in self.joint_names]
        self.joint_pub.publish(js_msg)

    def publish_tf(self):
        # Publish a TF transform from "odom" to "base_link" using bottom IMU's roll and yaw.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Create a quaternion from bottom IMU's roll and yaw (pitch=0)
        q = euler2quat(self.bottom_roll_rad, 0.0, np.deg2rad(self.heading_deg), axes='sxyz')
        # Reorder quaternion from (w, x, y, z) to (x, y, z, w)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        self.update_continuum_joints()
        self.publish_joint_states()
        self.publish_tf()

def main(args=None):
    rclpy.init(args=args)
    node = IMUJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


