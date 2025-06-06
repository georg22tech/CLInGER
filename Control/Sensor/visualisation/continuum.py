#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped, TransformStamped
import tf2_ros
import numpy as np
from transforms3d.quaternions import qinverse, qmult
from transforms3d.euler import quat2euler

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
        # Continuum joints (to be updated from IMU roll)
        self.continuum_joints = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6', 'joint_7'
        ]
        # Static joints from your URDF (they remain unchanged)
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

        # Baseline angles for continuum joints when a 90° (roll=π/2) bend is reached.
        # Adjust these baseline values to match your robot's desired 90° bending posture.
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

        # Baseline (neutral) quaternion for the continuum joints.
        # This should represent the sensor's orientation when no bending is applied.
        self.baseline_quat = [0.0, 0.0, 0.0, 1.0]

        # Latest heading (yaw) for TF (in degrees)
        self.heading_deg = 0.0

        # Subscribe to IMU data (quaternion) on '/imu/data'
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.get_logger().info("IMUJointPublisher (quaternion-based) initialized.")

    def imu_callback(self, msg: Imu):
        # Reorder the incoming quaternion as needed.
        current_quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # Compute the difference quaternion: q_diff = inverse(baseline_quat) * current_quat
        q_diff = qmult(qinverse(self.baseline_quat), current_quat)

        # Extract Euler angles (roll, pitch, yaw) from the difference quaternion (in radians)
        roll, pitch, yaw = quat2euler(q_diff)

        # For continuum joints, use the roll value.
        # Normalize: assume that ±90° (±π/2 radians) corresponds to ±1.
        norm = np.clip(roll / (np.pi / 2), -1.0, 1.0)
        for i, joint in enumerate(self.continuum_joints):
            self.joint_positions[joint] = norm * self.baseline_90deg[i]

        # For TF, use yaw (heading) and convert to degrees.
        self.heading_deg = np.rad2deg(yaw)

        self.get_logger().info(
            f"IMU: roll={np.rad2deg(roll):.1f}°, yaw={self.heading_deg:.1f}°, norm={norm:.2f}, "
            f"continuum: {[self.joint_positions[j] for j in self.continuum_joints]}"
        )

    def publish_joint_states(self):
        # Publish joint state message
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = [self.joint_positions[name] for name in self.joint_names]
        self.joint_pub.publish(js_msg)

    def publish_tf(self):
        # Publish a TF transform from "odom" to "base_link" using the heading.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"  # Global frame; adjust as needed.
        t.child_frame_id = "base_link"    #base_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Create a quaternion from heading (yaw) about the Z-axis.
        yaw_rad = np.deg2rad(self.heading_deg)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(yaw_rad / 2.0)
        t.transform.rotation.w = np.cos(yaw_rad / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
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

