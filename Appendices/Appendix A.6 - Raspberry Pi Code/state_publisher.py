#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ManualJointStatePublisher(Node):
    def __init__(self):
        super().__init__('manual_joint_state_publisher')

        # 1) Publisher on /joint_states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 2) Manually list every non-fixed joint in your Xacro hierarchy:
        self.joint_names = [
            # --- Inching Unit #1 ---
            'incher1_joint_1',
            'incher2_joint_1',
            'incher3_joint_1',
            'incher4_joint_1',
            'gripper_topL_outside_joint_1',
            'gripper_topR_outside_joint_1',
            'gripper_topL_inside_joint_1',
            'gripper_topR_inside_joint_1',
            'gripper_bottomL_outside_joint_1',
            'gripper_bottomR_outside_joint_1',
            'gripper_bottomL_inside_joint_1',
            'gripper_bottomR_inside_joint_1',

            # --- Inching Unit #2 ---
            'incher1_joint_2',
            'incher2_joint_2',
            'incher3_joint_2',
            'incher4_joint_2',
            'gripper_topL_outside_joint_2',
            'gripper_topR_outside_joint_2',
            'gripper_topL_inside_joint_2',
            'gripper_topR_inside_joint_2',
            'gripper_bottomL_outside_joint_2',
            'gripper_bottomR_outside_joint_2',
            'gripper_bottomL_inside_joint_2',
            'gripper_bottomR_inside_joint_2',

            # --- Actuators ---
#             'acc_1',         # prismatic actuator #1
#             'acc_1_motor',   # revolute motor on actuator #1
#             'acc_2',         # prismatic actuator #2

            # --- Continuum segments (11 revolute joints) ---
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
            'joint_8',
            'joint_9',
            'joint_10',
            'joint_11',

            # --- Camera servos ---
            'motor_frame_1_camera_motor_1',  # revolute
            'motor_frame_2_camera_motor_2',  # revolute
        ]

        # 3) Fire off a timer at 10 Hz to publish zeros
        self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = self.joint_names
        js.position = [0.0] * len(self.joint_names)
        js.velocity = [0.0] * len(self.joint_names)
        js.effort   = [0.0] * len(self.joint_names)
        self.joint_pub.publish(js)

def main():
    rclpy.init()
    node = ManualJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
