import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class InchingUnitPublisher(Node):
    def __init__(self):
        super().__init__('inching_unit_publisher') 

        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        # Publish joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Define joints
        self.continuum_joints = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9','joint_10','joint_11'
        ]
        self.incher1_inching_joints = ['incher1_joint_1', 'incher2_joint_1', 'incher3_joint_1', 'incher4_joint_1']
        self.incher2_inching_joints = ['incher1_joint_2', 'incher2_joint_2', 'incher3_joint_2', 'incher4_joint_2']
        self.incher1_gripper_top_joints = ['gripper_topL_outside_joint_1', 'gripper_topR_outside_joint_1',
                                           'gripper_topL_inside_joint_1', 'gripper_topR_inside_joint_1']
        self.incher1_gripper_bottom_joints = ['gripper_bottomL_outside_joint_1', 'gripper_bottomR_outside_joint_1',
                                              'gripper_bottomL_inside_joint_1', 'gripper_bottomR_inside_joint_1']
        self.incher2_gripper_top_joints = ['gripper_topL_outside_joint_2', 'gripper_topR_outside_joint_2',
                                           'gripper_topL_inside_joint_2', 'gripper_topR_inside_joint_2']
        self.incher2_gripper_bottom_joints = ['gripper_bottomL_outside_joint_2', 'gripper_bottomR_outside_joint_2',
                                              'gripper_bottomL_inside_joint_2', 'gripper_bottomR_inside_joint_2']
        
        # Camera Joints (Updated names)
        self.camera_joint_1 = "motor_frame_1_camera_motor_1"  # Left/Right movement
        self.camera_joint_2 = "motor_frame_2_camera_motor_2"  # Up/Down movement

        self.incher1_joints = self.incher1_gripper_bottom_joints + self.incher1_gripper_top_joints + self.incher1_inching_joints
        self.incher2_joints = self.incher2_gripper_bottom_joints + self.incher2_gripper_top_joints + self.incher2_inching_joints
        self.joint_names = self.incher1_joints + self.incher2_joints + self.continuum_joints + [self.camera_joint_1, self.camera_joint_2]

        # Joint positions
        self.joint_positions = {name: 0.0 for name in self.joint_names}

        # Set movement limits
        self.max_extension = 0.150  # Maximum extension limit for inchers
        self.min_extension = 0.0    # Minimum extension limit
        self.max_camera_angle = math.radians(90)  # ±90 degrees in radians
        self.min_camera_angle = math.radians(-90)

        # Movement speed
        self.actuator_speed = 0.00048  # m/s
        self.update_rate = 0.05  # 20Hz update rate
        self.movement_per_cycle = self.actuator_speed  # 0.0048 * 0.05 = 0.00024m per cycle
        self.camera_step = math.radians(2.86)  # Move camera in ~2.86-degree steps

        # Joystick deadzone
        self.deadzone = 0.05  

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers for publishing joint states and transforms
        self.timer = self.create_timer(self.update_rate, self.publish_joint_states)
        self.transform_timer = self.create_timer(self.update_rate, self.broadcast_transforms)

    def publish_joint_states(self):
        """ Publish the current joint states at a fixed rate. """
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = [self.joint_positions[name] for name in self.joint_names]
        self.joint_pub.publish(js_msg)

    def update_joint_position(self, joint, delta):
        """ Update joint position with actuator limits. """
        new_position = self.joint_positions[joint] + delta
        self.joint_positions[joint] = max(self.min_extension, min(new_position, self.max_extension))
        self.get_logger().info(f"Moving {joint}: {self.joint_positions[joint]:.5f} m")

    def update_camera_position(self, joint, delta):
        """ Update camera joint position with ±90-degree limits. """
        new_position = self.joint_positions[joint] + delta
        self.joint_positions[joint] = max(self.min_camera_angle, min(new_position, self.max_camera_angle))
        self.get_logger().info(f"Camera {joint}: {math.degrees(self.joint_positions[joint]):.2f}°")

    def joy_callback(self, msg):
        """ Map joystick inputs to joint movements. """
        left_stick_y = msg.axes[1]
        right_stick_y = msg.axes[4]
        
        # L2 and R2 buttons
        l2_pressed = msg.buttons[6] 
        r2_pressed = msg.buttons[7]

        btn_x = msg.buttons[0]  
        btn_circle = msg.buttons[1]
        btn_square = msg.buttons[2]
        btn_triangle = msg.buttons[3]

        # D-pad axes for camera control
        dpad_up = msg.axes[7] > 0
        dpad_down = msg.axes[7] < 0
        dpad_left = msg.axes[6] < 0
        dpad_right = msg.axes[6] > 0

        # Joystick movement for inchers
        left_stick_movement = left_stick_y * self.movement_per_cycle if abs(left_stick_y) > self.deadzone else 0.0
        right_stick_movement = right_stick_y * self.movement_per_cycle if abs(right_stick_y) > self.deadzone else 0.0

        trigger_movement = self.movement_per_cycle if r2_pressed else -self.movement_per_cycle if l2_pressed else 0.0

        if left_stick_movement:
            for joint in self.incher1_inching_joints:
                self.update_joint_position(joint, left_stick_movement)

        if right_stick_movement:
            for joint in self.incher2_inching_joints:
                self.update_joint_position(joint, right_stick_movement)

        if btn_x and trigger_movement:
            for joint in self.incher1_gripper_bottom_joints:
                self.update_joint_position(joint, trigger_movement)

        if btn_circle and trigger_movement:
            for joint in self.incher1_gripper_top_joints:
                self.update_joint_position(joint, trigger_movement)

        if btn_triangle and trigger_movement:
            for joint in self.incher2_gripper_bottom_joints:
                self.update_joint_position(joint, trigger_movement)

        if btn_square and trigger_movement:
            for joint in self.incher2_gripper_top_joints:
                self.update_joint_position(joint, trigger_movement)

        # Camera movement using D-pad
        if dpad_left or dpad_right:
            self.update_camera_position(self.camera_joint_1, self.camera_step if dpad_right else -self.camera_step)

        if dpad_up or dpad_down:
            self.update_camera_position(self.camera_joint_2, self.camera_step if dpad_up else -self.camera_step)



    def broadcast_transforms(self):
        """
        Broadcast TF transforms for inching joints only, and set continuum joints as fixed based on incher movement.
        """
        for joint in self.joint_names:
            current_position = self.joint_positions[joint]
            
            # Create a TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"  # Parent frame is base_link
            transform.child_frame_id = joint  # The frame for this joint
            
            # Only update transforms for inching joints
            if joint in self.incher1_joints or joint in self.incher2_joints:
                transform.transform.translation.x = 0.0
                transform.transform.translation.y = 0.0
                transform.transform.translation.z = current_position  # Update along z-axis

                # No rotation required for inching joints, set to zero
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0  

                # Send the transform for inching joints
                self.tf_broadcaster.sendTransform(transform)

            # For continuum joints, assume they are fixed (no changes in position) but can be affected by incher movement
            if joint in self.continuum_joints:
                transform.transform.translation.x = 0.0
                transform.transform.translation.y = 0.0
                transform.transform.translation.z = 0.0  

                # Set to identity quaternion for fixed state (no rotation)
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0  

                # Send the fixed transform for continuum joints
                self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = InchingUnitPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
