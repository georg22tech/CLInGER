#!/usr/bin/env python3
import pygame
import threading
import serial 
import time
import signal
import sys
from picamera2 import Picamera2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


# ————————— Globals —————————
stop_event = threading.Event()

# Servo state
servo0_angle = 0
servo1_angle = 135
step = 2
servo0_min, servo0_max = 0, 180
servo1_min, servo1_max = 0, 270

# LED Brightness Variables
brightness_reset = 0
brightness = 10  # oStart at 25%
brightness_step = 5  # Step size for brightness
brightness_min, brightness_max = 0, 100  # Brightness limits

# Serial port
SERIAL_PORT = '/dev/ttyAMA0'
BAUDRATE = 500000

class ControllerNode(Node):
    def __init__(self):
        super().__init__('camera_controller')
        # Set up pygame, serial, publishers 

        # Joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Serial
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

        # Creating publishers
        self.servo_pub = self.create_publisher(
            Float32MultiArray,  # message type
            'servo_angles',     # topic name
            10                  # QoS history depth
        )

        self.led_pub = self.create_publisher(
            Float32,            # message type
            'led_brightness',   # topic name
            10                  # QoS
        )
        
        
    def reset_servos(self):
        self.ser.write(b"S,0,135\n")

    def reset_led(self):
        self.ser.write(b"L,0\n")

    def joystick_loop(self):
        global brightness,servo0_angle, servo1_angle

        while not stop_event.is_set():
            pygame.event.pump()
            if stop_event.is_set():
                break
            # Hat for servo control
            hat = self.joystick.get_hat(0) if self.joystick.get_numhats() else (0,0)
            if hat == (-1,0):
                servo0_angle = max(servo0_angle - step, servo0_min)
            if hat == (1,0):
                servo0_angle = min(servo0_angle + step, servo0_max)
            if hat == (0,1):
                servo1_angle = min(servo1_angle + step, servo1_max)
            if hat == (0,-1):
                servo1_angle = max(servo1_angle - step, servo1_min)

            # Send servo command
            command = f"S,{servo0_angle},{servo1_angle}\n"
            self.ser.write(command.encode('utf-8'))
            print(f"Sent Servo0: {servo0_angle}°, Servo1: {servo1_angle}°")


            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if self.joystick.get_button(3):  # Square button (Decrease brightness)
                        brightness = max(brightness - brightness_step, brightness_min)
                        command = f"L,{brightness}\n"
                        self.ser.write(command.encode('utf-8'))
                        print(f"Decreased brightness to {brightness}%")
                    elif self.joystick.get_button(1):  # Circle button (Increase brightness)
                        brightness = min(brightness + brightness_step, brightness_max)
                        command = f"L,{brightness}\n"
                        self.ser.write(command.encode('utf-8'))
                        print(f"Increased brightness to {brightness}%")

            # publish with explicit float casts:
            servo_msg = Float32MultiArray()
            # ensuring these are floats not ints:
            servo_msg.data = [float(servo0_angle), float(servo1_angle)]
            self.servo_pub.publish(servo_msg)

            led_msg = Float32()
            led_msg.data = float(brightness)
            self.led_pub.publish(led_msg)
            time.sleep(0.05)

    def cleanup(self, signum=None, frame=None):
        # Signal the thread to stop
        stop_event.set()

        # Wait up to 1s for it to finish
        joystick_thread.join(timeout=1.0)

        # Reset hardware and quit
        self.reset_servos()
        self.reset_led()
        time.sleep(0.1)
        self.ser.close()
        pygame.quit()
        sys.exit(0)

# ————————— Main —————————
if __name__ == "__main__":
    rclpy.init(args=None)
    joy = ControllerNode()
    # Register SIGINT/SIGTERM
    signal.signal(signal.SIGINT, joy.cleanup)
    signal.signal(signal.SIGTERM, joy.cleanup)

    # Start joystick thread (non‑daemon, so we can join)
    joystick_thread = threading.Thread(target=joy.joystick_loop)
    joystick_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        joy.cleanup()
