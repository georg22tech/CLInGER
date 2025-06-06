import neopixel
import time
from machine import UART, Pin, PWM

# UART Setup (Receive from Raspberry Pi)
uart = UART(0, baudrate=500000, tx=Pin(0), rx=Pin(1))

# NeoPixel LED Ring Setup
LED_PIN = 15  # GPIO 15 on Raspberry Pi Pico
NUM_PIXELS = 16  # Number of LEDs in the rin
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_PIXELS)

# Microservos Setup
servo0 = PWM(Pin(8))  # Servo 0 on GP14
servo1 = PWM(Pin(9))  # Servo 1 on GP15    
servo0.freq(50)
servo1.freq(50)

# Servo Angle to PWM Mapping - servo0 (0-180) - servo1 (0-270) 
min_duty_servo1 = 1638   
max_duty_servo1 = 8392  

min_duty_servo0 = 3267
max_duty_servo0 = 7690

# Start brightness at 50% (Range: 0 - 255)
brightness = 127

def set_servo0_angle(angle):
    """Set servo0 to given angle"""
    duty = int((angle / 180) * (max_duty_servo0 - min_duty_servo0) + min_duty_servo0)
    servo0.duty_u16(duty)
    
def set_servo1_angle(angle):
    """Set servo1 to given angle"""
    duty = int((angle / 270) * (max_duty_servo1 - min_duty_servo1) + min_duty_servo1)
    servo1.duty_u16(duty)


def set_brightness(level):
    """Set brightness of NeoPixels (scaled from 0-100% to 0-255)."""
    global brightness
    brightness = int((level / 100) * 255)  # Convert 0-100% to 0-255 scale
    for i in range(NUM_PIXELS):
        np[i] = (brightness, brightness, brightness)  # White color
    np.write()

# Set initial servo angles & brightness
set_servo0_angle(0)
set_servo1_angle(135)
set_brightness(10)

print("Waiting for UART data to control LEDs and Servos...")

while True:
    if uart.any():
        data = uart.readline().decode('utf-8').strip()
        try:
            parts = data.split(",")
            if parts[0] == "S":  # Servo command
                servo0_angle, servo1_angle = int(parts[1]), int(parts[2])
                set_servo0_angle(servo0_angle)
                set_servo1_angle(servo1_angle)
                print(f"Received Servo0: {servo0_angle}°, Servo1: {servo1_angle}°")
            
            elif parts[0] == "L":  # LED command
                brightness_level = int(parts[1])
                set_brightness(brightness_level)
                print(f"Updated LED Brightness: {brightness_level}%")

        except ValueError:
            print("Invalid data received.")
            print(parts)

    time.sleep(0.05)


