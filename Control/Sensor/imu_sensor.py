from machine import I2C, Pin, UART
import time
from bno055 import BNO055

led = Pin(25, Pin.OUT)

i2c = I2C(0, scl=Pin(1), sda=Pin(0))
uart = UART(1, baudrate=115200)

imu = BNO055(i2c)

calibrated = False
while True:
    if not calibrated:
        calibrated = imu.calibrated()
        time.sleep(0.5)
        sys_cal, gyro_cal, accel_cal, mag_cal = imu.cal_status()
        print(f"Calibration required: sys {sys_cal} gyro {gyro_cal} accel {accel_cal} mag {mag_cal}")
    else:
        try:
            qx, qy, qz, qw = imu.quaternion()
            heading, roll, pitch = imu.euler()
            accel_x, accel_y, accel_z = imu.accel()
            print(f"{qx},{qy},{qz},{qw},{heading},{roll},{pitch}")
            led.toggle()
            time.sleep(0.1)
        except Exception as e:
            print(f"Error reading IMU data: {e}")
    time.sleep(0.05)
