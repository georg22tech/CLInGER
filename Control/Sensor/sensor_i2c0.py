from machine import I2C, Pin, UART
import utime
import sys
from lidar import LIDAR  
from bno055 import BNO055
from vl53l1x import VL53L1X

# ----- Global Setup -----
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
led = Pin(25, Pin.OUT)
uart = UART(1, baudrate=115200)

# ----- Sensor Initialization -----
# TF-Luna
LIDAR_ADDRESS = 0x10
if LIDAR_ADDRESS not in i2c.scan():
    print('Error: TF-Luna not found. Check wiring.')
    raise SystemExit
lidar = LIDAR(i2c, LIDAR_ADDRESS)
lidar.set_min_max(0, 800)
lidar.set_frequency(250)

# VL53L1X
try:
    vl53 = VL53L1X(i2c)
    vl53_ready = True
except Exception as e:
    print(f"VL53L1X Init Error: {e}")
    vl53_ready = False

# IMU (BNO055)
imu = BNO055(i2c)
calibrated = False

# ----- Sensor Functions -----
def read_tfluna_cm():
    try:
        return lidar.distance() / 10  # mm to cm
    except Exception as e:
        print(f"TF-Luna Read Error: {e}")
        return -1

def read_vl53l1x_cm():
    if not vl53_ready:
        return -1
    try:
        return vl53.read() / 10  # mm to cm
    except Exception as e:
        print(f"VL53L1X Read Error: {e}")
        return -1

def read_imu_data():
    global calibrated
    if not calibrated:
        sys_cal, gyro_cal, accel_cal, mag_cal = imu.cal_status()
        print(f"Calibration required: sys {sys_cal} gyro {gyro_cal} accel {accel_cal} mag {mag_cal}")
        utime.sleep(0.5)
        calibrated = imu.calibrated()
        return None
    try:
        qx, qy, qz, qw = imu.quaternion()
        heading, roll, pitch = imu.euler()
        return (qx, qy, qz, qw, heading, roll, pitch)
    except Exception as e:
        print(f"IMU Read Error: {e}")
        return None

# ----- Main Loop -----
while True:
    distance_luna_cm = read_tfluna_cm()
    distance_vl53_cm = read_vl53l1x_cm()
    imu_data = read_imu_data()

    if imu_data:
        qx, qy, qz, qw, heading, roll, pitch = imu_data
        print(f"{distance_luna_cm:.1f},{distance_vl53_cm:.1f},{qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f},{heading:.2f},{roll:.2f},{pitch:.2f}")
        led.toggle()

    utime.sleep_ms(50)
