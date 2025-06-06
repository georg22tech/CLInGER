import _thread
from machine import I2C, Pin, UART, ADC
import utime
import sys
from lidar import LIDAR  
from bno055 import BNO055
from vl53l1x import VL53L1X
from ina219 import INA219

# ----- Global Setup -----
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
led = Pin(25, Pin.OUT)
uart = UART(1, baudrate=115200)
adc = ADC(26)

# ----- Shared Sensor Data -----
sensor_data = {
    "luna": -1,
    "vl53": -1,
    "imu": None,
    "current": -1,
    "adc": 0.0
}

data_lock = _thread.allocate_lock()

# ----- Sensor Initialization -----
# TF-Luna
LIDAR_ADDRESS = 0x10
if LIDAR_ADDRESS not in i2c.scan():
    print('Error: TF-Luna not found.')
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

# BNO055
imu = BNO055(i2c)
calibrated = False

# INA219
try:
    ina = INA219(shunt_ohms=0.1, i2c=i2c, max_expected_amps=1.0)
    ina.configure()
    current_ready = True
except Exception as e:
    print(f"INA219 Init Error: {e}")
    current_ready = False

# ----- Sensor Read Functions -----
def read_tfluna_cm():
    try:
        return lidar.distance() / 10
    except:
        return -1

def read_vl53l1x_cm():
    if not vl53_ready:
        return -1
    try:
        return vl53.read() / 10
    except:
        return -1

def read_imu_data():
    global calibrated
    if not calibrated:
        if imu.calibrated():
            calibrated = True
        return None
    try:
        qx, qy, qz, qw = imu.quaternion()
        heading, roll, pitch = imu.euler()
        return (qx, qy, qz, qw, heading, roll, pitch)
    except:
        return None

def read_current_mA():
    if not current_ready:
        return -1
    try:
        return ina.current()
    except:
        return -1

def read_adc_voltage():
    return adc.read_u16() * 3.3 / 65535

# ----- Background Thread Function -----
def sensor_thread():
    global sensor_data
    while True:
        luna = read_tfluna_cm()
        vl53 = read_vl53l1x_cm()
        imu_vals = read_imu_data()
        current = read_current_mA()
        adc_v = read_adc_voltage()

        with data_lock:
            sensor_data["luna"] = luna
            sensor_data["vl53"] = vl53
            sensor_data["imu"] = imu_vals
            sensor_data["current"] = current
            sensor_data["adc"] = adc_v

        utime.sleep_ms(50)

# ----- Start Thread -----
_thread.start_new_thread(sensor_thread, ())

# ----- Main Loop -----
while True:
    with data_lock:
        luna = sensor_data["luna"]
        vl53 = sensor_data["vl53"]
        imu = sensor_data["imu"]
        current = sensor_data["current"]
        adc_v = sensor_data["adc"]

    if imu:
        qx, qy, qz, qw, heading, roll, pitch = imu
        print(f"{luna:.1f},{vl53:.1f},{qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f},{heading:.2f},{roll:.2f},{pitch:.2f},{current:.1f},{adc_v:.2f}")
        led.toggle()

    utime.sleep(0.1)
