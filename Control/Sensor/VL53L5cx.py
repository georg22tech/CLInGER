from vl53l5cx.mp import VL53L5CXMP
from machine import Pin, I2C
from vl53l5cx import DATA_TARGET_STATUS, DATA_DISTANCE_MM
from vl53l5cx import STATUS_VALID, RESOLUTION_8X8
import time

def print_tof_results(sensor, grid=7):
    if sensor.check_data_ready():
        results = sensor.get_ranging_data()
        distance = results.distance_mm
        status = results.target_status
        print(f"ToF sensor data at address {sensor.addr}:")
        for i, d in enumerate(distance):
            if status[i] == STATUS_VALID:
                print("{:4}".format(d), end=" ")
            else:
                print("xxxx", end=" ")
            if (i & grid) == grid:
                print("")
        print("")

# I2C and sensor setup for Raspberry Pi Pico
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=1_000_000)
lpn = Pin(0, Pin.OUT, value=1)

tof = VL53L5CXMP(i2c, lpn=lpn)
tof.reset()

if not tof.is_alive():
    raise ValueError("VL53L5CX sensor not detected")

tof.init()
tof.resolution = RESOLUTION_8X8
tof.ranging_freq = 2

tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})

while True:
    print_tof_results(tof, grid=7)
    time.sleep(0.5)
