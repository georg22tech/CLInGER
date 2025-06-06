from machine import I2C, Pin
import utime
import sys
from lidar import LIDAR

LIDAR_ADDRESS = 0x10

i2c_0 = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
utime.sleep_ms(50)

if LIDAR_ADDRESS not in i2c_0.scan():
    print('Bus error: Please check LIDAR wiring')
    raise SystemExit

lidar = LIDAR(i2c_0, LIDAR_ADDRESS)
lidar.set_min_max(0, 800)
lidar.set_frequency(250)

while True:
    distance = lidar.distance()
    print(f"{distance}\n")
    utime.sleep_ms(10)

