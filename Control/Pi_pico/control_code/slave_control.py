from machine import Pin, I2C, UART
import time
import VL53L1X
import Pi_pico.pins as pins 

i2c_0 = [pins.IMU_0_0, pins.IMU_0_1]
i2c_1 = [pins.IMU_1_0, pins.IMU_1_1]
# I2C bus
i2c1 = I2C(0, scl=Pin(i2c_0[1]), sda=Pin(i2c_0[0]))
i2c2 = I2C(1, scl=Pin(i2c_1[1]), sda=Pin(i2c_1[0]))

sensor1 = VL53L1X(i2c1)
sensor2 = VL53L1X(i2c2)
sensor1.open()
sensor2.open()

# UART setup
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

while True:
    distance1 = sensor1.start_ranging()
    distance2 = sensor2.start_ranging()
    
    # Send both distances over UART
    uart.write(f"<SENSOR>{distance1},{distance2}</SENSOR>\n")
    time.sleep(0.5)
