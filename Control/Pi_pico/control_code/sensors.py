from machine import I2C, Pin, ADC, Pin
import time
from vl53l1x import VL53L1X 
from bno055 import BNO055
from ina219 import INA219

# TFLuna via I2C
class TFLunaI2C:
    def __init__(self, i2c, address=0x10):
        """
        i2c: machine.I2C instance
        address: TFLuna I2C address (default 0x10)
        """
        self.i2c = i2c
        self.address = address

    def read_distance(self):
        try:
            # Read 2 bytes from register 0x00 (distance in cm)
            data = self.i2c.readfrom_mem(self.address, 0x00, 2)
            dist_cm = (data[0] << 8) | data[1]
            return dist_cm / 100.0  # Return in meters
        except Exception as e:
            print("TFLuna read error:", e)
            return None


# VL53L1X via I2C
class VL53L1XToF:
    def __init__(self, i2c):
        self.sensor = VL53L1X(i2c)
        self.sensor.open()
        self.sensor.start_ranging()

    def read_distance(self):
        try:
            dist_mm = self.sensor.get_distance()
            if dist_mm <= 0:
                return None
            return dist_mm / 1000.0  # Return in meters
        except Exception as e:
            print("VL53L1X read error:", e)
            return None

    def stop(self):
        self.sensor.stop_ranging()
        
class IMU:
    def __init__(self, i2c):
        self.imu = BNO055(i2c, address=0x29)

    def calibrate(self):
        time.sleep(0.5)
        sys_cal, gyro_cal, accel_cal, mag_cal = self.imu.cal_status()
        print(f"Calibration required: sys {sys_cal} gyro {gyro_cal} accel {accel_cal} mag {mag_cal}")

    def read(self):
        qx, qy, qz, qw = self.imu.quaternion()
        heading, roll, pitch = self.imu.euler()
        accel_x, accel_y, accel_z = self.imu.accel()
        return {
            "quaternion": (qx, qy, qz, qw),
            "euler": (heading, roll, pitch),
            "accel": (accel_x, accel_y, accel_z),
        }
    def get_accel(self):
        data = self.read()
        return data["accel"]

class INA:
    def __init__(self, i2c, address=0x40, offset=0.050, scale=0.0016):
        self.ina = INA219(0.1, i2c, address=address)
        self.ina.configure()
        self.offset = offset
        self.scale = scale

    def read(self):
        try:
            current = self.ina.shunt_voltage() / 0.1
            force = (current - self.offset) / self.scale
            return force
        except Exception as e:
            print("INA read error:", e)
            return None

class AccPot:
    def __init__(self,channel):
        self.pot = ADC(channel)
        self.counts_per_cm = 1420
        self._baseline = None  # Private variable to store baseline

    def calibrate(self):
        """Call this once to set baseline at 0 cm."""
        print("Initializing baseline...")
        time.sleep(1)
        self._baseline = self.pot.read_u16()
        print("Baseline reading (0 cm):", self._baseline)

    def read_pot(self):
        """Read and print potentiometer value and extension."""
        if self._baseline is None:
            print("Error: Baseline not set. Run calibrate() first.")
            return
        current_value = self.pot.read_u16()
        extension_cm = (current_value - self._baseline) / self.counts_per_cm
        print("Bit value:", current_value, "Extension (cm):", round(extension_cm, 2))
        time.sleep(0.5)
        return round(extension_cm, 2)
