from machine import I2C, Pin
from PiicoDev_VL53L1X import PiicoDev_VL53L1X
from bno055 import BNO055
from lidar import LIDAR  # Ensure your file structure is correct
import time

# ---- I2C Setup ----
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)  # Top-side sensors
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400_000)  # Bottom-side sensors (+ LIDAR later)
xshut_left = Pin(16, Pin.OUT)

# ---- Power Cycle Left ToF to Avoid I2C Address Conflict ----
xshut_left.value(0)
time.sleep_ms(10)

# ---- Initialize Top-Side Sensors ----
imu_top = BNO055(i2c0, address=0x29)
xshut_left.value(1)
time.sleep_ms(100)
vl53_left = PiicoDev_VL53L1X(bus=0, sda=0, scl=1)
vl53_left.change_addr(0x30)

# ---- Initialize Bottom-Side Sensors ----
imu_bottom = BNO055(i2c1, address=0x28)
vl53_right = PiicoDev_VL53L1X(bus=1, sda=2, scl=3)

# ---- IMU Calibration Helpers ----
def imu_calibrated(imu):
    sys, g, a, m = imu.cal_status()
    return sys == 3 and g == 3 and a == 3 and m == 3

def show_imu_cal(imu, label):
    try:
        sys, g, a, m = imu.cal_status()
        print(f"{label}: SYS={sys} G={g} A={a} M={m}")
    except Exception as e:
        print(f"{label}: Calibration read error: {e}")

# ---- Wait for Full Calibration of Both IMUs ----
print("Waiting for IMU calibration...")
while True:
    top_ready = imu_calibrated(imu_top)
    bottom_ready = imu_calibrated(imu_bottom)

    show_imu_cal(imu_top, "Top")
    show_imu_cal(imu_bottom, "Bottom")

    if top_ready and bottom_ready:
        print("IMUs calibrated.")
        break

    time.sleep(0.5)

# ---- Now Initialize LIDAR (TF-Luna) on i2c1 After Calibration ----
LIDAR_ADDRESS = 0x10
slaves = i2c1.scan()
if LIDAR_ADDRESS not in slaves:
    print('Bus error: Please check LIDAR wiring')
    raise SystemExit

lidar = LIDAR(i2c1, LIDAR_ADDRESS)
lidar.set_min_max(20, 800)  # Output only distances between 20 and 800 cm
lidar.set_frequency(250)    # Set measurement frequency to 250 Hz

# ---- Main Loop: Read and Print Sensor Data ----
while True:
    try:
        # IMU0 (Top/Front)
        tqx, tqy, tqz, tqw = imu_top.quaternion()
        thead, troll, tpitch = imu_top.euler()
        print(f"IMU0:{tqx:.3f},{tqy:.3f},{tqz:.3f},{tqw:.3f},{thead:.2f},{troll:.2f},{tpitch:.2f}")
    except Exception as e:
        print("IMU0 read error:", e)
    
    try:
        # IMU1 (Bottom/Rear)
        bqx, bqy, bqz, bqw = imu_bottom.quaternion()
        bhead, broll, bpitch = imu_bottom.euler()
        print(f"IMU1:{bqx:.3f},{bqy:.3f},{bqz:.3f},{bqw:.3f},{bhead:.2f},{broll:.2f},{bpitch:.2f}")
    except Exception as e:
        print("IMU1 read error:", e)
    
    try:
        # TOF0 (Left)
        d_left = vl53_left.read() / 10
        print(f"TOF0:{d_left:.1f}")
    except Exception as e:
        print("TOF0 read error:", e)
    
    try:
        # TOF1 (Right)
        d_right = vl53_right.read() / 10
        print(f"TOF1:{d_right:.1f}")
    except Exception as e:
        print("TOF1 read error:", e)
    
    try:
        # LIDAR0
        d_lidar = lidar.distance()
        print(f"LIDAR0:{d_lidar:.1f}")
    except Exception as e:
        print("LIDAR0 read error:", e)
    
    time.sleep_ms(100)
