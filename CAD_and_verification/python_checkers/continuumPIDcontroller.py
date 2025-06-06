from imu import MPU6050
from machine import I2C,Pin
import math
import time
import RPi.GPIO as GPIO
import time

# Constants
ENA_PIN = 25  # GPIO pin connected to the EN1 pin L298N
IN1_PIN = 8  # GPIO pin connected to the IN1 pin L298N
IN2_PIN = 7  # GPIO pin connected to the IN2 pin L298N
IN3_PIN = 10  # GPIO pin connected to the IN1 pin L298N
IN4_PIN = 9  # GPIO pin connected to the IN2 pin L298N
# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)
GPIO.setup(IN3_PIN, GPIO.OUT)
GPIO.setup(IN4_PIN, GPIO.OUT)
pwm.ChangeDutyCycle(speed)
# Set ENA_PIN to HIGH to enable the actuator
GPIO.output(ENA_PIN, GPIO.HIGH)

i2c=I2C(0, sda=Pin(16), scl=Pin(17), freq=200000) #IMU I2C
mpu = MPU6050(i2c)

#Initial Constants 
error = 0
angle = 0
last_error = 0
intergral = 0
force = 125
roll=0
pitch=0
yaw=0
tLoop=0
cnt=0
#PID tuning Constantd
kp=1
ki=1
kd=1

try :
    while True:
        last_error = error # Last error for PID
        intergral = intergral+error*tloop # sums previous errors
        tStart=time.ticks_ms() #starts timer for loop calculations
        #IMU calculations
        xGyro=mpu.gyro.x
        yGyro=mpu.gyro.y
        zGyro=mpu.gyro.z
        roll=roll+yGyro*tLoop
        pitch=pitch+xGyro*tLoop
        yaw=yaw+zGyro*tLoop
        
        #prints values periodically
        cnt=cnt+1
        if cnt==10:
            cnt=0
            print('R: ',roll,'P: ',pitch,'Y: ',yaw )
            
        wait(0.1)
        
        tStop=time.ticks_ms()
        tLoop=(tStop-tStart)*.001
        #PID calcualtion
        dq = kp*error+ki*intergral+kd*(last_error-error)

        #sets the direction of the control based on the dq calculation
        
        if dq<0:
            pwm.ChangeDutyCycle(round(abs(force+dq))) # force is a constant value from 0 to 255
            GPIO.output(ENA_PIN, GPIO.pwm)
            GPIO.output(IN1_PIN, GPIO.HIGH)
            GPIO.output(IN2_PIN, GPIO.LOW)
            GPIO.output(IN3_PIN, GPIO.HIGH)
            GPIO.output(IN4_PIN, GPIO.LOW)
        elif dq>0:
            pwm.ChangeDutyCycle(round(abs(force+dq))) # force is a constant value from 0 to 255
            GPIO.output(ENA_PIN, GPIO.pwm)
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.HIGH)
            GPIO.output(IN3_PIN, GPIO.LOW)
            GPIO.output(IN4_PIN, GPIO.HIGH)
            
        error = roll-target # roll and target are place holders

except KeyboardInterrupt:
    pass

finally:
    # Cleanup GPIO on program exit
    GPIO.cleanup()

  