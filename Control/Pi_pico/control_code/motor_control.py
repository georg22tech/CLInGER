import machine
import time
from sensors import INA

class MotorControl:
    def __init__(self):
        # Setup PWM on GPIO15 and GPIO14
        self.servo1 = machine.PWM(machine.Pin(15))
        self.servo2 = machine.PWM(machine.Pin(14))
        self.servo1.freq(50)
        self.servo2.freq(50)
        i2c=machine.I2C(0, scl=machine.Pin(self.i2c_0[1]), sda=machine.Pin(self.i2c_0[0]))
        self.ina = self.INA(i2c)
        # Duty cycle limits
        self.min_duty = 1800     # Retracted
        self.max_duty = 8000     # Extended
        self.stop_duty = 4900    # Neutral / Stop

    def tension(self,motor,tension=100):
        current = self.ina.read()
        while current <=tension:
            if motor == 1:
                self.servo1.duty_u16(self.max_duty)
                time.sleep(0.5)
                self.servo1.duty_u16(self.stop_duty)
                time.sleep(0.5)
            if motor == 2:
                self.servo2.duty_u16(self.max_duty)
                time.sleep(0.5)
                self.servo2.duty_u16(self.stop_duty)
                time.sleep(0.5)

    def scale_duty(self, target, value):
        """ Interpolate between stop and target based on value from 0.0 to 1.0 """
        return int(self.stop_duty + (target - self.stop_duty) * value)

    def set_speed(self, input_value):
        """
        input_value: float from -1.0 to 1.0
        -1.0 = full reverse (servo1 back, servo2 forward)
         0.0 = stop
         1.0 = full forward (servo1 forward, servo2 back)
        """
        input_value = max(-1.0, min(input_value, 1.0))  # Clamp to [-1.0, 1.0]

        if input_value > 0:
            # Servo1 forward, Servo2 backward
            servo1_duty = self.scale_duty(self.max_duty, input_value)
            servo2_duty = self.scale_duty(self.min_duty, input_value)
        elif input_value < 0:
            # Servo1 backward, Servo2 forward
            servo1_duty = self.scale_duty(self.min_duty, -input_value)
            servo2_duty = self.scale_duty(self.max_duty, -input_value)
        else:
            # Stop
            servo1_duty = self.stop_duty
            servo2_duty = self.stop_duty

        self.servo1.duty_u16(servo1_duty)
        self.servo2.duty_u16(servo2_duty)
        time.sleep(0.5)
        self.servo1.duty_u16(self.stop_duty)
        self.servo2.duty_u16(self.stop_duty)
