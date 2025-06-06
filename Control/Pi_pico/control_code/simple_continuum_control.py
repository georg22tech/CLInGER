import machine
import time
import sys
import select
from machine import I2C, Pin
from ina219 import INA219

# ─── PWM SETUP ───
servo1 = machine.PWM(Pin(0))
servo2 = machine.PWM(Pin(1))
servo1.freq(50)
servo2.freq(50)

i2c1 = I2C(0, scl=Pin(13), sda=Pin(12))
# ina = INA219(i2c1)  # Optional INA219 setup

min_duty = 1800
max_duty = 8000
stop_duty = 4900

speed1 = 0.0
speed2 = 0.0
tension_mode = False
running1 = False
running2 = False

def scale_duty(target, value):
    return int(stop_duty + (target - stop_duty) * value)

def set_speed(input_value, tension=False):
    input_value = max(-1.0, min(input_value, 1.0))
    if tension:
        if input_value > 0:
            duty1 = scale_duty(max_duty, input_value)
            duty2 = scale_duty(min_duty, input_value)
        elif input_value < 0:
            duty1 = scale_duty(min_duty, -input_value)
            duty2 = scale_duty(max_duty, -input_value)
        else:
            duty1 = duty2 = stop_duty
    else:
        if input_value > 0:
            duty1 = duty2 = scale_duty(max_duty, input_value)
        elif input_value < 0:
            duty1 = duty2 = scale_duty(min_duty, -input_value)
        else:
            duty1 = duty2 = stop_duty

    servo1.duty_u16(duty1)
    servo2.duty_u16(duty2)

def stop_motor():
    servo1.duty_u16(stop_duty)
    servo2.duty_u16(stop_duty)

def set_speed_individual(servo, input_value, tension=False):
    input_value = max(-1.0, min(input_value, 1.0))
    if tension:
        if input_value > 0:
            duty1 = scale_duty(max_duty, input_value)
            duty2 = scale_duty(min_duty, input_value)
        elif input_value < 0:
            duty1 = scale_duty(min_duty, -input_value)
            duty2 = scale_duty(max_duty, -input_value)
        else:
            duty1 = duty2 = stop_duty
    else:
        if input_value > 0:
            duty1 = duty2 = scale_duty(max_duty, input_value)
        elif input_value < 0:
            duty1 = duty2 = scale_duty(min_duty, -input_value)
        else:
            duty1 = duty2 = stop_duty

    servo.duty_u16(duty1)  # Use duty1 only for each individual servo

def check_input():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def print_help():
    print("\n--- Command Options ---")
    print("exit           : Stop motors and exit program")
    print("x              : Stop both motors immediately")
    print(",              : Run both servos forward at full speed")
    print(".              : Run both servos backward at full speed")
    print("1:<speed>      : Set servo 1 speed (-1.0 to 1.0)")
    print("2:<speed>      : Set servo 2 speed (-1.0 to 1.0)")
    print("<speed>        : Set both servos speed (-1.0 to 1.0)")
    print("help           : Show this help message")
    print("----------------------\n")

print_help()

try:
    while True:
        cmd = check_input()
        if not cmd:
            time.sleep(0.05)
            continue

        print(f"> {cmd}")

        if cmd == "exit":
            stop_motor()
            print("Exiting.")
            break
        elif cmd == "help":
            print_help()
            continue
        elif cmd == "x":
            stop_motor()
            running1 = False
            running2 = False
        elif cmd == ",":
            speed1 = 1.0
            speed2 = 1.0
            tension_mode = False
            running1 = True
            running2 = True
        elif cmd == ".":
            speed1 = -1.0
            speed2 = -1.0
            tension_mode = False
            running1 = True
            running2 = True
        elif ':' in cmd:
            try:
                servo_id, val = cmd.split(':')
                val = float(val)
                if servo_id == '1':
                    speed1 = max(-1.0, min(val, 1.0))
                    running1 = True
                elif servo_id == '2':
                    speed2 = max(-1.0, min(val, 1.0))
                    running2 = True
                else:
                    print("Invalid servo id. Use '1' or '2'.")
            except Exception:
                print("Invalid command format. Use '1:<speed>' or '2:<speed>' where speed is between -1.0 and 1.0.")
        else:
            try:
                val = float(cmd)
                speed1 = speed2 = max(-1.0, min(val, 1.0))
                running1 = True
                running2 = True
                tension_mode = False
            except ValueError:
                print("Unknown command or invalid input.")

        if running1:
            set_speed_individual(servo1, speed1, tension_mode)
        else:
            servo1.duty_u16(stop_duty)

        if running2:
            set_speed_individual(servo2, speed2, tension_mode)
        else:
            servo2.duty_u16(stop_duty)

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    stop_motor()
    servo1.deinit()
    servo2.deinit()
    print("PWM deinitialized. All done.")

continuum control