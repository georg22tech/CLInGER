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

# ─── MOTOR DIGITAL OUT SETUP ───
# UNIT 1
bottom_left_motor1  = (Pin(19, Pin.OUT), Pin(18, Pin.OUT))
bottom_right_motor1 = (Pin(17, Pin.OUT), Pin(16, Pin.OUT))
top_left_motor1     = (Pin(12, Pin.OUT), Pin(13, Pin.OUT))
top_right_motor1    = (Pin(14, Pin.OUT), Pin(15, Pin.OUT))
inch_motor_1        = (Pin(10, Pin.OUT), Pin(11, Pin.OUT))

# UNIT 2
bottom_left_motor2  = (Pin(28, Pin.OUT), Pin(27, Pin.OUT))
bottom_right_motor2 = (Pin(26, Pin.OUT), Pin(22, Pin.OUT))
top_left_motor2     = (Pin(21, Pin.OUT), Pin(20, Pin.OUT))
top_right_motor2    = (Pin(8,  Pin.OUT), Pin(9,  Pin.OUT))
inch_motor_2        = (Pin(6, Pin.OUT), Pin(7, Pin.OUT))

# GROUPS
bottom_group1 = [bottom_left_motor1, bottom_right_motor1]
top_group1    = [top_left_motor1, top_right_motor1]
bottom_group2 = [top_left_motor2, bottom_right_motor2]
top_group2    = [bottom_left_motor2, top_right_motor2]

def drive(motor, in1_val, in2_val, label):
    motor[0].value(in1_val)
    motor[1].value(in2_val)
    if in1_val == in2_val == 0:
        print(f"{label}: STOPPED")
    elif in1_val == 1:
        print(f"{label}: EXTEND")
    else:
        print(f"{label}: RETRACT")

def drive_group(group, in1_val, in2_val, label):
    for m in group:
        m[0].value(in1_val)
        m[1].value(in2_val)
    print(f"{label}: {'EXTEND' if in1_val else 'RETRACT'}")

def stop_all():
    for group in [top_group1, bottom_group1, top_group2, bottom_group2]:
        drive_group(group, 0, 0, "Group")
    inch_motor_1[0].value(0); inch_motor_1[1].value(0)
    inch_motor_2[0].value(0); inch_motor_2[1].value(0)
    print("All motors stopped.")

def check_input():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None

# ─── INSTRUCTIONS ───
# --- COMMANDS INFO ---
print("=== MOTOR & PWM CONTROL COMMANDS ===")
print("  t   → Inching motor 1 FORWARD     |  g  → Inching motor 1 REVERSE")
print("  y   → Inching motor 2 FORWARD     |  h  → Inching motor 2 REVERSE")
print("  q   → Top group 1 EXTEND          |  a  → Top group 1 RETRACT")
print("  w   → Bottom group 1 EXTEND       |  s  → Bottom group 1 RETRACT")
print("  e   → Top group 2 EXTEND          |  d  → Top group 2 RETRACT")
print("  r   → Bottom group 2 EXTEND       |  f  → Bottom group 2 RETRACT")
print("  x   → Stop all motors and servos")
print("  ,   → PWM full FORWARD            |  .  → PWM full BACKWARD")
print("  s   → PWM stop (neutral position)")
print("  [float] → Enter float [-1.0 to 1.0] for PWM speed")
print("  exit → Quit")
print("=====================================")


# ─── MAIN LOOP ───
speed = 0.0
tension_mode = False
running = False

try:
    while True:
        cmd = check_input()
        if not cmd:
            time.sleep(0.05)
            continue

        print(f"> {cmd}")

        if cmd == "exit":
            stop_all()
            print("Exiting.")
            break
        elif cmd == "x":
            stop_all()
            stop_motor()
            running = False

        # Digital OUT commands
        elif cmd == "t":
            drive(inch_motor_1, 1, 0, "Inching motor 1")
        elif cmd == "g":
            drive(inch_motor_1, 0, 1, "Inching motor 1")
        elif cmd == "y":
            drive(inch_motor_2, 1, 0, "Inching motor 2")
        elif cmd == "h":
            drive(inch_motor_2, 0, 1, "Inching motor 2")
        elif cmd == "q":
            drive_group(top_group1, 1, 0, "Top group 1")
        elif cmd == "a":
            drive_group(top_group1, 0, 1, "Top group 1")
        elif cmd == "w":
            drive_group(bottom_group1, 1, 0, "Bottom group 1")
        elif cmd == "s":
            drive_group(bottom_group1, 0, 1, "Bottom group 1")
        elif cmd == "e":
            drive_group(top_group2, 1, 0, "Top group 2")
        elif cmd == "d":
            drive_group(top_group2, 0, 1, "Top group 2")
        elif cmd == "r":
            drive_group(bottom_group2, 1, 0, "Bottom group 2")
        elif cmd == "f":
            drive_group(bottom_group2, 0, 1, "Bottom group 2")
        # PWM control updates
        elif cmd == ",":
            speed = 1.0
            tension_mode = False
            running = True
        elif cmd == ".":
            speed = -1.0
            tension_mode = False
            running = True
        else:
            try:
                speed = float(cmd)
                tension_mode = False
                running = True
            except ValueError:
                print("Unknown command or invalid input.")

        if running:
            set_speed(speed, tension_mode)
        else:
            stop_motor()

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    stop_motor()
    servo1.deinit()
    servo2.deinit()
    print("PWM deinitialized. All done.")
