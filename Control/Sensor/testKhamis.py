from machine import Pin
import sys

# ─── MOTOR PIN SETUP ───
bottom_motor = (Pin(21, Pin.OUT), Pin(20, Pin.OUT))   # IN1, IN2
top_motor    = (Pin(0, Pin.OUT), Pin(1, Pin.OUT)) # IN1, IN2
inch_motor   = (Pin(14, Pin.OUT), Pin(15, Pin.OUT)) # IN1, IN2

# ─── HELPERS ───
def drive(motor, in1_val, in2_val, label):
    motor[0].value(in1_val)
    motor[1].value(in2_val)
    if in1_val == in2_val == 0:
        print(f"{label}: STOP")
    elif in1_val == 1:
        print(f"{label}: FORWARD")
    else:
        print(f"{label}: REVERSE")

def stop_all():
    drive(bottom_motor, 0, 0, "Bottom motor")
    drive(top_motor, 0, 0, "Top motor")
    drive(inch_motor, 0, 0, "Inch motor")
    print("All motors stopped.")

# ─── USER PROMPT ───
print("3-Motor Directional Control")
print("Controls:")
print("  q → Bottom EXTEND    | a → Bottom RETRACT")
print("  w → Top EXTEND       | s → Top RETRACT")
print("  e → Inch FORWARD     | d → Inch REVERSE")
print("  x → Stop all")
print("  exit → Quit")

# ─── MAIN LOOP ───
while True:
    cmd = sys.stdin.readline().strip().lower()
    if not cmd:
        continue

    if cmd == "exit":
        stop_all()
        print("Exiting.")
        break
    elif cmd == "x":
        stop_all()
    elif cmd == "q":
        drive(bottom_motor, 1, 0, "Bottom motor")
    elif cmd == "a":
        drive(bottom_motor, 0, 1, "Bottom motor")
    elif cmd == "w":
        drive(top_motor, 1, 0, "Top motor")
    elif cmd == "s":
        drive(top_motor, 0, 1, "Top motor")
    elif cmd == "e":
        drive(inch_motor, 1, 0, "Inch motor")
    elif cmd == "d":
        drive(inch_motor, 0, 1, "Inch motor")
    else:
        print("Unknown command.")
