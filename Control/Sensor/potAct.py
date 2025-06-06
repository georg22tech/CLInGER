import sys
import time
import _thread
from machine import ADC, Pin


pot = ADC(0)

actuator_extend_pin = Pin(14, Pin.OUT)
actuator_retract_pin = Pin(15, Pin.OUT)

actuator_extend_pin.value(0)
actuator_retract_pin.value(0)


print("Initializing baseline...")
time.sleep(1)
baseline = pot.read_u16()
print("Baseline reading (0 cm):", baseline)


counts_per_cm = 1420  #  calibration

def adc_reader():
    """Continuously read and print the potentiometer value."""
    while True:
        current_value = pot.read_u16()  # Read ADC
        extension_cm = (current_value - baseline) / counts_per_cm  # Convert to cm
        
        
        print("Bit value:", current_value, "Extension (cm):", round(extension_cm, 2))
        
        time.sleep(0.5)  

def command_handler():
    """Handle actuator commands from standard input."""
    print("Enter command: 'e' for extend, 'r' for retract, or 's' for stop")
    while True:
        # Read a command from stdin
        command = sys.stdin.readline().strip().lower()
        
        if command == "e":
            actuator_extend_pin.value(1)
            actuator_retract_pin.value(0)
            print("Actuator extending")
        elif command == "r":
            actuator_extend_pin.value(0)
            actuator_retract_pin.value(1)
            print("Actuator retracting")
        elif command == "s":
            actuator_extend_pin.value(0)
            actuator_retract_pin.value(0)
            print("Actuator stopped")
        else:
            print("Unknown command. Use 'e' for extend, 'r' for retract, or 's' for stop.")
        
        time.sleep(0.1)  # Small delay before processing next command


_thread.start_new_thread(adc_reader, ())

# Run the command handler in the main thread
command_handler()
