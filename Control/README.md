# Clinger Control Code

This repository contains the control code for the **Clinger** robot, structured to run on a Raspberry Pi Pico. The project is organized to separate core logic, sensor initialization, and runtime control.

---

## Folder Structure

### [`pi_pico/`](./pi_pico/)
The root folder for code targeting the Raspberry Pi Pico.

- ### [`control_code/`](./pi_pico/control_code/)
  Contains the main control logic for the Clinger, including:

  - [`control.py`](./pi_pico/control_code/control.py):  
    Implements the complex control system, integrating sensor feedback to drive the robot's behavior.

  - [`main.py`](./pi_pico/control_code/main.py):  
    Entry point for running the Clinger's control system. It ties together sensor setup and control logic.

  - [`simple_control.py`](./pi_pico/control_code/simple_control.py):  
    A simplified controller for manual operation of the inching units using keyboard inputs.

- ### [`sensors/`](./sensors/)
  Contains sensor visualisation files for the Clinger.  
  > **Note**: Some sensor setup files require external libraries or drivers which can be easily found online. Make sure to install or include them if not already present.

---

### [`Raspberry_Pi/`](./Raspberry_Pi/)
Contains high-level robot visualization and external control tools.

- ### [`robot/`](./Raspberry_Pi/robot/)
  A ROS 2 package for simulating or visualizing the Clinger robot using its URDF model.

- ### [`controller_pkg/`](./Raspberry_Pi/controller_pkg/)
  This ROS 2 package converts PlayStation 4 controller (DualShock) joystick inputs into UART messages, which are then read by the Pi Pico for real-time control.

---

## Getting Started

### Pico Firmware

1. Flash your Raspberry Pi Pico with MicroPython.
2. Upload the contents of the `pi_pico/control_code/` folder to the Pico.
3. Run `main.py` or `simple_control.py` from a MicroPython IDE (e.g., Thonny):
   - Use `main.py` for full autonomous behavior.
   - Use `simple_control.py` for manual control via keyboard input.

---

# ROS 2 Launch Commands & PS4 Controller Mappings

This document outlines how to launch the ROS 2 components used with the Clinger robot, and provides the PS4 controller input mappings used in manual control mode.

---

## ROS 2 Launch Commands
1. Build the ROS 2 Workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

2. Launch the PS4 Controller Bridge
ros2 launch controller_pkg control.launch.py

3. Launch the sensor package

ros2 run visualisation/continuum.py



| **Function**              | **Input**                      | **Action**                  |
| ------------------------- | ------------------------------ | --------------------------- |
| **Holder Control**        | `L2` + 🔺 Triangle             | Retract Holder 1            |
|                           | `L2` + ⬜ Square                | Retract Holder 2            |
|                           | `L2` + 🟥 Circle               | Retract Holder 3            |
|                           | `L2` + ❌ Cross                 | Retract Holder 4            |
|                           | `R2` + 🔺 Triangle             | Extend Holder 1             |
|                           | `R2` + ⬜ Square                | Extend Holder 2             |
|                           | `R2` + 🟥 Circle               | Extend Holder 3             |
|                           | `R2` + ❌ Cross                 | Extend Holder 4             |
| **Continuum Steering**    | `L1`                           | Turn Continuum Left         |
|                           | `R1`                           | Turn Continuum Right        |
| **Inching Unit Movement** | Left Joystick (Vertical Axis)  | Move Inching Unit 1 Up/Down |
|                           | Right Joystick (Vertical Axis) | Move Inching Unit 2 Up/Down |
| **Camera Control**        | D-Pad Left / Right             | Swivel Camera Left / Right  |
|                           | D-Pad Up / Down                | Tilt Camera Up / Down       |

