# Camera System File Strucutre 

`raspberry_pi_ws` containts `HRI` and `yolo_v11_model`. `HRI` contains GUI and servo commands scripts alongside some folders containing captured videos and images. `yolo_v11_model` folder contains the ncnn model of the custom trained object detection model used in the GUI live camera feed. 
`ros_ws` containts some testing scripts that were used to test the ROS2 topic subcribing features of the GUI.


# GUI script - gui_LLM.py

A PyQt5-based GUI for controlling and viewing a Raspberry Pi camera feed(via Picamera2) with live YOLO detection (which can be toggled), plus ROS 2-based sensor monitoring and actuator data. HRI folder contais `gui_LLM.py` which launches the GUI and ties together camera preview, YOLO object detection, joystick control, and ROS 2 topic subscriptions.

## Prerequisites

Before installing Python packages, make sure you have the following system-level requirements in place (especially on a Raspberry Pi):

- **Python 3.9+**  
  (venv running Python 3.11 was used for the `gui_LLM.py` scirpt)  
- **Virtual environment (venv)**  
  It’s highly recommended to isolate dependencies in a virtual environment.  
- **Picamera2 (Raspberry Pi)**  
  - On Raspberry Pi OS you can install via apt:  
    ```bash
    sudo apt update
    sudo apt install -y python3-picamera2 libcamera-apps
    ```

---

## Python Dependencies

Below is the complete list of packages required by `gui_LLM.py`. Install them into your virtual environment (or your global Python environment, if you choose) so that all imports resolve successfully:
pygame
pillow
piexif
opencv-python
numpy
ultralytics
PyQt5
rclpy
sensor-msgs
std-msgs
picamera2



# Servo Controller

Servo and LED‐ring control is handled by two complementary scripts:  
- **`camera_controller.py`** on the Raspberry Pi reads joystick inputs (via `pygame`) and publishes both ROS 2 topics (`servo_angles` and `led_brightness`) and raw UART commands (`S,<angle0>,<angle1>` for servos, `L,<brightness>` for LEDs).  
- **`Pico_controller.py`** running on the Raspberry Pi Pico listens over UART (at 500 000 baud) for those same commands, maps them to PWM signals for two servos (GP8 and GP9) and adjusts a NeoPixel LED ring (GPIO 15).  

To use this feature, you must wire up a UART link between the Pi’s TX/RX pins and the Pico’s UART0 pins (GP0/GP1). All other pin assignments (servo GPIOs, LED‐ring GPIO) are defined within each script itself.

