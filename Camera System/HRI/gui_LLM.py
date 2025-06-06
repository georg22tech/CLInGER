import sys
import os
import time
import threading
import signal
import pygame
from picamera2 import Picamera2
from picamera2.previews.qt import QGlPicamera2
from libcamera import controls
from PIL import Image
import piexif
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32, Float32MultiArray
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QTabWidget,
    QGroupBox, QHBoxLayout, QLabel, QSlider, QButtonGroup, QRadioButton,
    QPushButton, QSizePolicy, QGridLayout, QCheckBox
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
import collections
import cv2
import numpy as np
from ultralytics import YOLO
from PyQt5.QtGui import QImage, QPixmap


# Global stop event
stop_event = threading.Event()
# Lock to prevent overlapping captures
capture_lock = threading.Lock()

# Initialize joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Focus
focus_position = 5.0
focus_step = 0.2
focus_min, focus_max = 0.0, 10.0

# Image & Video folder
image_folder = os.path.join(os.path.expanduser("~"),"raspberry_pi_ws",
                             "HRI", "FD_images")
os.makedirs(image_folder, exist_ok=True)
video_folder = os.path.join(os.path.expanduser("~"),"raspberry_pi_ws",
                             "HRI", "FD_videos")
os.makedirs(video_folder, exist_ok=True)

# Camera init
picam2 = Picamera2()
preview_width = 800
preview_height = int(picam2.sensor_resolution[1] * preview_width / picam2.sensor_resolution[0])
cfg = picam2.create_preview_configuration(
             main={"format": "XRGB8888",
                   "size": (preview_width, preview_height)}
         )
picam2.configure(cfg)
picam2.set_controls({
    "AfMode": controls.AfModeEnum.Manual,
    "LensPosition": focus_position,
    "AwbMode": controls.AwbModeEnum.Auto
})


class SliderSetting:
    def on_Setting_valueChanged(self):
        value = float(self.sliderSetting.value()) / (100 if self.setting_name=="LensPosition" else self.factor)
        self.labelSetting.setText(f"{self.setting_name}: {value}")

    def on_Setting_sliderReleased(self):
        valueSetting = float(self.sliderSetting.value()) / (100 if self.setting_name=="LensPosition" else self.factor)
        self.callback(self.setting_name, valueSetting)
        
    def __init__(self, setting_name, factor, callback):
        self.setting_name = setting_name
        self.factor = factor
        self.callback = callback
        # Get control limits and default
        cam_controls = picam2.camera_controls
        self.sliderSetting = QSlider(Qt.Horizontal)
        self.sliderSetting.setMinimum(int(cam_controls[self.setting_name][0] * self.factor))
        self.sliderSetting.setMaximum(int(cam_controls[self.setting_name][1] * self.factor))
        # store default for reset
        default = cam_controls[setting_name][2]
        self.default = default
        self.sliderSetting.setValue(int(self.default * self.factor))
        self.sliderSetting.setTickPosition(QSlider.TicksBelow)
        self.sliderSetting.setTickInterval(1)
        self.sliderSetting.sliderReleased.connect(self.on_Setting_sliderReleased)
        self.sliderSetting.valueChanged.connect(self.on_Setting_valueChanged)
        self.labelSetting = QLabel(f"{setting_name}: {self.default}")

    def apply(self):
        val = self.sliderSetting.value() / self.factor
        # Apply camera control via set_controls
        picam2.set_controls({self.setting_name: val})
        self.callback(self.setting_name, val)
    def reset(self):
        # reset slider to its default value and apply
        self.sliderSetting.setValue(int(self.default * self.factor))
        self.apply()

class MainWidget(QWidget):
    captureFinished = pyqtSignal()

    def __init__(self):
        super().__init__()
        main_layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        # Control Tab
        self.tabControl = QWidget()
        self.settings_layout = QVBoxLayout(self.tabControl)
        self.tabControl.setLayout(self.settings_layout)

        # Sensor Monitoring Tab
        self.tabSensors = QWidget()
        self.sensor_layout = QGridLayout(self.tabSensors)

        # Dev Tab
        self.tabDev = QWidget()
        self.dev_layout = QGridLayout(self.tabDev)


        # Labelling Tabs
        self.tabs.addTab(self.tabSensors, "   Sensors   ")
        self.tabs.addTab(self.tabControl, "   Control   ")
        self.tabs.addTab(self.tabDev, "   Dev/Eng   ")

        # for sliding-window FPS:
        self._timestamps = collections.deque()

        # Live camera feed on top
        # —––– YOLO-NCNN preview setup –––—
        # 1) Load your NCNN-exported YOLO11n model
        # point this at the real location of your NCNN export:
        model_dir = os.path.expanduser("~/raspberry_pi_ws/yolo_v11_model/cracks_v4/cracks_v4_ncnn_model")
        if not os.path.isdir(model_dir):
            raise FileNotFoundError(f"YOLO model folder not found: {model_dir}")
        self.yolo_model = YOLO(model_dir, task="detect")
        self.conf_threshold = 0.5
        self.resize_dims = (preview_width, preview_height)

        # 2) Replace the QGlPicamera2 widget with a simple QLabel
        #    (so we can draw our own boxes)
        self.preview_label = QLabel()
        main_layout.addWidget(self.preview_label, stretch=3, alignment=Qt.AlignHCenter)
        

        # 3) Start the camera and kick off a QTimer to update frames
        picam2.start()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # ~33 fps

        # Tabbed controls at bottom
        main_layout.addWidget(self.tabs, stretch=1)

        self.camControls = QGroupBox("Camera Controls")
        sub = QVBoxLayout(self.camControls)
        self.settings_layout.addWidget(self.camControls)

        #---------------------------------Brightness slider---------------------------------
        def callback_Brightness(setting_name, value):
            with picam2.controls as cam_controls:
                cam_controls.Brightness = value
                print(f"{value} => {setting_name} = {cam_controls.Brightness}")
        self.sliderBrightness = SliderSetting("Brightness", 10, callback_Brightness)
        sub.addWidget(self.sliderBrightness.labelSetting)
        sub.addWidget(self.sliderBrightness.sliderSetting)

        #---------------------------------Contrast slider---------------------------------
        def callback_Contrast(setting_name, value):
            with picam2.controls as cam_controls:
                cam_controls.Contrast = value
                print(f"{value} => {setting_name} = {cam_controls.Contrast}")
        self.sliderContrast = SliderSetting("Contrast", 1, callback_Contrast)
        sub.addWidget(self.sliderContrast.labelSetting)
        sub.addWidget(self.sliderContrast.sliderSetting)

        #---------------------------------ExposureValue slider---------------------------------
        def callback_ExposureValue(setting_name, value):
            with picam2.controls as cam_controls:
                cam_controls.ExposureValue = value
                print(f"{value} => {setting_name} = {cam_controls.ExposureValue}")
        self.sliderExposureValue = SliderSetting("ExposureValue", 1, callback_ExposureValue)
        sub.addWidget(self.sliderExposureValue.labelSetting)
        sub.addWidget(self.sliderExposureValue.sliderSetting)

        #---------------------------------Saturation slider---------------------------------
        def callback_Saturation(setting_name, value):
            with picam2.controls as cam_controls:
                cam_controls.Saturation = value
                print(f"{value} => {setting_name} = {cam_controls.Saturation}")
        self.sliderSaturation = SliderSetting("Saturation", 1, callback_Saturation)
        sub.addWidget(self.sliderSaturation.labelSetting)
        sub.addWidget(self.sliderSaturation.sliderSetting)

        #---------------------------------Sharpness slider---------------------------------
        def callback_Sharpness(setting_name, value):
            with picam2.controls as cam_controls:
                cam_controls.Sharpness = value
                print(f"{value} => {setting_name} = {cam_controls.Sharpness}")
        self.sliderSharpness = SliderSetting("Sharpness", 1, callback_Sharpness)
        sub.addWidget(self.sliderSharpness.labelSetting)
        sub.addWidget(self.sliderSharpness.sliderSetting)

        #---------------------------------Focus Slider---------------------------------
        self.focus_position = focus_position
        def callback_Focus(name, v):
            # v will be a float in [focus_min, focus_max]
            self.focus_position = v
            picam2.set_controls({"LensPosition": v})
            print(f"{v:.2f} => {name} = {v:.2f}")

        # The camera_controls entry for LensPosition gives (min, max, default)
        cam_ctrls = picam2.camera_controls
        min_fp, max_fp, default_fp = cam_ctrls["LensPosition"]
        # We'll multiply by 100 so slider is 0…100*(max-min)
        self.sliderFocus = SliderSetting(
            "LensPosition",
            factor=1.0,           # no scaling; slider uses actual units 0.0–10.0
            callback=callback_Focus
        )
        # override its range to exactly your min/max:
        self.sliderFocus.sliderSetting.setMinimum(int(min_fp * 100))
        self.sliderFocus.sliderSetting.setMaximum(int(max_fp * 100))
        self.sliderFocus.sliderSetting.setValue(int(default_fp * 100))
        sub.addWidget(self.sliderFocus.labelSetting)
        sub.addWidget(self.sliderFocus.sliderSetting)

        #---------------------------------White Balance Buttons ---------------------------------
        wb_box = QGroupBox("White Balance")
        wb_layout = QHBoxLayout(wb_box)
        self.wb_group = QButtonGroup(self)
        modes = ["Auto","Tungsten","Fluorescent","Indoor","Daylight","Cloudy","Custom"]
        for idx, mode in enumerate(modes):
            btn = QRadioButton(mode)
            if mode == "Auto":
                btn.setChecked(True)
            self.wb_group.addButton(btn, idx)
            wb_layout.addWidget(btn)
        self.wb_group.buttonClicked.connect(self.on_wb_mode)
        sub.addWidget(wb_box)

        #---------------------------------YOLO ON/OFF toggle---------------------------------
        self.yolo_enabled = True
        self.yolo_toggle = QCheckBox("Enable YOLO Detection")
        self.yolo_toggle.setChecked(True)
        self.yolo_toggle.stateChanged.connect(self.on_yolo_toggle)
        sub.addWidget(self.yolo_toggle)
        
        

        #---------------------------------Capture + Reset buttons---------------------------------
        btn_layout = QHBoxLayout()
        self.cap_btn = QPushButton("Capture")
        self.cap_btn.clicked.connect(self.on_capture_clicked)
        self.reset_btn = QPushButton("Reset Settings")
        self.reset_btn.clicked.connect(self.reset_camera_settings)
        self.record_btn = QPushButton("Start Recording")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self.on_record_toggled)
        btn_layout.addWidget(self.cap_btn)
        btn_layout.addWidget(self.record_btn)
        btn_layout.addWidget(self.reset_btn)
        sub.addLayout(btn_layout)

        

        #---------------------------------Sensors tab boxes---------------------------------
        self.sensor_labels = {}
        self.sensor_boxes = []
        self.sensor_dict = {
            "Front Unit IMU":       ("/top_imu/data",         Imu,                lambda m:
                                     f"ori=({m.orientation.x:.2f},{m.orientation.y:.2f},{m.orientation.z:.2f})"),
            "Rear Unit IMU":        ("/bottom_imu/data",      Imu,                lambda m:
                                     f"ori=({m.orientation.x:.2f},{m.orientation.y:.2f},{m.orientation.z:.2f})"),
            "Front TF Luna":        ("/top_tf_luna/range",    Range,              lambda m: f"{m.range:.3f} m"),
            "Rear TF Luna":         ("/bottom_tf_luna/range", Range,              lambda m: f"{m.range:.3f} m"),
            "Left ToF Sensor":      ("/left_tof/distance",    Float32,            lambda m: m.data),
            "Right ToF Sensor":     ("/right_tof/distance",   Float32,            lambda m: m.data),
            "LED Ring Brightness":     ("/led_ring_brightness",   Float32,            lambda m: m.data),
        }
        for name, (topic, _, _) in self.sensor_dict.items():
            gb = QGroupBox()
            gb.setStyleSheet(
                "QGroupBox {"
                "  border: 1px solid grey;"
                "  border-radius: 4px;"
                "  margin: 4px;"
                "}"
            )
            gb.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            tile = QWidget()
            hl = QHBoxLayout(tile)
            title = QLabel(f"<b>{name}</b>")
            value = QLabel("N/A")
            hl.addWidget(title)
            hl.addStretch()
            hl.addWidget(value)
            gb.setLayout(hl)
            self.sensor_boxes.append((name, gb))
            self.sensor_labels[name] = value

        # initial grid positioning
        self.position_sensor_grid()

        # Start ROS2 and one subscriber per sensor
        rclpy.init(args=None)
        self.sensor_nodes = []
        for name, (topic, msg_type, parser) in self.sensor_dict.items():
            node = SensorSubscriber(
                name,
                topic,
                msg_type,
                parser,
                self.update_sensor_value
            )
            self.sensor_nodes.append(node)

        # self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        # self.ros_thread.start()
        

        

        # 1) Pick off just your two actuator specs from the big dict
        self.dev_sensors = {
            "Actuator Extension":   ("/actuator/extension_array", Float32MultiArray,
                                     lambda m: ", ".join(f"{x:.2f}" for x in m.data)),
            "Actuator Force":       ("/actuator/force_array",     Float32MultiArray,
                                     lambda m: ", ".join(f"{x:.2f}" for x in m.data)),
        }

        self.dev_boxes  = []
        self.dev_labels = {}
        for name, (topic, msg_t, parser) in self.dev_sensors.items():
            gb = QGroupBox()
            gb.setStyleSheet(
                "QGroupBox {"
                "  border: 1px solid grey;"
                "  border-radius: 4px;"
                "  margin: 4px;"
                "}"
            )
            gb.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            h = QHBoxLayout(gb)
            title = QLabel(f"<b>{name}</b>")
            value = QLabel("N/A")
            h.addWidget(title)
            h.addStretch()
            h.addWidget(value)
            gb.setLayout(h)

            self.dev_boxes.append((name, gb))
            self.dev_labels[name] = value   

        # call it now and also hook into resizeEvent
        self.position_dev_grid()
        #self.resizeEvent = lambda ev, orig=self.resizeEvent: (orig(ev), position_dev_grid())

        # 4) Subscribe & append to the same sensor_nodes list
        for name, (topic, msg_t, parser) in self.dev_sensors.items():
            node = SensorSubscriber(
                name, topic, msg_t, parser,
                lambda s, v, lbl=self.dev_labels[name]: lbl.setText(str(v))
            )
            self.sensor_nodes.append(node)
        




        # ————— ROS2 init + subscriptions for controller data —————
        self.ros_node = rclpy.create_node('gui_capture_node')
        self.sensor_nodes.append(self.ros_node)
        

        # initialize attributes from controller
        self.current_servo0 = 0.0
        self.current_servo1 = 90.0
        self.current_brightness = 25.0

        self.ros_node.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self._on_servo_update,
            10
        )
        self.ros_node.create_subscription(
            Float32,
            'led_brightness',
            self._on_led_update,
            10
        )

        self.captureFinished.connect(lambda: self.cap_btn.setEnabled(True))
        self.spin_thread = threading.Thread(target=self._ros_spin_all, daemon=True)
        self.spin_thread.start()
        #threading.Thread(target=self._spin_ros, daemon=True).start()

    def _ros_spin_all(self):
        while rclpy.ok():
            for node in self.sensor_nodes:
                rclpy.spin_once(node, timeout_sec=0.1)

    def on_yolo_toggle(self, state):
            # Qt.Checked == enabled, anything else == off
            self.yolo_enabled = (state == Qt.Checked)

    def update_frame(self):
        now = time.time()

        # record this frame’s timestamp, and drop any older than 1 s ago
        self._timestamps.append(now)
        while self._timestamps and self._timestamps[0] < now - 1.0:
            self._timestamps.popleft()

        frame = picam2.capture_array()
        if frame is None:
            return

        # strip alpha, resize
        if frame.ndim == 3 and frame.shape[2] == 4:
            # assume OpenCV sees this as BGRA: [B,G,R,A]
            frame = frame[:, :, :3]
        else:
            frame = frame
        frame = cv2.resize(frame, self.resize_dims)


        if self.yolo_enabled:
            # run and draw YOLO boxes
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.yolo_model(rgb, verbose=False)[0]
            for box in results.boxes:
                conf = float(box.conf)
                if conf < self.conf_threshold:
                    continue
                cls = int(box.cls)
                x1,y1,x2,y2 = map(int, box.xyxy.cpu().numpy().squeeze())
                cv2.rectangle(frame, (x1,y1),(x2,y2), (255,0,0), 2)
                cv2.putText(frame,
                            f"{self.yolo_model.names[cls]} {conf:.2f}",
                            (x1,y1-2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                
        # Video Capture 
        if getattr(self, 'recording', False):
            # write BGR frame to video
            self.video_writer.write(frame)


        # display either raw or annotated frame
        rgb2 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = rgb2.shape[:2]
        qimg = QImage(rgb2.data, w, h, 3*w, QImage.Format_RGB888)
        self.preview_label.setPixmap(QPixmap.fromImage(qimg))

        fps = len(self._timestamps)
        print(f"[FPS] {fps:.1f}")

    def reset_camera_settings(self):
            # restore each slider to its original default
            for slider in (self.sliderBrightness,
                        self.sliderContrast,
                        self.sliderExposureValue,
                        self.sliderSaturation,
                        self.sliderSharpness):
                slider.reset()
            # reset white balance back to Auto
            for btn in self.wb_group.buttons():
                if btn.text() == "Auto":
                    btn.setChecked(True)
                    picam2.set_controls({"AwbMode": controls.AwbModeEnum.Auto})
                    break
    def on_record_toggled(self, checked):
        if checked:
            # start recording
            ts = time.strftime("%Y-%m-%d_%H-%M-%S")
            path = os.path.join(video_folder, f"video_{ts}.mp4")
            # e.g. use H264 or MJPG; adjust fourcc/fps to taste
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                path,
                fourcc,
                5,  # match your preview fps
                self.resize_dims
            )
            self.recording = True
            self.record_btn.setText("Stop Recording")
        else:
            # stop recording
            self.recording = False
            self.video_writer.release()
            self.record_btn.setText("Start Recording")

    def _on_servo_update(self, msg: Float32MultiArray):
        self.current_servo0, self.current_servo1 = msg.data

    def _on_led_update(self, msg: Float32):
        self.current_brightness = msg.data

    def update_sensor_value(self, sensor_name: str, val: float):
        label = self.sensor_labels.get(sensor_name)
        if label:
            label.setText(f"{val:.3f}")

    def position_sensor_grid(self):
        n = len(self.sensor_boxes)
        if n == 0:
            return
        w = self.tabSensors.width() or 1
        desired = 200
        cols = max(1, min(n, w // desired))
        while self.sensor_layout.count():
            item = self.sensor_layout.takeAt(0)
            wdg = item.widget()
            if wdg:
                wdg.setParent(None)
        for idx, (_, box) in enumerate(self.sensor_boxes):
            r, c = divmod(idx, cols)
            self.sensor_layout.addWidget(box, r, c)

    def position_dev_grid(self):
            n = len(self.dev_boxes)
            if n == 0:
                return
            w = self.tabDev.width() or 1
            desired = 200
            cols = max(1, min(n, w // desired))
            # clear
            while self.dev_layout.count():
                item = self.dev_layout.takeAt(0)
                wdg = item.widget()
                if wdg:
                    wdg.setParent(None)
            # populate
            for idx, (_, box) in enumerate(self.dev_boxes):
                r, c = divmod(idx, cols)
                self.dev_layout.addWidget(box, r, c)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.position_sensor_grid()
        self.position_dev_grid()

    def closeEvent(self, event):
        rclpy.shutdown()
        super().closeEvent(event)
        if getattr(self, 'recording', False):
            self.video_writer.release()


    def on_wb_mode(self, btn):
        enum = getattr(controls.AwbModeEnum, btn.text())
        picam2.set_controls({"AwbMode": enum})

    def on_capture_clicked(self):
        if capture_lock.locked():
            return
        self.cap_btn.setEnabled(False)
        threading.Thread(target=self.capture_worker, daemon=True).start()

    def capture_worker(self):
        global focus_position
        if not capture_lock.acquire(False):
            return
        try:
            ts = time.strftime("%Y-%m-%d_%H-%M-%S")
            path = os.path.join(image_folder, f"image_{ts}.jpg")
            req = picam2.capture_request()
            req.save("main", path)
            req.release()
            img = Image.open(path)
            exif = piexif.load(img.info.get("exif", piexif.dump({})))
            meta = (
                f"LED: {self.current_brightness:.0f}%, "
                f"Servo0: {self.current_servo0:.0f}, "
                f"Servo1: {self.current_servo1:.0f}, "
                f"Focus: {focus_position:.1f}"
            )
            exif["Exif"][piexif.ExifIFD.UserComment] = b"\x00\x00\x00\x00" + meta.encode()
            piexif.insert(piexif.dump(exif), path)
            print(f"Captured {path}")
        finally:
            capture_lock.release()
            self.captureFinished.emit()

class SensorSubscriber(Node):
    def __init__(self, sensor_name: str, topic: str,
                 msg_type, parser: callable, callback: callable):
        super().__init__(f"sensor_sub_{sensor_name.replace(' ', '_')}")
        self.sensor_name = sensor_name
        self.parser = parser
        self.callback = callback
        self.create_subscription(
            msg_type,
            topic,
            self._on_message,
            10
        )
    def _on_message(self, msg):
        try:
            val = self.parser(msg)
            self.callback(self.sensor_name, val)
        except Exception as e:
            self.get_logger().warn(f"Parse error in {self.sensor_name}: {e}")

class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pi Camera Controller")
        self.setCentralWidget(MainWidget())
        self.showMaximized()
    def closeEvent(self, ev):
        rclpy.shutdown()
        picam2.stop()
        super().closeEvent(ev)

# GUI window size helper, cleanup, joystick focus thread
def setWindowSize(app, win):
    screen_geom = app.primaryScreen().availableGeometry()
    half_width = screen_geom.width() // 2
    full_height = screen_geom.height()
    win.setGeometry(
        screen_geom.x(), screen_geom.y(),
        half_width, full_height
    )

def cleanup():
    stop_event.set()
    time.sleep(0.1)
    pygame.quit()
    picam2.stop()

def joystick_loop():
    global focus_position
    while not stop_event.is_set():
        pygame.event.pump()
        l2 = joystick.get_axis(2)
        r2 = joystick.get_axis(5)
        if l2 > 0.5:
            focus_position = max(focus_position - focus_step, focus_min)
            picam2.set_controls({"LensPosition": focus_position})
        if r2 > 0.5:
            focus_position = min(focus_position + focus_step, focus_max)
            picam2.set_controls({"LensPosition": focus_position})
        time.sleep(0.05)

#threading.Thread(target=joystick_loop, daemon=True).start()
signal.signal(signal.SIGINT, lambda *args: (cleanup(), sys.exit(0)))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.aboutToQuit.connect(cleanup)
    win = MainWidget()
    setWindowSize(app, win)
    win.show()
    sys.exit(app.exec_())
