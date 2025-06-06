from machine import Pin, PWM, UART, Timer, I2C
import time
import utime
import serial
import queue
import numpy as np
from math import pi, cos, sin
import Pi_pico.pins as pins 
import asyncio
from KalmanFilter import VerticalKalmanFilter, HorizontalKalmanFilter
from sensors import TFLunaI2C, VL53L1XToF, IMU, AccPot, INA
import logging
from motor_control import MotorControl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu, Range

logging.basicConfig(
    level=logging.INFO,  # or DEBUG during development
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)

logger = logging.getLogger(__name__)

pico = serial.Serial("/dev/ttyTHS1", 115200)

class ControllerNode:
    def __init__(self):
        super().__init__()
        self.force = 100
        self.increase = 0
        self.prev_accel_z = 0.0
        self.current = 0
        self.mode = 1  # Define mode as instance variable
        self.motor_command = None
        self.dt = 0.01
        self.uart = UART(0, 115200)
        self.uart.init(115200, bits=8, parity=None, stop=1)
        ###### PID intialisation #########
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.05

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = utime.ticks_ms()

        self.i2c_0 = [pins.IMU_0_0, pins.IMU_0_1]
        self.i2c_1 = [pins.IMU_1_0, pins.IMU_1_1]

        
        ######### Motor Init #########
        self.motor = MotorControl()
        ######### Sensor Init ############
        i2c1 = I2C(0, scl=Pin(self.i2c_0[1]), sda=Pin(self.i2c_0[0]))
        i2c2 = I2C(1, scl=Pin(self.i2c_1[1]), sda=Pin(self.i2c_1[0]))

        # --
        self.left_tof_sensor = VL53L1XToF(i2c1)
        self.right_tof_sensor = VL53L1XToF(i2c2)
 
        # --
        self.top_sensor = TFLunaI2C(i2c1)
        self.bottom_sensor = TFLunaI2C(i2c2)
        
        # --
        self.imu1 = IMU(i2c1)
        self.imu2 = IMU(i2c2)

        self.imu1.calibrate()
        self.imu2.calibrate()

        self.ina1 = INA(i2c1)
        self.ina2 = INA(i2c2)
        
        # - 
        self.pots = [sensors.AccPot(i) for i in range(8)]
        for pot in self.pots:
            pot.calibrate()   

        #--------- Creating ROS topic Publishers-------
        self.top_imu_pub = self.create_publisher(
            Imu,                # message type
            '/top_imu/data',    # topic name
            10                  # QoS history depth
        )
        self.bottom_imu_pub = self.create_publisher(
            Imu,                # message type
            '/bottom_imu/data', # topic name
            10                  # QoS history depth
        )

        self.top_tf_pub = self.create_publisher(
            Range,                     # message type
            '/top_tf_luna/range',      # topic name
            10                         # QoS
        )
        self.bottom_tf_pub = self.create_publisher(
            Range,                     # message type
            '/bottom_tf_luna/range',   # topic name
            10                         # QoS
        )

        self.left_tof_pub = self.create_publisher(
            Float32,                # message type
            '/left_tof/distance',   # topic name
            10                      # QoS
        )
        self.right_tof_pub = self.create_publisher(
            Float32,                # message type
            '/right_tof/distance',  # topic name
            10                      # QoS
        )

        self.actuators_extension_pub = self.create_publisher(
            Float32MultiArray,             # message type
            '/actuator/extension_array',   # topic name
            10                             # QoS
        )
        self.actuators_force_pub = self.create_publisher(
            Float32MultiArray,         # message type
            '/actuator/force_array',   # topic name
            10                         # QoS
        )
        
        ########## Kalman Filter Init ############
        self.sensor_errors = [0.02, 0.02]
        self.imu_error = 0.01
        self.initial_distances=[self.top_sensor.read_distance(), self.bottom_sensor.read_distance()]
        self.Vkf = VerticalKalmanFilter(self.dt, self.imu_error, self.sensor_errors, self.initial_distances)
        self.obstacle_positions = []
        self.section_positions = {1: 0, 2: -0.2, 3: -0.8, 4: -1}
        
        self.actuators = {
            1: {"extend": Pin(2, Pin.OUT), "retract": Pin(3, Pin.OUT)},
            2: {"extend": Pin(4, Pin.OUT), "retract": Pin(5, Pin.OUT)},
            3: {"extend": Pin(6, Pin.OUT), "retract": Pin(7, Pin.OUT)},
            4: {"extend": Pin(8, Pin.OUT), "retract": Pin(9, Pin.OUT)},
        }

        self.extenders = {
            1: {"extend": Pin(10, Pin.OUT), "retract": Pin(11, Pin.OUT)},
            2: {"extend": Pin(12, Pin.OUT), "retract": Pin(13, Pin.OUT)},
        }

        for act in self.actuators.values():
            act["extend"].value(0)
            act["retract"].value(0)

        for ext in self.extenders.values():
            ext["extend"].value(0)
            ext["retract"].value(0)

        self.continuum = [
            [pins.CONT_1_1, pins.CONT_1_2, pins.ENA_CONT_1],
            [pins.CONT_2_1, pins.CONT_2_2, pins.ENA_CONT_2]
        ]

        asyncio.create_task(self._async_init())

    async def _async_init(self):
        await asyncio.sleep(0.0)
        
    async def Slippage(self, threshold=3.0):
        accel1 = self.imu1.get_accel()
        accel_1z = accel1['z']
        accel2 = self.imu2.get_accel()
        accel_2z = accel2['z']

        if not hasattr(self, 'prev_accel_z1'):
            self.prev_accel_z1 = accel_1z
            self.prev_accel_z2 = accel_2z
            return False

        drop1 = self.prev_accel_z1 - accel_1z
        drop2 = self.prev_accel_z2 - accel_2z

        self.prev_accel_z1 = accel_1z
        self.prev_accel_z2 = accel_2z

        # Only consider positive drops (acceleration downward)
        drop1 = drop1 if drop1 > 0 else 0
        drop2 = drop2 if drop2 > 0 else 0

        return drop1 > threshold and drop2 > threshold
    
    async def anchor_tilt(self, threshold=0.09):
        heading1, roll1, pitch1 = self.imu1.get_accel()
        heading2, roll2, pitch2 = self.imu2.get_accel()
        prev_pitch1 = pitch1
        prev_pitch2 = pitch2
        await asyncio.sleep(0.1)
        heading1, roll1, pitch1 = self.imu1.get_accel()
        heading2, roll2, pitch2 = self.imu2.get_accel()
        delta_1 = pitch1 - prev_pitch1
        delta_2 = pitch2 - prev_pitch2
        if delta_1 > delta_2:
            heading, roll, pitch = heading1, roll1, pitch1
            num = 1
        else:
            heading, roll, pitch = heading2, roll2, pitch2
            num = 2
        
        if abs(heading) > threshold or abs(roll) > threshold or abs(pitch)>threshold:
            logger.info(f"Anchor inching unit tilt detected")
            return True, num
        else:
            return False, 3
        

    async def control_holder(self, actuator_num, direction, dist):
        if actuator_num not in self.actuators:
            logger.error(f"Invalid actuator: {actuator_num}")
            return

        actuator = self.actuators[actuator_num]
        try:
            while True:
                current_pos = self.pot.read_pot()  # Read live potentiometer value

                if direction == "extend":
                    if current_pos >= dist:
                        break
                    actuator["extend"].value(1)
                    actuator["retract"].value(0)

                elif direction == "retract":
                    if current_pos <= dist:
                        break
                    actuator["extend"].value(0)
                    actuator["retract"].value(1)

                else:
                    logger.error(f"Invalid direction: {direction}")
                    break  # exit loop on error

                await asyncio.sleep(0.1)  # Yield to event loop

        finally:
            # Always stop actuators when done
            actuator["extend"].value(0)
            actuator["retract"].value(0)
    

    async def control_extender(self, extender_num, direction, dist):
        if extender_num not in self.extenders:
            logger.error(f"Invalid extender: {extender_num}")
            return False

        extender = self.extenders[extender_num]
        buffer = ""
        dist1 = dist2 = 0  # Default fallback values

        if direction not in ["extend", "retract"]:
            if direction == "stop":
                extender["extend"].value(0)
                extender["retract"].value(0)
                return True
            else:
                logger.error(f"Invalid direction: {direction}")
                return False

        try:
            while True:
                # Read and buffer UART data
                if self.uart.any():
                    try:
                        data = self.uart.read()
                        if data:
                            buffer += data.decode()
                    except Exception as e:
                        logger.error(f"UART read/decode error: {e}")

                # Extract and process sensor data
                while "<SENSOR>" in buffer and "</SENSOR>" in buffer:
                    start = buffer.find("<SENSOR>") + len("<SENSOR>")
                    end = buffer.find("</SENSOR>")
                    payload = buffer[start:end]
                    try:
                        dist1, dist2 = map(int, payload.split(","))
                        logger.debug(f"[ToF] Sensor 1: {dist1} mm, Sensor 2: {dist2} mm")
                    except Exception as e:
                        logger.error(f"Sensor data parse error: {e}")
                    buffer = buffer[end + len("</SENSOR>"):]

                # Use correct sensor reading
                current_pos = dist1 if extender_num == 1 else dist2

                # Movement logic
                if direction == "extend":
                    if current_pos >= dist:
                        break
                    extender["extend"].value(1)
                    extender["retract"].value(0)

                elif direction == "retract":
                    if current_pos <= dist:
                        break
                    extender["extend"].value(0)
                    extender["retract"].value(1)

                # Safety: crash detection
                if await self.crash_detect():
                    logger.warning(f"Near Crash detected on extender {extender_num}. Stopping and clearing UART.")
                    extender["extend"].value(0)
                    extender["retract"].value(0)

                    try:
                        while self.uart.any():
                            self.uart.read()
                    except Exception as e:
                        logger.error(f"UART clear failed: {e}")
                    return False

                await asyncio.sleep(0.1)

        finally:
            extender["extend"].value(0)
            extender["retract"].value(0)

        return True




    async def anchor(self, unit, threshold):
        if unit == 1:
            sections = [1, 2]
        elif unit == 2:
            sections = [3, 4]
        else:
            logger.error(f"Invalid unit: {unit}")
            return
        while self.current < threshold:
            for i in sections:
                self.actuators[i]["extend"].value(1)
                self.actuators[i]["retract"].value(0)
        await asyncio.sleep(0.01)

    def Force(self, force):
        return force

    async def inching_semi_auto(self, dir):
        async def safe_extender(extender_num, direction, dist):
            success = await self.control_extender(extender_num, direction, dist)
            if not success:
                logger.warning(f"Aborting sequence: Crash or failure on extender {extender_num}.")
                return False
            return True

        if dir == "up":
            # Phase 1: Release holder 1
            await self.control_holder(1, 'retract', 0.02)
            await asyncio.sleep(1.0)

            # Phase 2: Extend extender 1
            if not await safe_extender(1, 'extend', 0.1):
                return

            await asyncio.sleep(1.0)
            await self.control_holder(1, 'extend', 0.02)
            await asyncio.sleep(1.0)

            # Phase 3: Release holders 2 and 3
            await self.control_holder(2, 'retract', 0.02)
            await self.control_holder(3, 'retract', 0.02)
            await asyncio.sleep(1.0)

            # Phase 4: Retract extender 1, extend extender 2
            if not await safe_extender(1, 'retract', 0.02):
                return
            if not await safe_extender(2, 'extend', 0.1):
                return

            await asyncio.sleep(1.0)
            await self.control_holder(2, 'extend', 0.02)
            await self.control_holder(3, 'extend', 0.02)
            await asyncio.sleep(1.0)

            # Phase 5: Release holder 4
            await self.control_holder(4, 'retract', 0.02)
            await asyncio.sleep(1.0)

            # Phase 6: Retract extender 2
            if not await safe_extender(2, 'retract', 0.02):
                return

            await asyncio.sleep(1.0)
            await self.control_holder(4, 'extend', 0.02)
            await asyncio.sleep(2.0)

        elif dir == "down":
            # Phase 1: Release holder 4
            await self.control_holder(4, 'retract', 0.02)
            await asyncio.sleep(1.0)

            # Phase 2: Extend extender 2
            if not await safe_extender(2, 'extend', 0.1):
                return

            await asyncio.sleep(1.0)
            await self.control_holder(4, 'extend', 0.02)
            await asyncio.sleep(1.0)

            # Phase 3: Release holders 2 and 3
            await self.control_holder(2, 'retract', 0.02)
            await self.control_holder(3, 'retract', 0.02)
            await asyncio.sleep(1.0)

            # Phase 4: Retract extender 2, extend extender 1
            if not await safe_extender(2, 'retract', 0.02):
                return
            if not await safe_extender(1, 'extend', 0.1):
                return

            await asyncio.sleep(1.0)
            await self.control_holder(2, 'extend', 0.02)
            await self.control_holder(3, 'extend', 0.02)
            await asyncio.sleep(1.0)

            # Phase 5: Release holder 1
            await self.control_holder(1, 'retract', 0.02)
            await asyncio.sleep(1.0)

            # Phase 6: Retract extender 1
            if not await safe_extender(1, 'retract', 0.02):
                return

            await asyncio.sleep(1.0)
            await self.control_holder(1, 'extend', 0.02)
            await asyncio.sleep(2.0)

        else:
            logger.error("Invalid direction given to inching_semi_auto. Use 'up' or 'down'.")
            
    async def manual_bend(self, dir):
        heading, roll, pitch = self.imu1.get_accel()
        current_angle = abs(pitch)
        if dir == "right" :
            angle = current_angle + 1
        else:
            angle = current_angle - 1
        while abs(angle) <= 95:
            logger.debug(f"Current Angle: {angle}")
            await self.continuum_PID(angle)
            angle += 1
        while True:
            if await self.Slippage():
                await self.anchor(1, self.Force(self.increase))
                await self.anchor(2, self.Force(self.increase))
                logger.warning("Slippage detected. Anchoring. Increasing force threshold.")
                self.increase += 5
                break
            tilt, num = await self.anchor_tilt()
            if tilt:
                self.continuum_PID((abs(pitch)-1)*np.sign(pitch))
                await self.anchor(num, self.Force(self.increase))
                logger.warning("Anchor inching unit tilt detected. Reducing bend and inchreasing force threshold before attempting again")
                self.increase += 5
                break
            await asyncio.sleep(0.05)
            await asyncio.sleep(0.01)

    async def continuum_PID(self, target_angle):
        # Get sensor readings
        heading, roll, pitch = self.imu1.get_accel()

        error = target_angle - abs(pitch)

        current_time = utime.ticks_ms()
        dt = utime.ticks_diff(current_time, self.last_time) / 1000.0  # seconds

        if dt <= 0:
            dt = 0.01  # Avoid division by zero or negative dt

        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Clamp output to input value range (0 to 1)
        output = int(max(min(output, 1.0), -1.0))

        # Apply PID output to actuators' PWM signals - see motor_control.py
        self.motor.set_speed(output)

        self.last_error = error
        self.last_time = current_time
        while True:
            if await self.Slippage():
                await self.anchor(1, self.Force(self.increase))
                await self.anchor(2, self.Force(self.increase))
                logger.warning("Slippage detected. Anchoring. Increasing force threshold.")
                self.increase += 5
                break
            tilt, num = await self.anchor_tilt()
            if tilt:
                self.continuum_PID((abs(pitch)-1)*np.sign(pitch))
                await self.anchor(num, self.Force(self.increase))
                logger.warning("Anchor inching unit tilt detected. Reducing bend and inchreasing force threshold before attempting again")
                self.increase += 5
                break
            await asyncio.sleep(0.05)

        await asyncio.sleep(0.1)
            
    def detect_obstacle(self, threshold=200):  # threshold in mm
        dist_left = self.left_tof_sensor.read_distance()
        dist_right = self.right_tof_sensor.read_distance()
        
        # Only average if both distances are valid
        if dist_left is not None and dist_right is not None:
            dist = (dist_left + dist_right) / 2
        elif dist_left is not None:
            dist = dist_left
        elif dist_right is not None:
            dist = dist_right
        else:
            return []  # no valid readings
        
        obstacles = []
        pos, _ = self.Vkf.get_state()  # get current estimated position
        
        x1 = pos[0, 0]  # extract x1 from 2x1 matrix
        
        if dist < threshold:
            obstacle_pos = x1  # obstacle position in mm
            obstacle_width = dist
            obstacles.append(obstacle_pos,obstacle_width)
        
        return obstacles



async def clear_uart(controller):
    while controller.uart.any() != 0:
        controller.uart.readline()
    await asyncio.sleep(0.0)


# main function


async def control(controller):
    uart_message = controller.uart.readline()
    if not uart_message:
        await asyncio.sleep(0.25)
        return

    try:
        uart_message = uart_message.decode("ascii").strip().upper()
    except UnicodeDecodeError:
        logger.warning("Failed to decode UART message")
        return

    logger.debug(f"Full UART: {uart_message}")
    handled_any = False

    # Find all <control>...</control> blocks
    control_blocks = re.findall(r"<control>(.*?)</control>", uart_message, re.DOTALL)

    for control_data in control_blocks:
        control_data = control_data.strip()
        logger.debug(f"Processing control block: {control_data}")
        handled = False

        # Mode toggle
        if "START" in control_data:
            logger.info("Switching control mode")
            controller.mode = 1 - controller.mode
            handled = True

        holder_map = {"X": 4, "O": 3, "SQU": 2, "TRI": 1}

        # Manual Mode
        if controller.mode == 1:
            for btn, num in holder_map.items():
                if btn in control_data:
                    if "R2" in control_data:
                        logger.info(f"holder {num} extend")
                        await controller.control_holder(num, 'extend')
                        handled = True
                        break
                    elif "L2" in control_data:
                        logger.info(f"holder {num} retract")
                        await controller.control_holder(num, 'retract')
                        handled = True
                        break

            if "LSUP" in control_data:
                logger.info("Incher 1 extend")
                await controller.control_extender(1, 'extend')
                handled = True
            elif "LSDOWN" in control_data:
                logger.info("Incher 1 retract")
                await controller.control_extender(1, 'retract')
                handled = True
            elif "RSUP" in control_data:
                logger.info("Incher 2 extend")
                await controller.control_extender(2, 'extend')
                handled = True
            elif "RSDOWN" in control_data:
                logger.info("Incher 2 retract")
                await controller.control_extender(2, 'retract')
                handled = True
            elif "R1" in control_data:
                logger.info("Turning Right")
                await controller.manual_bend('right')
                handled = True
            elif "L1" in control_data:
                logger.info("Turning Left")
                await controller.manual_bend('left')
                handled = True

        # Semi-Auto Mode
        elif controller.mode == 0:
            if "LSUP" in control_data:
                logger.info("Inching up")
                await controller.inching_semi_auto('up')
                handled = True
            elif "LSDOWN" in control_data:
                logger.info("Inching down")
                await controller.inching_semi_auto('down')
                handled = True
            elif "X" in control_data:
                logger.info("Turn to negative 90")
                await controller.continuum_PID("left", 90)
                handled = True
            elif "O" in control_data:
                logger.info("Turn to positive 45")
                await controller.continuum_PID("right", 45)
                handled = True
            elif "SQU" in control_data and "L2" in control_data:
                logger.info("Turn to negative 45")
                await controller.continuum_PID("left", 45)
                handled = True
            elif "TRI" in control_data:
                logger.info("Turn to positive 90")
                await controller.continuum_PID("right", 90)
                handled = True

        if handled:
            # Remove all processed <control>...</control> blocks from the original message
            uart_message = re.sub(r"<control>.*?</control>", "", uart_message, flags=re.DOTALL).strip()

    if uart_message:
        logger.debug(f"Remaining message for other handlers: {uart_message}")

    await asyncio.sleep(0.1)


async def sensors(controller):
    buffer = ""
    dist1, dist2 = 0, 0  # Default ToF sensor values
    period_ms = 100  # 10 Hz = 100ms per loop

    while True:
        loop_start = time.ticks_ms()  # Timestamp at loop start

        try:
            # Detect and track new obstacles
            obstacles = controller.detect_obstacle()
            new_obs = [obs for obs in obstacles if obs not in controller.obstacle_positions]
            for obs in new_obs:
                logger.info(f"New obstacle at {obs[0]} mm")
                controller.obstacle_positions.append(obs)

            # Retract holders if section passed an obstacle
            for section, pos in controller.section_positions.items():
                for obs_pos, obs_width in controller.obstacle_positions:
                    if pos == obs_pos:
                        logger.info(f"Section {section} passing obstacle at {obs_pos} mm, retracting...")
                        await controller.control_holder(section, 'retract', obs_width)

            # IMU read and prediction
            accel1 = controller.imu1.read()
            accel2 = controller.imu2.read()
            if accel1 is None or accel2 is None:
                continue
            u = np.array([[accel1['z']], [accel2['z']]])
            controller.Vkf.predict(u)

            # Sensor read and Kalman update
            z1 = controller.top_sensor.read_distance()
            z2 = controller.bottom_sensor.read_distance()
            if z1 is None or z2 is None:
                continue
            controller.Vkf.update(np.array([[z1], [z2]]))

            # Get filtered state
            pos, vel = controller.Vkf.get_state()
            x1 = pos[0, 0]
            x2 = pos[1, 0]

            logger.debug(f"Estimated position: {x1:.2f} mm, velocity: {vel:.2f} mm/s")

            # UART parsing for extra ToF sensors
            if controller.uart.any():
                buffer += controller.uart.read().decode()
                while "<SENSOR>" in buffer and "</SENSOR>" in buffer:
                    start = buffer.find("<SENSOR>") + len("<SENSOR>")
                    end = buffer.find("</SENSOR>")
                    payload = buffer[start:end]
                    try:
                        dist1, dist2 = map(int, payload.split(","))
                        logger.debug(f"[ToF] Sensor 1: {dist1} mm, Sensor 2: {dist2} mm")
                    except Exception as e:
                        logger.error(f"Sensor data parse error: {e}")
                    buffer = buffer[end + len("</SENSOR>"):]

            # Update section positions
            controller.section_positions = {
                1: x1,
                2: x1 - dist1,
                3: x2 + dist2,
                4: x2
            }

        except Exception as e:
            logger.error(f"Sensor loop error: {e}")
 
        # Maintain 10Hz loop rate
        elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
        delay = max(0, period_ms - elapsed)
        await asyncio.sleep(delay / 1000)

async def serial_messages(controller):
    imu1_data = controller.imu1.read()
    imu2_data = controller.imu2.read()
    top_dist = controller.top_sensor.read_distance()
    bottom_dist = controller.bottom_sensor.read_distance()
    left_dist = controller.left_tof_sensor.read_distance()
    right_dist = controller.right_tof_sensor.read_distance()
    extensions = [pot.read_pot() for pot in controller.pots]
    forces = [controller.ina1.read(), controller.ina2.read()]
    
    # IMU publish
    now = controller.get_clock().now().to_msg()
    for data, pub, frame in (
            (imu1_data, controller.top_imu_pub,    'top_imu'),
            (imu2_data, controller.bottom_imu_pub, 'bottom_imu')):
                msg = Imu()
                msg.header.stamp = now
                msg.header.frame_id = frame
            # orientation quaternion
                qx, qy, qz, qw = data['quaternion']
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                msg.orientation.w = qw
            # no angular velocity on BNO055 here
                msg.angular_velocity.x = 0.0
                msg.angular_velocity.y = 0.0
                msg.angular_velocity.z = 0.0
            # linear acceleration
                ax, ay, az = data['accel']
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az
                pub.publish(msg)

    # TF Luna Publish 
    for dist, pub, frame in (
            (top_dist,    controller.top_tf_pub,    'top_tf_luna'),
            (bottom_dist, controller.bottom_tf_pub, 'bottom_tf_luna')
        ):
        if dist is None:
            continue
        r = Range()
        r.header.stamp = now
        r.header.frame_id = frame
        r.range     = float(dist)
        r.min_range = 0.03   # TFLuna min
        r.max_range = 8.0    # TFLuna max
        pub.publish(r)
    
    # ToF sensor publisher
    if left_dist is not None:
        controller.left_tof_pub.publish(Float32(data=float(left_dist)))
    if right_dist is not None:
        controller.right_tof_pub.publish(Float32(data=float(right_dist)))

    # Actuator extension and force publisher
    ext_msg = Float32MultiArray(data=[float(e) for e in extensions])
    force_msg = Float32MultiArray(data=[float(f) for f in forces])

    controller.actuators_extension_pub.publish(ext_msg)
    controller.actuators_force_pub.publish(force_msg)



