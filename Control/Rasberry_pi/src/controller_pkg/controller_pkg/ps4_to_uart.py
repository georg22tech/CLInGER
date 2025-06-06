from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node
from custom_interface_package.msg import ControllerTranslation
import serial
import queue

pico = serial.Serial("/dev/ttyTHS1", 115200)

class PS4TranslatorNode(Node):
    def __init__(self):
        super().__init__("ps4_controller_translator")

        # subscriber to read Joy Messages
        self.ps4_controller_translator = self.create_subscription(
            msg_type=Joy,
            topic="/joy",
            callback=self.ps4_translator_callback,
            qos_profile=1,
        )

        # publisher to publish translated controller inputs
        self.translator_publisher = self.create_publisher(
            msg_type=ControllerTranslation,
            topic="/xbox_controller_topic",
            qos_profile=1,
        )

        self.controller_translation = ControllerTranslation()

    def ps4_translator_callback(self, msg: Joy):
        if (round(msg.axes[0]) == 1) & (round(msg.axes[1] == 0)):
            self.controller_translation.message = "LSLeft"
            self.get_logger().info("Left")
            self.translator_publisher.publish(self.controller_translation)
        elif (round(msg.axes[0]) == -1) & (round(msg.axes[1]) == 0):
            self.controller_translation.message = "LSRight"
            self.get_logger().info("Right")
            self.translator_publisher.publish(self.controller_translation)
        elif (round(msg.axes[0]) == 0) & (round(msg.axes[1]) == 1):
            self.controller_translation.message = "LSUp"
            self.get_logger().info("Up")
            self.translator_publisher.publish(self.controller_translation)
        elif (round(msg.axes[0]) == 0) & (round(msg.axes[1]) == -1):
            self.controller_translation.message = "LSDown"
            self.get_logger().info("Down")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[6]) == 1:
            self.controller_translation.message = "L2"
            self.get_logger().info("LT")
            self.translator_publisher.publish(self.controller_translation)

        if(round(msg.axes[4]) == 1):
            self.controller_translation.message = 'RsUp'
            self.get_logger().info('RsUp')
            self.translator_publisher.publish(self.controller_translation)
        elif(round(msg.axes[4]) == -1):
            self.controller_translation.message = 'RsDown'
            self.get_logger().info('RsDown')
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[7]) == 1:
            self.controller_translation.message = "R2"
            self.get_logger().info("RT")
            self.translator_publisher.publish(self.controller_translation)

        if msg.axes[7] > 0:
            self.controller_translation.message = "DUp"
            self.get_logger().info("D-Pad Up")
            self.translator_publisher.publish(self.controller_translation)
        if msg.axes[7] < 0:
            self.controller_translation.message = "DDown"
            self.get_logger().info("D-Pad Down")
            self.translator_publisher.publish(self.controller_translation)
        if msg.axes[6]>0:
            self.controller_translation.message = "DLeft"
            self.get_logger().info("D-Pad Left")
            self.translator_publisher.publish(self.controller_translation)
        if msg.axes[6] < 0:
            self.controller_translation.message = "DRight"
            self.get_logger().info("D-Pad Right")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[0]) == 1:
            self.controller_translation.message = "X"
            self.get_logger().info("X")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[1]) == 1:
            self.controller_translation.message = "O"
            self.get_logger().info("O")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[2]) == 1:
            self.controller_translation.message = "Squ"
            self.get_logger().info("Square")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[3]) == 1:
            self.controller_translation.message = "Tri"
            self.get_logger().info("Tri")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[4]) == 1:
            self.controller_translation.message = "L1"
            self.get_logger().info("LB")
            self.translator_publisher.publish(self.controller_translation)

        if round(msg.buttons[5]) == 1:
            self.controller_translation.message = "R1"
            self.get_logger().info("RB")
            self.translator_publisher.publish(self.controller_translation)

class PS4SubscriberNode(Node):
    def __init__(self):
        super().__init__("read_controller_node")

        self.controller_subscriber = self.create_subscription(
            msg_type=ControllerTranslation,
            topic="/ps4_controller_topic",
            callback=self.ps4_subscriber_callback,
            qos_profile=1,
        )

        self.msg_queue = queue.Queue(500)
        self.timer = self.create_timer(0.05, self.write_buffer)

    def ps4_subscriber_callback(self, msg: ControllerTranslation):
        # self.get_logger().info(f'{msg.message}\n')
        self.msg_queue.put(f"{msg.message}")
        # pico.write(f"{msg.message}".encode())

    def write_buffer(self):
        self.get_logger().info(
            f"Timer called, queue size is {self.msg_queue.qsize()}\n"
        )
        try:
            msg = self.msg_queue.get_nowait()
            self.get_logger().info(f"{msg}\n")
            pico.write(msg.encode("ascii"))
            msg = "\n"
            pico.write(msg.encode("ascii"))
        except queue.Empty:
            self.get_logger().info(f"Queue empty")
            return


def main(args=None):
    try:
        rclpy.init(args=args)
        ps4_controller_translator = PS4TranslatorNode()
        ps4_subscriber = PS4SubscriberNode()
        rclpy.spin(ps4_controller_translator)
        rclpy.spin(ps4_subscriber)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()