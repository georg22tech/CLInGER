import rclpy
from rclpy.node import Node
from custom_interface_package.msg import ControllerTranslation
import serial
import queue

pico = serial.Serial("/dev/ttyTHS1", 115200)


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
            wrapped_msg = f"<control>{msg}</control>\n"
            self.get_logger().info(f"{wrapped_msg}\n")
            pico.write(wrapped_msg.encode("ascii"))
        except queue.Empty:
            self.get_logger().info("Queue empty")
            return


def main(args=None):
    try:
        rclpy.init(args=args)
        ps4_subscriber_node = PS4SubscriberNode()
        rclpy.spin(ps4_subscriber_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
