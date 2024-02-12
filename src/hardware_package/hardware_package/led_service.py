import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import board


class LedService(Node):

    def __init__(self):
        super().__init__("led_service")
        self.get_logger().info("Pince Service has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = LedService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()