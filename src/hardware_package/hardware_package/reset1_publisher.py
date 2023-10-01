#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import RPi.GPIO as GPIO
import time
import board
from std_msgs.msg import Bool


class Reset1Publisher(Node):

    def __init__(self):
        # ros2 setup
        super().__init__("reset1_publisher")
        self.create_timer(0.25, self.publish_state)

        # Pi setup
        self.tiretteGPIO = 14
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tiretteGPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Publisher
        self.publisher = self.create_publisher(Bool, "reset1_topic", 10)

        # end
        self.get_logger().info("Node reset1_publisher starting")

    def publish_state(self):
        msg = Bool()
        reset1_state = not GPIO.input(self.tiretteGPIO)
        msg.data = bool(reset1_state)
        self.publisher.publish(msg)
        if msg.data:
            self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = Reset1Publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
