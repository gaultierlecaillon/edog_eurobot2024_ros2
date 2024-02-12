#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# PI Import
import RPi.GPIO as GPIO
import time
import board


class TirettePublisher(Node):

    def __init__(self):
        # ros2 setup
        super().__init__("tirette_publisher")
        self.create_timer(0.25, self.publish_state)

        # Pi setup
        self.tiretteGPIO = 4
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tiretteGPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Publisher
        self.publisher_ = self.create_publisher(Bool, "tirette_topic", 10)

        # end
        self.get_logger().info("Node tirette_publisher started")

    #Todo, if msg.data == True the shutdown the node
    def publish_state(self):
        msg = Bool()
        tirette_state = not GPIO.input(self.tiretteGPIO)
        msg.data = bool(tirette_state)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TirettePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
