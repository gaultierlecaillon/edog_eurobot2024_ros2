#!/usr/bin/env python3
import time
import json
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CmdPositionService
import RPi.GPIO as GPIO
from robot_interfaces.srv import IntBool
from functools import partial
from std_msgs.msg import Bool
from robot_interfaces.srv import NullBool
from robot_interfaces.srv import FloatBool

# Servo
from adafruit_servokit import ServoKit

# Stepper
from RpiMotorLib import RpiMotorLib


class ArmService(Node):
    arm_offset = {
        "servo0_offset": 0,
        "servo1_offset": 0
    }

    # Node State
    motion_complete = False

    def __init__(self):
        super().__init__("arm_service")
        self.kit = ServoKit(channels=16)
        self.open_arm()
        '''
        self.create_service(
            NullBool,
            "cmd_arm_service",
            self.arm_grab_callback)

        self.create_service(
            NullBool,
            "cmd_arm_drop_service",
            self.arm_drop_callback)

        self.create_service(
            NullBool,
            "cmd_arm_unstack_service",
            self.arm_unstack_callback)
        '''

        self.get_logger().info("Arm Service has been started.")

    def open_arm(self):
        self.get_logger().info("Arm opened fake")


def main(args=None):
    rclpy.init(args=args)
    node = ArmService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
