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
    arm_config = None

    def __init__(self):
        super().__init__("arm_service")
        self.arm_config = self.loadArmConfig()
        self.kit = ServoKit(channels=16)
        
        self.create_service(
            NullBool,
            "cmd_arm_service",
            self.arm_solarpanel_callback)

        '''
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

    def arm_solarpanel_callback(self, request, response):

        self.get_logger().info(f"XXXXXXXXXX: {self.arm_config['motor5']['default']}")

        self.kit.servo[5].angle = self.arm_config['motor5']['default']

        for i in range(10):
            self.get_logger().info("arm_solarpanel_callback open")
            self.kit.servo[4].angle = self.arm_config['motor4']['open']
            time.sleep(1)
            self.kit.servo[5].angle = self.arm_config['motor5']['yellow']
            time.sleep(1)
            self.kit.servo[5].angle = self.arm_config['motor5']['purple']
            time.sleep(1)
            self.kit.servo[5].angle = self.arm_config['motor5']['default']
            time.sleep(0.5)



            self.get_logger().info("arm_solarpanel_callback close")
            self.kit.servo[4].angle = self.arm_config['motor4']['close']
            time.sleep(1)



        response.success = True
        return response
    
    def loadArmConfig(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/armConfig.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Arm Config] armConfig.json")

        return config['calibration']



def main(args=None):
    rclpy.init(args=args)
    node = ArmService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
