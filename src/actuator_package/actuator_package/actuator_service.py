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
from robot_interfaces.srv import CmdActuatorService


# Servo
from adafruit_servokit import ServoKit

# Stepper
from RpiMotorLib import RpiMotorLib


class ActuatorService(Node):

    # Node State
    motion_complete = False
    actuator_config = None

    def __init__(self):
        super().__init__("actuator_service")
        self.actuator_config = self.loadActuatorConfig()
        self.kit = ServoKit(channels=16)
        
        self.create_service(
            CmdActuatorService,
            "cmd_pince_service",
            self.pince_callback)

        self.get_logger().info("Pince Service has been started.")

    def pince_callback(self, request, response):
        self.get_logger().info(f"Pince_callback Called : {request}")

        for i in range(10):
            self.get_logger().info(f"Close")
            self.close_pince()
            time.sleep(1)

            self.get_logger().info(f"Open")
            self.open_pince()
            time.sleep(1)

            '''
            self.get_logger().info(f"Right")
            self.open_right_pince()
            time.sleep(1)
            
            self.get_logger().info(f"Right")
            self.open_left_pince()
            time.sleep(1)
            '''

        response.success = True
        return response
    
    def close_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['close']
            
    def open_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['open']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['open']
              
    def open_right_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['open']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['close']
                
    def open_left_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['open']
                  
    
    def loadActuatorConfig(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/actuatorConfig.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Actuator Config] actuatorConfig.json")

        return config



def main(args=None):
    rclpy.init(args=args)
    node = ActuatorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
