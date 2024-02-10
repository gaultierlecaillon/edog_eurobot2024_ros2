#!/usr/bin/env python3
import time
import json
import rclpy
from rclpy.node import Node
from functools import partial
import RPi.GPIO as GPIO
from robot_interfaces.srv import IntBool
from std_msgs.msg import Bool
from robot_interfaces.srv import CmdActuatorService


# Servo
from adafruit_servokit import ServoKit

# Stepper
from RpiMotorLib import RpiMotorLib


class ActuatorService(Node):
    # Stepper Config
    direction = 22  # Direction (DIR) GPIO Pin
    step = 23  # Step GPIO Pin
    EN_pin = 24  # enable pin (LOW to enable)

    # Node State
    motion_complete = False
    actuator_config = None
    elevator_position = 0

    def __init__(self):
        super().__init__("actuator_service")
        self.actuator_config = self.loadActuatorConfig()
        self.kit = ServoKit(channels=16)
        self.initStepper()

        ''' Subscribers '''
        self.create_subscription(
            Bool,
            'is_motion_complete',
            self.motion_complete_callback,
            10)
        
        ''' Services '''
        self.create_service(
            CmdActuatorService,
            "cmd_pince_service",
            self.pince_callback)
        
        self.create_service(
            CmdActuatorService,
            "cmd_elevator_service",
            self.elevator_callback)

        self.create_service(
            CmdActuatorService,
            "cmd_solarpanel_service",
            self.solarpanel_callback)
        
        self.create_service(
            CmdActuatorService,
            "cmd_graber_service",
            self.graber_callback)

        self.get_logger().info("Pince Service has been started.")

    def motion_complete_callback(self, msg):
        if msg.data:
            self.get_logger().info(
                "\033[38;5;208m[motion_complete_callback]\033[0m")
            self.get_logger().info('Motion completed received: %r' % msg.data)

    def elevator_callback(self, request, response):
        try:
            self.get_logger().info(f"Elevator_callback Called : {request.param}")
            if request.param == "loop":
                for i in range(1):
                    self.down_elevator()
                    time.sleep(1)
                    self.up_elevator()
                    time.sleep(1)
                    self.down_elevator()
                    GPIO.output(self.EN_pin, GPIO.HIGH)
            else:
                self.get_logger().info(f"unknown action: request.param: {request.param}")

            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute elevator_callback: {e}")
            response.success = False
        return response

    def solarpanel_callback(self, request, response):
        try:
            self.get_logger().info(f"solarpanel_callback Called : {request}")
            timesleep = 1
            for i in range(1):
                self.get_logger().info("Open")
                self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['open']
                self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default']
                time.sleep(timesleep)

                self.get_logger().info("Right")
                self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['yellow']
                time.sleep(timesleep)

                self.get_logger().info("Right")
                self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['purple']
                time.sleep(timesleep)

                self.get_logger().info("Close")
                self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default']
                time.sleep(0.2)
                self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['close']
                time.sleep(timesleep)

            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute pince_callback: {e}")
            response.success = False
        return response
    
    def pince_callback(self, request, response):
        try:
            self.get_logger().info(f"Pince_callback Called : {request}")
            timesleep = 0.75
            for i in range(1):
                self.get_logger().info("Open")
                self.open_pince()
                time.sleep(timesleep)

                self.get_logger().info("Right")
                self.open_right_pince()
                time.sleep(timesleep)
                
                self.get_logger().info("Left")
                self.open_left_pince()
                time.sleep(timesleep)

                self.get_logger().info("Close")
                self.close_pince()
                time.sleep(timesleep)

            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute pince_callback: {e}")
            response.success = False
        return response

    def graber_callback(self, request, response):
        try:
            self.close_pince()
            self.kit.servo[2].angle = self.actuator_config['graber']['motor2']['close']
            self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['open']    
            time.sleep(1)        
            self.get_logger().info(f"graber_callback Called : param={request.param}")

            if request.param == "":
                
                self.open_pince()
                time.sleep(0.5)
                self.cmd_forward(1000)


                self.get_logger().info(
                "\033[38;5;208mself.wait_for_motion_complete() done :)\033[0m")
                self.close_pince()

            elif request.param == "loop":                
                self.kit.servo[2].angle = self.actuator_config['graber']['motor2']['close']
                time.sleep(2) 
                for i in range(3):
                    self.up_elevator() 
                    time.sleep(2)               
                    self.kit.servo[2].angle = self.actuator_config['graber']['motor2']['open']
                    self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['open']
                    time.sleep(2)
                    self.down_elevator()
                    time.sleep(1)  
                    self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['close']
                    time.sleep(1)
                
                self.kit.servo[2].angle = self.actuator_config['graber']['motor2']['close']
                self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['open']
            else:
                self.get_logger().error(f"graber_callback unknown params")


            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute pince_callback: {e}")
            response.success = False

        return response

    
    def close_pince(self):
        self.get_logger().info(f"CLOOOOOOOOOOOOOOOOOSED")

        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['close']
            
    def open_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['open']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['open']
              
    def open_right_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['extend']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['open']
                
    def open_left_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['open']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['extend']

    def down_elevator(self):
        GPIO.output(self.EN_pin, GPIO.LOW)
        step = 0
        delta = step - self.elevator_position
        self.get_logger().info(f"Elevator DOWN {delta}")

        self.stepper_motor.motor_go(delta < 0,  # True=Clockwise, False=Counter-Clockwise
                                    "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                                    abs(delta),  # number of steps
                                    .0008,  # step delay [sec]
                                    False,  # True = print verbose output
                                    .05)  # initial delay [sec]
        self.elevator_position = step
        GPIO.output(self.EN_pin, GPIO.HIGH)

    
    def up_elevator(self):
        GPIO.output(self.EN_pin, GPIO.LOW)
        step = 350
        delta = step - self.elevator_position
        self.get_logger().info(f"Elevator UP {abs(delta)}")

        self.stepper_motor.motor_go(delta < 0,  # True=Clockwise, False=Counter-Clockwise
                                    "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                                    abs(delta),  # number of steps
                                    .0008,  # step delay [sec]
                                    False,  # True = print verbose output
                                    .05)  # initial delay [sec]
        self.elevator_position = step       

    ''' Motion Funcitons '''
    def cmd_forward(self, distance_mm):
        service_name = "cmd_forward_service"

        self.get_logger().info(f"[Exec Action] forward of: {distance_mm}mm")
        client = self.create_client(IntBool, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = IntBool.Request()
        request.distance_mm = int(distance_mm)
        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    ''' EndMotion Funcitons '''          
    
    def loadActuatorConfig(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/actuatorConfig.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Actuator Config] actuatorConfig.json")

        return config
    
    def initStepper(self):
        self.stepper_motor = RpiMotorLib.A4988Nema(self.direction, self.step, (21, 21, 21), "DRV8825")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.EN_pin, GPIO.OUT)  # set enable pin as output
        GPIO.output(self.EN_pin, GPIO.HIGH)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
