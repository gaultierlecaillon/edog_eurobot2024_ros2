#!/usr/bin/env python3
import os
import time
import json
import rclpy
from rclpy.node import Node
from functools import partial
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from robot_interfaces.srv import CmdActuatorService
from robot_interfaces.srv import CmdForwardService
from robot_interfaces.msg import MotionCompleteResponse
from example_interfaces.msg import String
from std_msgs.msg import Int32


# Stepper (set first BCM)
from RpiMotorLib import RpiMotorLib

# Servo
from adafruit_servokit import ServoKit


class ActuatorService(Node):
    # Stepper Config
    direction = 22  # Direction (DIR) GPIO Pin
    step = 23  # Step GPIO Pin
    EN_pin = 24  # enable pin (LOW to enable)

    # Node State
    actuator_config = None
    elevator_position = 0

    def __init__(self):
        super().__init__("actuator_service")

        self.pid_publisher = self.create_publisher(Int32, 'killable_nodes_pid', 10)        

        self.actuator_config = self.loadActuatorConfig()
        self.kit = ServoKit(channels=16)
        self.initStepper()
        self.initServo()

        ''' Subscribers '''
        self.create_subscription(
            MotionCompleteResponse,
            'is_motion_complete',
            self.motion_complete_callback,
            10)
        
        self.create_subscription(
            Bool,
            'shutdown_topic',
            self.shutdown_callback,
            10)

        
        ''' Publisher '''
        if not hasattr(self, 'voice_publisher'):
            self.voice_publisher = self.create_publisher(String, "voice_topic", 10)
        
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

        self.publish_pid()
        self.get_logger().info("Pince Service has been started.")

    def publish_pid(self):
        msg = Int32()
        msg.data = os.getpid()  # Get the current process ID
        self.get_logger().error(f'\033[91m[publish_pid] {msg.data}\033[0m')
        self.pid_publisher.publish(msg)

    def motion_complete_callback(self, msg):
        if msg.success and msg.service_requester == str(self.__class__.__name__):
            self.get_logger().info(f"\033[38;5;208m[motion_complete_callback] Received in ActuatorService: {msg.data}\033[0m")

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
                self.get_logger().info(f"\033[91mself.get_logger().info(f\"Unknown action: {request.param} (elevator_callback)\")\033[0m")
                

            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute elevator_callback: {e}")
            response.success = False
        return response

    def solarpanel_callback(self, request, response):
        try:
            self.get_logger().info(f"solarpanel_callback Called : {request}")
            if request.param == "yellow" or request.param == "blue":
                offset = 0
                self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['open']
                self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default']
                time.sleep(0.3)
                if request.param == "yellow":
                    self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['yellow']
                    offset = 10
                else:
                    self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['blue']
                    offset = -15
                time.sleep(0.2)
                self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['low']
                time.sleep(0.2)
                self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default'] + offset
                time.sleep(0.1)
                self.initServo()
            elif request.param == "slow":
                for i in range(0,5):
                    self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['close']
                    self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default']
                    time.sleep(0.2)
                    self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['open']
                    time.sleep(0.2)
                    self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['blue']
                    time.sleep(0.2)
                    self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default']
                    time.sleep(0.5)
                    self.kit.servo[4].angle = self.actuator_config['solarpanel']['motor4']['close']
                    self.kit.servo[5].angle = self.actuator_config['solarpanel']['motor5']['default']



            else:
                self.get_logger().info(f"\033[91mself.get_logger().info(f\"Unknown action: {request.param} (solarpanel_callback)\")\033[0m")

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
            self.get_logger().info(f"graber_callback Called : param={request.param}")
            self.open_pince()
            time.sleep(0.5)
            self.kit.servo[2].angle = self.actuator_config['graber']['motor2']['open']
            self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['open']    
            time.sleep(1)
            
            self.cmd_forward(700)
            time.sleep(3)
            
            step = self.actuator_config['elevator']['level1']
            self.move_elevator(step)
            #do top before /!\
            self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['plant'] #bottom
            
            time.sleep(2)
            
            
            
            

            if request.param == "":
                self.get_logger().info(f"params default")
            elif request.param == "up":
                self.get_logger().info(f"params up")
            else:
                self.get_logger().error(f"graber_callback unknown params")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute pince_callback: {e}")
            response.success = False

        return response

    
    def close_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['close']
    
    def extend_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['extend']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['extend']
            
    def open_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['open']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['open']

    def tight_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['tight']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['tight']
              
    def open_right_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['extend']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['open']
                
    def open_left_pince(self):
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['open']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['extend']

    def down_elevator(self):
        step = self.actuator_config['elevator']['down']
        self.move_elevator(step)

    def up_elevator(self):
        step = self.actuator_config['elevator']['up']
        self.move_elevator(step)
        
    def move_elevator(self, step):
        GPIO.output(self.EN_pin, GPIO.LOW)
        delta = step - self.elevator_position
        self.get_logger().info(f"Move elevator to {abs(delta)}")

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
        client = self.create_client(CmdForwardService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdForwardService.Request()
        request.service_requester = self.__class__.__name__
        request.distance_mm = int(distance_mm)
        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def speak(self, action):
        msg = String()
        msg.data = str(action)
        self.voice_publisher.publish(msg)
        self.get_logger().info(f"[Publish topic] voice_topic msg:{msg}")

    ''' EndMotion Functions '''          
    
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

    def initServo(self):
        self.kit.servo[2].angle = self.actuator_config['graber']['motor2']['close']
        time.sleep(0.25)
        self.kit.servo[3].angle = self.actuator_config['graber']['motor3']['close']        
        time.sleep(0.5)
        self.kit.servo[0].angle = self.actuator_config['pince']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['pince']['motor1']['close']
        time.sleep(0.5)
        # todo: add for solar panel servo
        
        self.kit.servo[0].angle = None
        self.kit.servo[1].angle = None
        self.kit.servo[2].angle = None
        self.kit.servo[3].angle = None
        self.kit.servo[4].angle = None
        self.kit.servo[5].angle = None
    
    def shutdown_callback(self, msg):       
        if msg.data:                  
            try:
                self.get_logger().info("Shutdown signal received {msg}. Shutting down node... ") 
                self.initServo()                
                self.initStepper()  
                GPIO.cleanup()
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
            finally:
                rclpy.shutdown()  # Shutdown the ROS client library for Python

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
