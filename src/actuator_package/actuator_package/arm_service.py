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
    # define GPIO pins
    direction = 22  # Direction (DIR) GPIO Pin
    step = 23  # Step GPIO Pin
    EN_pin = 24  # enable pin (LOW to enable)
    arm_offset = {
        "open": 25,
        "slightly": 65,
        "close": 73,
        "servo0_offset": -15,
        "servo1_offset": 0,
    }

    # Node State
    stack_loaded = 0
    arm_position = 0

    motion_complete = False

    def __init__(self):
        super().__init__("arm_service")
        self.kit = ServoKit(channels=16)
        self.open_arm()
        print("arm_offset", self.arm_offset)
        self.initStepper()

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

        self.get_logger().info("Arm Service has been started.")

    def initStepper(self):
        self.stepper_motor = RpiMotorLib.A4988Nema(self.direction, self.step, (21, 21, 21), "DRV8825")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.EN_pin, GPIO.OUT)  # set enable pin as output
        GPIO.output(self.EN_pin, GPIO.HIGH)


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

    def cmd_rotate(self, angle):
        service_name = "cmd_rotate_service"

        self.get_logger().info(f"[Exec Action] angle of: {angle}°")
        client = self.create_client(FloatBool, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = FloatBool.Request()
        request.angle_deg = float(angle)
        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def arm_unstack_callback(self, request, response):
        self.get_logger().info(f"Service starting process arm_unstack_callback function (request:{request})")
        GPIO.output(self.EN_pin, GPIO.LOW)

        angle = 45
        forward = 150

        # depose Pile 1 Cake 1
        self.slightlyArm()
        time.sleep(0.2)
        self.move_arm(1)
        time.sleep(0.2)
        self.close_arm()
        time.sleep(0.3)
        self.move_arm(2)
        self.cmd_forward(-forward)  # instantaneous
        time.sleep(1.5)
        self.cmd_rotate(-angle)  # instantaneous but wait for cmd_forward to finsh
        time.sleep(1.2)
        self.move_arm_down()

        # depose Pile 2 Cake 1
        self.cmd_forward(forward)  # instantaneous but wait for
        time.sleep(1.5)
        self.slightlyArm()
        time.sleep(0.2)
        self.move_arm(1)
        time.sleep(0.2)
        self.close_arm()
        time.sleep(0.3)
        self.move_arm(2)
        time.sleep(0.1)
        self.cmd_forward(-forward)  # instantaneous
        time.sleep(1.5)
        self.move_arm_down()

        # depose Pile 3 Cake 1 et 2
        self.cmd_rotate(-angle)  # instantaneous but wait for
        time.sleep(1.5)
        self.cmd_forward(forward)  # instantaneous but wait for
        time.sleep(1.5)
        self.slightlyArm()
        time.sleep(0.2)
        self.move_arm(2)
        time.sleep(0.3)
        self.close_arm()
        time.sleep(0.3)
        self.move_arm_up()
        self.cmd_forward(-forward)  # instantaneous
        time.sleep(1.2)
        self.cmd_rotate(angle)  # instantaneous but wait for

        # depose Pile 2 Cake 2
        time.sleep(1.5)
        self.move_arm(2)
        self.cmd_forward(forward)  # not wait for rotate, why?
        time.sleep(1.5)
        self.slightlyArm()
        time.sleep(0.2)
        self.close_arm()
        time.sleep(0.3)
        self.move_arm(3)
        self.cmd_forward(-forward)
        time.sleep(1.5)
        self.move_arm(2)
        self.cmd_rotate(angle)

        # depose Pile 1 Cake 2 et 3
        time.sleep(1.5)
        self.cmd_forward(forward)
        time.sleep(1.5)
        self.slightlyArm()
        time.sleep(0.2)
        self.move_arm(3)
        time.sleep(0.3)
        self.close_arm()
        time.sleep(0.3)
        self.move_arm_up()
        self.cmd_forward(-forward)  # instant
        time.sleep(1.5)
        self.cmd_rotate(-angle)

        # depose Pile 2 Cake 3
        time.sleep(1.5)
        self.cmd_forward(forward)
        time.sleep(1.5)
        self.slightlyArm()
        time.sleep(0.2)
        self.move_arm(3)
        time.sleep(0.3)
        self.close_arm()
        time.sleep(0.3)
        self.move_arm_up()
        self.cmd_forward(-forward)
        time.sleep(1.5)
        self.cmd_rotate(-angle)

        # depose Pile 3 Cake 3
        time.sleep(1.5)
        self.cmd_forward(forward)
        time.sleep(1.5)
        self.open_arm()
        self.move_arm_down()
        self.cmd_forward(-forward)

        GPIO.output(self.EN_pin, GPIO.HIGH)

        response.success = True
        return response

    def arm_drop_callback(self, request, response):
        self.get_logger().info(f"Service starting process arm_drop_callback function (request:{request})")
        GPIO.output(self.EN_pin, GPIO.LOW)

        self.move_arm_down()
        time.sleep(0.2)
        self.open_arm()

        # clean
        GPIO.output(self.EN_pin, GPIO.HIGH)
        # GPIO.cleanup()

        response.success = True
        return response

    def arm_grab_callback(self, request, response):
        push_distance = 30  # TODO
        self.get_logger().info(f"Cmd arm_drop_callback received: {request}")

        GPIO.output(self.EN_pin, GPIO.LOW)

        if self.stack_loaded == 0:
            self.slightlyArm()
            self.cmd_forward(push_distance)
            self.close_arm()
            time.sleep(0.5)
            self.move_arm_up()
            self.stack_loaded += 1
        elif self.stack_loaded == 1:
            self.open_arm()
            time.sleep(0.5)
            self.move_arm_down()
            time.sleep(0.2)
            self.slightlyArm()
            self.cmd_forward(push_distance)
            self.close_arm()
            time.sleep(0.5)
            self.move_arm_up()
            self.stack_loaded += 1
        elif self.stack_loaded == 2:
            self.open_arm()
            time.sleep(0.5)
            self.move_arm_down()
            time.sleep(0.2)
            self.slightlyArm()
            self.cmd_forward(push_distance)
            self.close_arm()
            self.stack_loaded += 1
        else:
            self.get_logger().fatal(
                f"Unknown arm setup (arm_position:{self.arm_position}, stack_loaded:{self.stack_loaded})")
            # exit(1)

        # clean
        # GPIO.output(self.EN_pin, GPIO.HIGH)
        # GPIO.cleanup()

        response.success = True
        return response

    def move_arm(self, number_of_cake):
        step = number_of_cake * 110
        if step > 360:
            step = 360
        delta = step - self.arm_position
        self.stepper_motor.motor_go(delta < 0,  # True=Clockwise, False=Counter-Clockwise
                                    "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                                    abs(delta),  # number of steps
                                    .0004,  # step delay [sec]
                                    False,  # True = print verbose output
                                    .05)  # initial delay [sec]
        self.arm_position = step

    def move_arm_up(self):
        step = 380
        delta = step - self.arm_position
        self.stepper_motor.motor_go(delta < 0,  # True=Clockwise, False=Counter-Clockwise
                                    "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                                    abs(delta),  # number of steps
                                    .0004,  # step delay [sec]
                                    False,  # True = print verbose output
                                    .05)  # initial delay [sec]
        self.arm_position = step

    def move_arm_down(self):
        step = 0
        delta = step - self.arm_position
        self.stepper_motor.motor_go(True,  # True=Clockwise, False=Counter-Clockwise
                                    "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                                    abs(delta),  # number of steps
                                    .0004,  # step delay [sec]
                                    False,  # True = print verbose output
                                    .05)  # initial delay [sec]
        self.arm_position = 0

    def close_arm(self):
        self.kit.servo[0].angle = self.arm_offset['close'] + self.arm_offset['servo0_offset']
        self.kit.servo[1].angle = 180 - self.arm_offset['close'] + self.arm_offset['servo1_offset']

    def open_arm(self):
        self.kit.servo[0].angle = self.arm_offset['open'] + self.arm_offset['servo0_offset']
        self.kit.servo[1].angle = 180 - self.arm_offset['open'] + self.arm_offset['servo1_offset']

    def slightlyArm(self):
        self.kit.servo[0].angle = self.arm_offset['slightly'] + self.arm_offset['servo0_offset']
        self.kit.servo[1].angle = 180 - self.arm_offset['slightly'] + self.arm_offset['servo1_offset']


def main(args=None):
    rclpy.init(args=args)
    node = ArmService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
