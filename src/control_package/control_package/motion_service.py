#!/usr/bin/env python3
import os
import time
import json
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Position
from robot_interfaces.msg import CmdPositionResult
from robot_interfaces.srv import CmdPositionService
from robot_interfaces.srv import CmdMotionHasStart
from robot_interfaces.srv import CmdForwardService
from robot_interfaces.srv import PositionBool
from robot_interfaces.srv import CmdRotateService
from robot_interfaces.msg import MotionCompleteResponse
from std_msgs.msg import Bool

# odrive
import odrive
from odrive.enums import *
import math


class MotionService(Node):
    cpr = 8192
    cpr_error_tolerance = 0.02

    x_ = 0  # Current robot x position
    y_ = 0  # Current robot y position
    r_ = 0  # Current robot r target_angle
    x_target = 0
    y_target = 0
    pos_estimate_0 = 0
    pos_estimate_1 = 0

    target_0 = 0
    target_1 = 0
    current_motion = {
        'in_motion': False,
        'type': '',
        'start': None,
        'target_position_0': 0,
        'target_position_1': 0,
        'emergency': False,
        'evitement': True,
    }

    last_callback_service_requester = ""

    def __init__(self):
        super().__init__("motion_service")

        self.calibration_config = None
        self.odrv0 = None
        self.loadCalibrationConfig()

        self.calibration_service_ = self.create_service(
            PositionBool,
            "cmd_calibration_service",
            self.calibration_callback)

        self.position_service_ = self.create_service(
            CmdPositionService,
            "cmd_goto_service",
            self.goto_callback)

        self.forward_service_ = self.create_service(
            CmdForwardService,
            "cmd_forward_service",
            self.forward_callback)

        self.rotate_service_ = self.create_service(
            CmdRotateService,
            "cmd_rotate_service",
            self.rotate_callback)

        self.create_service(
            CmdMotionHasStart,
            "motion_has_start",
            self.motion_has_started)

        # Subscribe to the "emergency_stop_topic"
        self.create_subscription(
            Bool,
            "emergency_stop_topic",
            self.emergency_stop_callback,
            10)

        self.publisher_is_complete = self.create_publisher(MotionCompleteResponse, 'is_motion_complete', 10)

        self.publisher_pos = self.create_publisher(Position, 'robot_position', 10)

        self.get_logger().info("Motion Service has been started.")

    def loadCalibrationConfig(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/calibration.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Calibration Config] caibration.json")

        self.calibration_config = config['calibration']
        print("self.calibration_config", self.calibration_config)

    def calibration_callback(self, request, response):

        # Find a connected ODrive (this will block until you connect one)
        self.connect_to_odrive()       

        if self.is_in_closed_loop_control():
            self.disable_motor_loop_control()
            # Loop Control
            self.reset_encoders()
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            self.get_logger().warn(f"Robot already in closed loop control")
        else:
            self.get_logger().info(f"Starting calibration...")
            self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

            while self.odrv0.axis0.current_state != AXIS_STATE_IDLE and self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)

            # Loop Control
            self.reset_encoders()
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.setPID("odrive_config.json")
        self.setPIDGains("odrive_config.json")

        self.x_ = request.start_position.x
        self.y_ = request.start_position.y
        self.r_ = request.start_position.r

        self.pos_estimate_0 = 0
        self.pos_estimate_1 = 0

        response.success = True
        time.sleep(0.2)
        return response

    def emergency_stop_callback(self, msg):
        if self.current_motion['in_motion']:

            if self.current_motion['type'] == "forward":
                pos = Position()
                x_mm = (self.odrv0.axis0.encoder.pos_estimate - self.pos_estimate_0) / \
                       self.calibration_config["linear"]["coef"]
                y_mm = (self.odrv0.axis1.encoder.pos_estimate - self.pos_estimate_1) / \
                       self.calibration_config["linear"]["coef"]

                new_x = x_mm * math.cos(math.radians(self.r_)) + self.x_
                new_y = y_mm * math.sin(math.radians(self.r_)) + self.y_

                pos.x = int(new_x)
                pos.y = int(new_y)
                pos.r = float(self.r_)
                self.publisher_pos.publish(pos)

            if msg.data and not self.current_motion['emergency'] and self.current_motion['target_position_0'] == \
                    self.current_motion['target_position_1']:
                self.setPID("emergency_stop.json")
                self.setPIDGains("emergency_stop.json")
                self.odrv0.axis0.controller.input_pos = self.odrv0.axis0.encoder.pos_estimate
                self.odrv0.axis1.controller.input_pos = self.odrv0.axis1.encoder.pos_estimate
                #self.get_logger().error(f"Obstacle in front of the robot. Stopped @ input_pos_0={self.odrv0.axis0.encoder.pos_estimate} and input_pos_1={self.odrv0.axis1.encoder.pos_estimate} ")
                self.current_motion['emergency'] = True
                self.current_motion['in_motion'] = False
                time.sleep(2)
                self.setPID("odrive_config.json")
                self.setPIDGains("odrive_config.json")

            self.is_motion_complete()

        if not msg.data and self.current_motion['emergency']:
            time.sleep(1)
            pos_error_0 = abs(
                self.pos_estimate_0 + self.current_motion['target_position_0'] - self.odrv0.axis0.encoder.pos_estimate)
            pos_error_1 = abs(
                self.pos_estimate_1 + self.current_motion['target_position_1'] - self.odrv0.axis1.encoder.pos_estimate)

            pos_error_mm = ((pos_error_0 + pos_error_1) / 2) / self.calibration_config["linear"]["coef"]
            self.pos_estimate_0 = self.odrv0.axis0.encoder.pos_estimate
            self.pos_estimate_1 = self.odrv0.axis1.encoder.pos_estimate

            print("pos_error_0", pos_error_0)
            print("pos_error_1", pos_error_1)
            print("pos_error_mm", pos_error_mm)

            self.motionForward(pos_error_mm)
            self.current_motion['emergency'] = False
        elif msg.data and self.current_motion['emergency']:
            print("waiting for the obstacle to move")

    #
    # Distance to do in mm
    #
    def forward_callback(self, request, response):
        self.get_logger().info(f"Cmd forward_callback received: {request}")

        self.motionForward(request.distance_mm)
        self.x_target = self.x_ + round(request.distance_mm * math.cos(math.radians(self.r_)), 2)
        self.y_target = self.y_ + round(request.distance_mm * math.sin(math.radians(self.r_)), 2)

        self.last_callback_service_requester = str(request.service_requester)
        response.motion_completed_response.service_requester = request.service_requester
        response.motion_completed_response.success = True
        return response

    def rotate_callback(self, request, response):
        self.get_logger().info(f"Cmd rotate_callback received: {request}")

        self.motionRotate(request.angle_deg)
        self.r_ += request.angle_deg

        self.last_callback_service_requester = str(request.service_requester)
        response.motion_completed_response.service_requester = request.service_requester
        response.motion_completed_response.success = True
        return response

    def goto_callback(self, request, response):
        self.get_logger().info(f"Cmd goto_callback received: {request}")

        # Calculate the target_angle in degrees to reach the point(x,y)
        target_angle = math.degrees(math.atan2(request.y - self.y_, request.x - self.x_)) - self.r_

        # Calculate the distance between A and B in mm
        increment_mm = math.sqrt((request.x - self.x_) ** 2 + (request.y - self.y_) ** 2)
        # Calculate the finak angle

        final_target_angle = request.r - target_angle - self.r_
        self.get_logger().warn(f"final_target_angle={final_target_angle}={target_angle} + {self.r_}+{request.r}")

        if request.r == -1:
            final_target_angle = 0

        response.cmd = CmdPositionResult()
        response.cmd.rotation = float(target_angle)
        response.cmd.forward = int(increment_mm)
        response.cmd.final_rotation = float(final_target_angle)
        return response

    def motionRotate(self, target_angle):
        try:
            rotation_to_do = self.r_ + target_angle
            increment_mm = target_angle * float(self.calibration_config["rotation"]["coef"])
            increment_pos = float(self.calibration_config["linear"]["coef"]) * increment_mm

            self.get_logger().warn(f"[MotionRotate] target_angle={target_angle}°, rotation_to_do={rotation_to_do}°")

            self.call_motion_has_started(increment_pos, -increment_pos)

            self.odrv0.axis0.controller.move_incremental(increment_pos, False)
            self.odrv0.axis1.controller.move_incremental(-increment_pos, False)
        except AttributeError:
            self.get_logger().error("NoOdrive Initiated")
            self.shutdown_node()

    def motionForward(self, increment_mm):
        try:
            increment_pos = float(self.calibration_config["linear"]["coef"]) * increment_mm

            self.get_logger().warn(f"[MotionForward] (increment_mm={increment_mm} mm, increment_pos={increment_pos} pos)")

            self.call_motion_has_started(increment_pos, increment_pos)

            self.odrv0.axis0.controller.move_incremental(increment_pos, False)
            self.odrv0.axis1.controller.move_incremental(increment_pos, False)
        except AttributeError:
            self.get_logger().error("\033[91m[🚨 ERROR] Something went wrong with Odrive 🚨\033[0m")  
            self.shutdown_node()

    def call_motion_has_started(self, increment_pos_0, increment_pos_1):
        service_name = "motion_has_start"
        client = self.create_client(CmdMotionHasStart, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")
        request = CmdMotionHasStart.Request()
        request.target_position_0 = increment_pos_0
        request.target_position_1 = increment_pos_1
        request.evitement = True
        client.call_async(request)
        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def motion_has_started(self, request, response):
        target_position_0 = request.target_position_0
        target_position_1 = request.target_position_1

        if target_position_0 == target_position_1:
            type = "forward"
        else:
            type = "rotation"

        self.get_logger().info("Motion has begun !");

        self.current_motion['in_motion'] = True
        self.current_motion['type'] = type
        self.current_motion['start'] = time.time()
        self.current_motion['target_position_0'] = target_position_0
        self.current_motion['target_position_1'] = target_position_1
        self.current_motion['evitement'] = request.evitement

        response.success = True
        return response

    def is_motion_complete(self):
        motion_completed = False

        if self.current_motion['in_motion']:
            timeout = 5

            pos_error_0 = abs(
                self.pos_estimate_0 + self.current_motion['target_position_0'] - self.odrv0.axis0.encoder.pos_estimate)
            pos_error_1 = abs(
                self.pos_estimate_1 + self.current_motion['target_position_1'] - self.odrv0.axis1.encoder.pos_estimate)

            if (self.current_motion['target_position_0'] == 0 and self.current_motion['target_position_1'] == 0) or not \
                    self.current_motion['in_motion']:
                motion_completed = True

            # Check if both axes have reached their target positions within the tolerance range
            elif pos_error_0 <= self.cpr_error_tolerance and pos_error_1 <= self.cpr_error_tolerance:
                self.get_logger().info(
                    f"\033[38;5;46mMotion completed in {time.time() - self.current_motion['start']:.3f} seconds (pos_error_0:{pos_error_0}, pos_error_1:{pos_error_1}\033[0m")
                motion_completed = True

            elif time.time() - self.current_motion['start'] > timeout:
                error_message = f"Motion completion timeout (pos_error_0: {pos_error_0}, pos_error_1: {pos_error_1})"
                self.get_logger().error(f'\033[91m{error_message}\033[0m')

                self.print_robot_infos()
                motion_completed = False

            '''
            self.get_logger().info(
                f"Motion started from {time.time() - self.current_motion['start']:.3f} seconds (pos_error_0:{pos_error_0}, pos_error_1:{pos_error_1}\n")
            self.get_logger().info(
                f"self.pos_estimate_0: {self.pos_estimate_0}, self.current_motion['target_position_0']: {self.current_motion['target_position_0']}, axis0.encoder.pos_estimate: {self.odrv0.axis0.encoder.pos_estimate}")
            '''

        if motion_completed:
            self.x_ = self.x_target
            self.y_ = self.y_target

            # Publish True on the 'is_motion_complete' topic
            msg = MotionCompleteResponse()
            msg.service_requester = self.last_callback_service_requester #TODO fix, can be also from rotate, only forward is managed
            msg.success = True
            self.publisher_is_complete.publish(msg)

            self.pos_estimate_0 = self.odrv0.axis0.encoder.pos_estimate
            self.pos_estimate_1 = self.odrv0.axis1.encoder.pos_estimate

            self.current_motion['in_motion'] = False
            self.current_motion['start'] = None
            self.current_motion['target_position_0'] = 0
            self.current_motion['target_position_1'] = 0

            self.print_robot_infos()

        return motion_completed

    def getEncoderIndex(self, axis):
        return axis.encoder.shadow_count

    def reset_encoders(self):
        self.get_logger().info(f"Encoders reset")
        self.odrv0.axis0.encoder.set_linear_count(0)
        self.odrv0.axis1.encoder.set_linear_count(0)

    def setPIDGains(self, config_filename):
        with open('/home/edog/ros2_ws/src/control_package/resource/' + config_filename) as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Odrive Config] odrive_config.json")

        gain = config['gain']
        vel_gain = config['vel_gain']
        vel_integrator_gain = config['vel_integrator_gain']

        self.odrv0.axis0.controller.config.pos_gain = gain  # Position gain for axis0
        self.odrv0.axis1.controller.config.pos_gain = gain  # Position gain for axis1

        self.odrv0.axis0.controller.config.vel_gain = vel_gain  # Velocity gain for axis0
        self.odrv0.axis1.controller.config.vel_gain = vel_gain  # Velocity gain for axis1

        self.odrv0.axis0.controller.config.vel_integrator_gain = vel_integrator_gain  # Velocity integrator gain for axis0
        self.odrv0.axis1.controller.config.vel_integrator_gain = vel_integrator_gain  # Velocity integrator gain for axis1

    def setPID(self, config_filename):
        with open('/home/edog/ros2_ws/src/control_package/resource/' + config_filename) as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Odrive Config] odrive_config.json")

        accel_limit = config['accel_limit']
        decel_limit = config['decel_limit']
        vel_limit = config['vel_limit']

        self.odrv0.axis0.trap_traj.config.accel_limit = accel_limit
        self.odrv0.axis1.trap_traj.config.accel_limit = accel_limit

        self.odrv0.axis0.trap_traj.config.decel_limit = decel_limit
        self.odrv0.axis1.trap_traj.config.decel_limit = decel_limit

        self.odrv0.axis0.trap_traj.config.vel_limit = vel_limit
        self.odrv0.axis1.trap_traj.config.vel_limit = vel_limit

        self.odrv0.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
        self.odrv0.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ

    def is_in_closed_loop_control(self):
        axis0_state = self.odrv0.axis0.current_state
        axis1_state = self.odrv0.axis1.current_state

        if axis0_state == AXIS_STATE_CLOSED_LOOP_CONTROL and axis1_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            return True
        else:
            return False

    def disable_motor_loop_control(self):
        if self.odrv0 is not None:
            self.get_logger().info(f"Disabling motor loop control")
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
        else:
            self.get_logger().error(f"Odrive board is not connected")

    def print_robot_infos(self):
        self.get_logger().info(
            f"[Robot Infos] x:{self.x_}, y:{self.y_}, r:{self.r_}, encoder_0_index: {self.getEncoderIndex(self.odrv0.axis0)}, encoder_1_index:{self.getEncoderIndex(self.odrv0.axis1)}")

    def connect_to_odrive(self):
        try:
            self.get_logger().info("Attempting to connect to ODrive...")
            self.odrv0 = odrive.find_any(timeout=5)  # Adjust timeout as needed
            if self.odrv0 is None:
                raise Exception("\033[38;5;208mNo Odrive found 😔 we need it baguette\033[0m\n")
            self.get_logger().info(f"🎉 OdriveBoard connected successfully ! (VBus Voltage: {self.odrv0.vbus_voltage}⚡)")
            self.odrv0.clear_errors()
        except Exception as e:
            self.get_logger().error('\033[91m' + str(e) + '\033[0m', throttle_duration_sec=1)  # Red text for errors


    def shutdown_node(self):
        self.get_logger().info("\033[91mShutting down node due to ODrive connection failure. 💥\033[0m")
        # Perform any necessary cleanup here
        os.kill(os.getpid(), signal.SIGINT)  # Send SIGINT to the current process This method uses os.kill to send a SIGINT (interrupt signal) to the current 

def main(args=None):
    rclpy.init(args=args)
    node = MotionService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
