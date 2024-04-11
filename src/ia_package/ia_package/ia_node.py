#!/usr/bin/env python3
import json
import time
import rclpy
import RPi.GPIO as GPIO
import os
import signal

from rclpy.node import Node
from functools import partial
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from robot_interfaces.srv import PositionBool
from robot_interfaces.srv import CmdPositionService
from robot_interfaces.srv import CmdActuatorService
from robot_interfaces.srv import CmdForwardService
from robot_interfaces.srv import CmdRotateService
from robot_interfaces.msg import MotionCompleteResponse
from example_interfaces.msg import String


# Servo
from adafruit_servokit import ServoKit

class IANode(Node):
    action_name = None
    action_param = None
    current_action_already_printed = False

    # Declare Timer (Default 100 secondes)
    shutdown_after_seconds = 100

    def __init__(self):
        super().__init__('ia_node')
        self.kit = ServoKit(channels=16)        

        # Declare and get the strategy_filename parameter
        self.config = None
        self.declare_parameter('strategy_filename', 'strat')
        self.strategy_filename = self.get_parameter('strategy_filename').get_parameter_value().string_value

        self.actions_dict = []
        self.load_strategy()

        # ROS2 Node timer
        self.number_timer_ = self.create_timer(0.1, self.master_callback)

        ''' Publisher '''
        if not hasattr(self, 'voice_publisher'):
            self.voice_publisher = self.create_publisher(String, "voice_topic", 10)
        
        # Create a publisher for the shutdown signal
        self.shutdown_publisher = self.create_publisher(Bool, 'shutdown_topic', 10)

        ''' Subscribers '''
        self.pids = []
        self.create_subscription(
            Int32,
            'killable_nodes_pid',
            self.kill_callback,
            10) 

        self.create_subscription(
            Bool,
            'bau_topic',
            self.bau_callback,
            10)        

        self.create_subscription(
            MotionCompleteResponse,
            "is_motion_complete",
            self.is_motion_complete_callback,
            10)

        self.get_logger().info("\033[38;5;208mIA Node is running!\n\n\t\t\t (⌐■_■) 𝘴𝘶𝘱 𝘣𝘳𝘢 ?\n\033[0m\n")

    def master_callback(self):
        self.execute_current_action()

    '''
    Execute Actions in the queue
    '''

    def execute_current_action(self):
        if len(self.actions_dict) > 0:
            current_action = self.actions_dict[0]
            action = current_action['action']
            self.action_name, self.action_param = list(action.items())[0]

            if not self.current_action_already_printed:
                self.get_logger().info(
                    f"[Exec Current Action] {self.action_name} {self.action_param} ({current_action['status']})")
                self.current_action_already_printed = True

            if current_action['status'] == "pending":
                try:
                    getattr(self, self.action_name)(self.action_param)
                    self.update_current_action_status('on going')
                except Exception as e:
                    self.get_logger().fatal(e)
                    self.get_logger().fatal(
                        f"Action {self.action_name} is unknown, no method call {self.action_name} in ia node")
                    exit(1)
        else:
            self.get_logger().info(
                "\033[38;5;208m[Match done] No more actions to exec\n\n\t\t\t (⌐■_■) 𝘪𝘴 𝘪𝘵 𝘗1 ?\033[0m\n")            
            self.shutdown_nodes()

    def is_motion_complete_callback(self, msg):
        action_name = next(iter(self.actions_dict[0]['action']))
        self.get_logger().info(f"\033[38;5;208m[motion_complete_callback] Received in IAService: {msg} for {action_name}\033[0m")

        if msg.success and msg.service_requester == str(self.__class__.__name__):
            if action_name == 'graber':
                self.get_logger().info(f"\033[38;5;208mIgnore this motion complete graber\033[0m")
                pass
            else:
                self.get_logger().info(f"\033[95m[IaNode.is_motion_complete_callback] Current Action Done ! {action_name}\033[0m")
                self.update_current_action_status('done')

    def waiting_tirette(self, param):
        self.subscriber_ = self.create_subscription(
            Bool,
            "tirette_topic",
            lambda msg: self.callback_waiting_tirette(msg, param), 1)
        if param:
            self.get_logger().info(f"\033[95m[⏳ WAITING ⏳] Waiting for tirette\033[0m")
        else:
            self.get_logger().info(f"\033[95m[⏳ WAITING ⏳] Pull the tirette and the match will start for {self.shutdown_after_seconds}s 🏁\033[0m")





    def callback_waiting_tirette(self, msg, param):
        if msg.data == param:
            if not param: # The tirette have been pulled, the Match start
                # Match timer
                self.match_timer = self.create_timer(self.shutdown_after_seconds, self.shutdown_nodes)
            
            self.update_current_action_status('done')
            self.destroy_subscription(self.subscriber_)  # Unsubscribe from the topic


    def calibrate(self, param):
        service_name = "cmd_calibration_service"

        self.get_logger().info(f"[Exec Action] calibrate with param: {param}")
        client = self.create_client(PositionBool, service_name)
        while not client.wait_for_service(0.25):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        int_param = [int(x) for x in self.config['startingPos'].split(",")]
        request = PositionBool.Request()
        request.start_position.x = int_param[0]
        request.start_position.y = int_param[1]
        request.start_position.r = float(int_param[2])
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to cmd_calibration_service")

    def led(self, param):
        service_name = "cmd_led_service"
        self.get_logger().info(f"Performing 'Led' action with param: {param}")
        client = self.create_client(CmdActuatorService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdActuatorService.Request()
        request.param = param
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to {service_name}")


    def solarpanel(self, param):
        service_name = "cmd_solarpanel_service"
        self.get_logger().info(f"Performing 'SolarPanel' action with param: {param}")
        client = self.create_client(CmdActuatorService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdActuatorService.Request()
        request.param = param
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def pince(self, param):
        service_name = "cmd_pince_service"
        self.get_logger().info(f"Performing 'Pince' action with param: {param}")
        client = self.create_client(CmdActuatorService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdActuatorService.Request()
        request.param = param
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def graber(self, param):
        service_name = "cmd_graber_service"
        self.get_logger().info(f"Performing 'Graber' action with param: {param}")
        client = self.create_client(CmdActuatorService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdActuatorService.Request()
        request.param = param
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def elevator(self, param):
        service_name = "cmd_elevator_service"
        self.get_logger().info(f"Performing 'Elevator' action with param: {param}")
        client = self.create_client(CmdActuatorService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdActuatorService.Request()
        request.param = str(param)
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def forward(self, param):
        service_name = "cmd_forward_service"

        self.get_logger().info(f"[Exec Action] forward with param: '{param}'")
        client = self.create_client(CmdForwardService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdForwardService.Request()
        request.service_requester = str(self.__class__.__name__)
        request.distance_mm = int(param)

        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def rotate(self, param):
        service_name = "cmd_rotate_service"

        self.get_logger().info(f"[Exec Action] rotate with param: '{param}'")
        client = self.create_client(CmdRotateService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdRotateService.Request()
        request.angle_deg = float(param)
        request.service_requester = str(self.__class__.__name__)

        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def goto(self, param):
        service_name = "cmd_goto_service"
        self.get_logger().info(f"[Exec Action] goto with param: {param}")

        client = self.create_client(CmdPositionService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        int_param = [int(x) for x in param.split(",")]
        request = CmdPositionService.Request()
        request.x = int_param[0]
        request.y = int_param[1]
        request.r = int_param[2]
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.transform_goto_in_cmd))

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def transform_goto_in_cmd(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"[transform_goto_in_cmd pop!] {self.actions_dict}")
            self.actions_dict.pop(0) #remove goto action and replace by rotate -> forward -> rotate
            self.get_logger().info(f"[transform_goto_in_cmd after pop] {self.actions_dict}")
            
            
            self.get_logger().info(f"\033[95m[response.cmd] {response.cmd}\033[0m")

            
            if response.cmd.final_rotation != 0:
                self.actions_dict.insert(0, {
                    'action': {'rotate': response.cmd.final_rotation},
                    'status': 'pending'
                })
            if response.cmd.forward != 0:
                self.actions_dict.insert(0, {
                    'action': {'forward': response.cmd.forward},
                    'status': 'pending'
                })
            if response.cmd.rotation != 0:
                self.actions_dict.insert(0, {
                    'action': {'rotate': response.cmd.rotation},
                    'status': 'pending'
                })
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def callback_current_action(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"\033[95m[IaNode.callback_current_action] Current Action Done ! {response}\033[0m")
                self.update_current_action_status('done')
            else:
                self.get_logger().info(f"Something went wrong with response: {response}")

        except Exception as e:

            self.get_logger().error("Service call failed %r" % (e,))

    def update_current_action_status(self, status):
        if status == "done":
            self.get_logger().info(
                f"\033[38;5;208m[ACTION COMPLETE] {self.actions_dict[0]} received status: {status}\033[0m")
            self.actions_dict.pop(0)
            self.get_logger().info(
                f"[NEXT ACTION(S)] {self.actions_dict}")
            self.speak("task_complet_sound.mp3")
        else: # on going
            self.actions_dict[0]['status'] = status
        self.current_action_already_printed = False

    def load_strategy(self):
        try:
            # Attempt to open and load the strategy file
            with open('/home/edog/ros2_ws/src/ia_package/resource/' + self.strategy_filename + '.json') as file:
                self.config = json.load(file)


            if 'timer' in self.config:
                self.shutdown_after_seconds = int(self.config['timer'])

            # Logging the loaded strategy information
            self.get_logger().info(f"\033[95m[Loading Strategy] {self.config['name']} ({self.config['description']}) during {self.shutdown_after_seconds} secondes !\033[0m")
            self.get_logger().info(f"[Start] Color: {self.config['color']} | StartPos:({self.config['startingPos']})")

            # Processing the strategy actions
            self.actions_dict = []  
            for strat in self.config['strategy']:
                for action in strat['actions']:
                    self.actions_dict.append(
                        {
                            'action': action,
                            'status': 'pending'
                        }
                    )
        except Exception as e:
            # Log the error message in red text
            self.get_logger().error('\033[91m' + "Error loading strategy: " + str(e) + '\033[0m')


    def speak(self, action):
        msg = String()
        msg.data = str(action)
        self.voice_publisher.publish(msg)
        self.get_logger().info(f"[Publish topic] voice_topic msg:{msg}")

    '''
    Shutdown the node at the end of the Match
    '''

    def bau_callback(self, msg):
        self.shutdown_nodes()

    def kill_callback(self, msg):
        self.pids.append(msg.data)
        self.get_logger().error(f'\033[91mReceived PIDs: {self.pids}\033[0m')        

    def kill_all(self):
        
        self.get_logger().error('\033[91mKILL THEM ALL !\033[0m')

        for pid in self.pids:            
            self.kill_process(pid)
    
    def kill_process(self, pid):
        if pid:
            try:
                # Replace 'SIGINT' with 'SIGKILL' if necessary
                os.kill(pid, signal.SIGINT)
                self.get_logger().info(f'Successfully sent kill signal to PID {pid}')
            except Exception as e:
                self.get_logger().info(f'Failed to kill process {pid}: {e}')
    
    def disableActuators(self):
        self.kit.servo[0].angle = None
        self.kit.servo[1].angle = None
        self.kit.servo[2].angle = None
        self.kit.servo[3].angle = None
        self.kit.servo[4].angle = None
        self.kit.servo[5].angle = None
        #todo add stepper

    def shutdown_nodes(self):
        self.get_logger().info(f"\033[38;5;208m[SHUTDOW NODE] Adios\n\n\t\t\t (⌐■_■) 𝘪𝘴 𝘪𝘵 𝘗1 ?\n\033[0m\n")
        self.kill_all()
        self.match_timer.cancel()        
        self.actions_dict.clear()
        self.disableActuators()

        try:           
            # Allow some time for the publisher to be set up
            rclpy.spin_once(self, timeout_sec=0.1)

            # Publish the shutdown message
            shutdown_msg = Bool()
            shutdown_msg.data = True  # Message indicating shutdown
            self.shutdown_publisher.publish(shutdown_msg)
            self.get_logger().info('Published shutdown signal')
        except Exception as e:
            self.get_logger().error('Error while publishing shutdown signal: {}'.format(e))

        # Shutdown the ROS context
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    ia_node = IANode()
    rclpy.spin(ia_node)
    ia_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
