#!/usr/bin/env python3
import json
import rclpy
import math
from rclpy.node import Node
from functools import partial
from std_msgs.msg import Bool
from robot_interfaces.srv import PositionBool
from robot_interfaces.srv import CmdPositionService
from robot_interfaces.srv import CmdActuatorService
from robot_interfaces.srv import BoolBool
from robot_interfaces.srv import IntBool
from robot_interfaces.srv import NullBool
from robot_interfaces.srv import FloatBool


class IANode(Node):
    action_name = None
    action_param = None
    current_action_already_printed = False

    unstack_num_action = 0

    def __init__(self):
        super().__init__('ia_node')

        # Declare and get the strategy_filename parameter
        self.config = None
        self.declare_parameter('strategy_filename', 'strat')
        self.strategy_filename = self.get_parameter('strategy_filename').get_parameter_value().string_value

        self.actions_dict = []
        self.load_strategy()

        self.number_timer_ = self.create_timer(0.1, self.master_callback)

        self.create_subscription(
            Bool,
            "is_motion_complete",
            self.is_motion_complete_callback,
            10)

        self.get_logger().info("\033[38;5;208mIA Node is running!\n\n\t\t\t (⌐■_■) 𝘴𝘶𝘱 𝘣𝘳𝘢 ?\033[0m\n")

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

            rclpy.shutdown()

    def is_motion_complete_callback(self, msg):
        if msg.data:
            if next(iter(self.actions_dict[0]['action'])) == 'unstack' and self.unstack_num_action < 18:
                self.unstack_num_action += 1
                print("self.unstack_num_action", self.unstack_num_action)
            elif next(iter(self.actions_dict[0]['action'])) == 'unstack' and self.unstack_num_action >= 18:
                self.unstack_num_action = 0
                self.update_current_action_status('done')
                print("DOOOOOOOOOONE")
            else:
                self.update_current_action_status('done')
                print("3333333333333333")


    def waiting_tirette(self, param):
        self.subscriber_ = self.create_subscription(
            Bool,
            "tirette_topic",
            lambda msg: self.callback_waiting_tirette(msg, param), 1)


    def callback_waiting_tirette(self, msg, param):
        if msg.data == cast_str_bool(param):
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

    def pince(self, param):
        service_name = "cmd_pince_service"
        self.get_logger().info(f"Performing 'Pince' action with param: {param}")
        client = self.create_client(CmdActuatorService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdActuatorService.Request()
        request.param = str(param)
        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def unstack(self, param):
        service_name = "cmd_arm_unstack_service"  # TODO unstack and drop and grab soulg be the same service
        self.get_logger().info(f"Performing 'unstack' action with param: {param}")
        client = self.create_client(NullBool, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = NullBool.Request()
        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def drop(self, param):
        service_name = "cmd_arm_drop_service"

        self.get_logger().info(f"Performing 'drop' action with param: {param}")

        client = self.create_client(NullBool, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = NullBool.Request()
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to cmd_arm_drop_service")

    def forward(self, param):
        service_name = "cmd_forward_service"

        self.get_logger().info(f"[Exec Action] forward with param: '{param}'")
        client = self.create_client(IntBool, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = IntBool.Request()
        request.distance_mm = int(param)

        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def rotate(self, param):
        service_name = "cmd_rotate_service"

        self.get_logger().info(f"[Exec Action] rotate with param: '{param}'")
        client = self.create_client(FloatBool, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = FloatBool.Request()
        request.angle_deg = float(param)

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
            self.get_logger().warn(f"[callback_goto]: {response.cmd}")

            self.actions_dict.pop(0)
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
                self.get_logger().info(f"[Callback Current Action] Done ! {response}")
                self.update_current_action_status('done')
            else:
                self.get_logger().info(f"Something went wrong with response: {response}")

        except Exception as e:

            self.get_logger().error("Service call failed %r" % (e,))

    def update_current_action_status(self, status):
        if status == "done":
            self.actions_dict.pop(0)
        else:
            self.actions_dict[0]['status'] = status
        self.current_action_already_printed = False

    def load_strategy(self):
        with open('/home/edog/ros2_ws/src/ia_package/resource/' + self.strategy_filename + '.json') as file:
            self.config = json.load(file)

        self.get_logger().info(f"[Loading Strategy] {self.config['name']} ({self.config['description']})")
        self.get_logger().info(f"[Start] Color: {self.config['color']} | StartPos:({self.config['startingPos']})")

        for strat in self.config['strategy']:
            for action in strat['actions']:
                self.actions_dict.append(
                    {
                        'action': action,
                        'status': 'pending'
                    }
                )


def cast_str_bool(var):
    return var == 'True'


def main(args=None):
    rclpy.init(args=args)
    ia_node = IANode()
    rclpy.spin(ia_node)
    ia_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
