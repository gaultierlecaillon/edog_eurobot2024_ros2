<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="https://i.postimg.cc/m2LKh4Fk/banner.png" alt="Logo">
  </a>

  <h3 align="center">Edog Eurobot 2024 ROS2 packages </h3>

  <p align="center">
    A collection of ROS2 packages to control the robot Edog 2024
    </p>
</div>


## Getting Started

## ROS2 Cheat Sheet

Create a python package
```shell
ros2 pkg create hardware_package --build-type ament_python --dependencies rclpy
```

Build a specific package
```shell
colcon build --packages-select hardware_package
colcon build --allow-overriding actuator_package --packages-select actuator_package 

```

Build a specific package with symlink
```shell
colcon build --packages-select hardware_package --symlink-install
```
FUlll
```shell
colcon build --allow-overriding actuator_package --packages-select actuator_package --symlink-install
```


Create a Node

Node are created in ~/ros2_ws/src/hardware_package/hardware_package/ folder
```shell
touche reset1_node
```

Python Node exemple with switch logic input:
```python#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import board


class Reset1Node(Node):

    def __init__(self):
        super().__init__("reset1_node")

        # Pi setup
        self.tiretteGPIO = 14
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tiretteGPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # ros2 setup
        self.get_logger().info("Node reset1_node starting")
        self.create_timer(0.25, self.timer_callback)

    def timer_callback(self):
        reset1_state = not GPIO.input(self.tiretteGPIO)
        self.get_logger().info("Reset1:" + str(reset1_state))


def main(args=None):
    rclpy.init(args=args)
    node = Reset1Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```
