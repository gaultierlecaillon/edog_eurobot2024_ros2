from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    strategy_filename = LaunchConfiguration('strat', default='strat')


    tirette_publisher = Node(
        package="hardware_package",
        executable="tirette_publisher",
        name="tirette_publisher"
    )

    lidar_publisher = Node(
        package="control_package",
        executable="lidar_filter",
        name="lidar_filter"
    )

    motion_service = Node(
        package="control_package",
        executable="motion_service",
        name="motion_service"
    )

    arm_service = Node(
        package="actuator_package",
        executable="arm_service",
        name="arm_service"
    )

    ia = Node(
        package="ia_package",
        executable="ia_node",
        name="ia_node",
        parameters=[{"strategy_filename": strategy_filename}]
    )

    ld.add_action(tirette_publisher)
    ld.add_action(lidar_publisher)
    ld.add_action(motion_service)
    ld.add_action(arm_service)
    ld.add_action(ia)

    return ld
