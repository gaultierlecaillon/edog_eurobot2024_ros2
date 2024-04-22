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

    actuator_service = Node(
        package="actuator_package",
        executable="actuator_service",
        name="actuator_service"
    )

    ia = Node(
        package="ia_package",
        executable="ia_node",
        name="ia_node",
        parameters=[{"strategy_filename": strategy_filename}]
    )

    voice = Node(
        package="ia_package",
        executable="voice_service",
        name="voice_service"
    )

    ld.add_action(tirette_publisher)
    ld.add_action(lidar_publisher)
    ld.add_action(motion_service)
    ld.add_action(actuator_service)
    ld.add_action(ia)
    ld.add_action(voice)

    return ld
