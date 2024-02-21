from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_printer_simulator',
            executable='printer_sim_node',
            name='printer_sim_node',
        ),
        Node(
            package='sofar_printer_simulator',
            executable='shape_service',
            name='shape_service',
        ),
        Node(
            package='sofar_printer_simulator',
            executable='robot_logic',
            name='robot_logic',
            parameters=[
                {'radius': 100.0},
                {'n_vertices': 8},
            ],
        ),
        Node(
            package='sofar_printer_simulator',
            executable='motor_x',
            name='motor_x',
        ),
        Node(
            package='sofar_printer_simulator',
            executable='motor_y',
            name='motor_y',
        ),
    ])
