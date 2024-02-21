from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_cannon_simulator',
            executable='cannon_logic_node',
            name='cannon_logic_node'
        ),
        Node(
            package='sofar_cannon_simulator',
            executable='cannon_controller_node',
            name='cannon_controller_node'
        ),
        Node(
            package='sofar_cannon_simulator',
            executable='cannon_sim_node',
            name='cannon_sim_node'
        ),
    ])