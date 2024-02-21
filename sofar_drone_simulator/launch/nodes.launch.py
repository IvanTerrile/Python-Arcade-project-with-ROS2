from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_drone_simulator',
            executable='drone_controller_node',
            name='drone_controller_node'
        ),
        Node(
            package='sofar_drone_simulator',
            executable='drone_sim_node',
            name='drone_sim_node'
        ),
        # Node(
        #     package='sofar_drone_simulator',
        #     executable='drone_logic_node',
        #     name='drone_logic_node',
        # )
    ])
