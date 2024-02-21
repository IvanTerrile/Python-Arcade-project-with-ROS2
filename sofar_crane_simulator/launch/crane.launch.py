# A simple ROS launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_crane_simulator',
            namespace='sofar_crane_simulator',
            executable='crane_sim_node',
        ),
        Node(
            package='sofar_crane_simulator',
            namespace='sofar_crane_simulator',
            executable='robot_logic',
        ),
        Node(
            package='sofar_crane_simulator',
            namespace='sofar_crane_simulator',
            executable='motor_x',
        ),
        Node(
            package='sofar_crane_simulator',
            namespace='sofar_crane_simulator',
            executable='motor_y',
        ),
    ]
)