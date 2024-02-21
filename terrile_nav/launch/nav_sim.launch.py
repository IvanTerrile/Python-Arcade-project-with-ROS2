from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrile_nav',
            executable='nav_sim',
            name='nav_sim',
        ),
        Node(
            package='terrile_nav',
            executable='robot_logic',
            name='robot_logic',
        ),
        Node(
            package='terrile_nav',
            executable='robot_controller',
            name='robot_controller',
        ),
    ])
