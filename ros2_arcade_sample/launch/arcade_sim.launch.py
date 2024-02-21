from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_arcade_sample',
            executable='sim_node',
            name='sim_node',
        ),
        Node(
            package='ros2_arcade_sample',
            executable='controller',
            name='controller',
        ),
        Node(
            package='ros2_arcade_sample',
            executable='set_goal',
            name='set_goal',
            parameters=[
                {'goal_x': 500  },
                {'goal_y': 150},
            ],
        ),
    ])