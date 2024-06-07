from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='team5',
            description='Robot''s namespace.'
        ),
        Node(
            package='team5_project',
            executable='find_tennis_ball',
            name='Find_Tennis_Ball_Node',
            namespace=LaunchConfiguration('namespace'),
            arguments=['--ros-args'],
            parameters=[]
        ),
    ])
