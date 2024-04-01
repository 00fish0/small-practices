from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='enemy_estimator_task',
            namespace='enemy_estimator',
            executable='enemy_estimator_task',
            name='enemy_estimator_task'
        )
    ])