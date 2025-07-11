import os
from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        'src', 'config', 'params.yaml'
    )

    bag_reading_node = Node(
        package='bag_reading',
        executable='bag_reading_node',
        name='bag_reading_node',
        output='screen',
        parameters=[
            launch_ros.parameter_descriptions.ParameterFile(
                param_file=config,
                allow_substs=True
            )
        ]
    )

    return LaunchDescription([
        bag_reading_node
    ])