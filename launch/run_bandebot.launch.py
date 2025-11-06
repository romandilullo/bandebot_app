from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mulita_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mulita'),
                'launch',
                'run_mulita.launch.py'
            )
        )
    )

    bandebot_node = Node(
        package='bandebot_app',
        executable='bandebot_node',
        name='bandebot_node',
        output='screen'
    )

    catering_node = Node(
        package='bandebot_app',
        executable='catering_node',
        name='catering_node',
        output='screen'
    )

    head_display_node = Node(
            package='bandebot_display_controller',
            executable='display_controller_node',
            name='display_controller',
            output='screen',
            parameters=[{
                'serial_port': '/dev/display_controller',
                'serial_baudrate': 115200,
            }],
            remappings=[
                ('display/text', '/bandebot/display/text'),
            ]
        )

    return LaunchDescription([
        mulita_launch,
        bandebot_node,
        # head_display_node,
        catering_node
    ])