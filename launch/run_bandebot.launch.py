from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for all nodes. Leave empty for no namespace.",
        )
    )

    # Get namespace configuration
    namespace = LaunchConfiguration("namespace")
 
    mulita_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mulita'),
                'launch',
                'run_mulita.launch.py'
            )
        ),
        launch_arguments={'namespace': namespace}.items()
    )

    bandebot_node = Node(
        package='bandebot_app',
        executable='bandebot_node',
        name='bandebot_node',
        namespace=namespace,
        output='screen'
    )

    catering_node = Node(
        package='bandebot_app',
        executable='catering_node',
        name='catering_node',
        namespace=namespace,
        output='screen'
    )

    head_display_node = TimerAction(
        period=6.0,  # Wait 6 seconds before launching (after ros2_control finishes)
        actions=[
            Node(
                package='bandebot_display_controller',
                executable='display_controller_node',
                name='display_controller',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'serial_port': '/dev/display_controller',
                    'serial_baudrate': 115200,
                }],
                remappings=[
                    ('display/text', 'bandebot/display/text'),
                ]
            )
        ]
    )

    return LaunchDescription(declared_arguments + [
        mulita_launch,
        bandebot_node,
        head_display_node,
        catering_node
    ])