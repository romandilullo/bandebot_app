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

    promobot_node = Node(
        package='bandebot_app',
        executable='promobot_node',
        name='promobot_node',
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



    return LaunchDescription(declared_arguments + [
        mulita_launch,
        promobot_node,
        catering_node
    ])