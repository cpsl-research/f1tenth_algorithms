import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('auto_py'),
        'config', 'auto_example.yaml'
    )

    ld = LaunchDescription([
        launch.actions.DeclareLaunchArgument('auto_config', default_value=parameters_file),
    ])

    ld.add_action(launch_ros.actions.Node(
            package='auto_py', executable='straight_control',
            parameters=[launch.substitutions.LaunchConfiguration('auto_config')]))


    return ld
