from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('hdl_global_localization')
    params_file = os.path.join(pkg_dir, 'config', 'hdl_global_localization.param.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='hdl_global_localization',
            executable='hdl_global_localization_node',
            name='hdl_global_localization',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
