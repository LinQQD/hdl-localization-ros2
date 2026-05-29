import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _as_bool(value: str) -> bool:
    return value.lower() in ('true', '1', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    pkg_loc = get_package_share_directory('hdl_localization')
    pkg_gloc = get_package_share_directory('hdl_global_localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    points_topic = LaunchConfiguration('points_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id')
    send_tf_transforms = LaunchConfiguration('send_tf_transforms')
    globalmap_pcd = LaunchConfiguration('globalmap_pcd')
    use_global_localization = LaunchConfiguration('use_global_localization')
    use_imu = LaunchConfiguration('use_imu')
    invert_imu_acc = LaunchConfiguration('invert_imu_acc')
    invert_imu_gyro = LaunchConfiguration('invert_imu_gyro')
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    # Resolve init-pose launch args to plain Python values so ComposableNode receives
    # them reliably (LaunchConfiguration inside ParameterValue dict is often ignored).
    specify_init_pose = _as_bool(LaunchConfiguration('specify_init_pose').perform(context))
    init_pos_x = float(LaunchConfiguration('init_pos_x').perform(context))
    init_pos_y = float(LaunchConfiguration('init_pos_y').perform(context))
    init_pos_z = float(LaunchConfiguration('init_pos_z').perform(context))
    init_ori_w = float(LaunchConfiguration('init_ori_w').perform(context))
    init_ori_x = float(LaunchConfiguration('init_ori_x').perform(context))
    init_ori_y = float(LaunchConfiguration('init_ori_y').perform(context))
    init_ori_z = float(LaunchConfiguration('init_ori_z').perform(context))
    enable_auto_relocalize_monitor = _as_bool(LaunchConfiguration('enable_auto_relocalize_monitor').perform(context))
    auto_relocalize_error_threshold = float(LaunchConfiguration('auto_relocalize_error_threshold').perform(context))
    auto_relocalize_cooldown = float(LaunchConfiguration('auto_relocalize_cooldown').perform(context))
    base_to_livox_x = float(LaunchConfiguration('base_to_livox_x').perform(context))
    base_to_livox_y = float(LaunchConfiguration('base_to_livox_y').perform(context))
    base_to_livox_z = float(LaunchConfiguration('base_to_livox_z').perform(context))
    base_to_livox_qx = float(LaunchConfiguration('base_to_livox_qx').perform(context))
    base_to_livox_qy = float(LaunchConfiguration('base_to_livox_qy').perform(context))
    base_to_livox_qz = float(LaunchConfiguration('base_to_livox_qz').perform(context))
    base_to_livox_qw = float(LaunchConfiguration('base_to_livox_qw').perform(context))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gloc, 'launch', 'hdl_global_localization.launch.py')
            ),
            launch_arguments=[('use_sim_time', use_sim_time)],
            condition=IfCondition(use_global_localization),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox_tf',
            arguments=[
                '--x', str(base_to_livox_x),
                '--y', str(base_to_livox_y),
                '--z', str(base_to_livox_z),
                '--qx', str(base_to_livox_qx),
                '--qy', str(base_to_livox_qy),
                '--qz', str(base_to_livox_qz),
                '--qw', str(base_to_livox_qw),
                '--frame-id', odom_child_frame_id,
                '--child-frame-id', 'livox_frame',
            ],
            parameters=[{'use_sim_time': sim_time_param}],
        ),
        ComposableNodeContainer(
            name='hdl_localization_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            parameters=[{'use_sim_time': sim_time_param}],
            composable_node_descriptions=[
                ComposableNode(
                    package='hdl_localization',
                    plugin='hdl_localization::GlobalmapServerNodelet',
                    name='globalmap_server',
                    parameters=[{
                        'use_sim_time': sim_time_param,
                        'globalmap_pcd': ParameterValue(globalmap_pcd, value_type=str),
                        'convert_utm_to_local': True,
                        'downsample_resolution': 0.1,
                    }],
                ),
                ComposableNode(
                    package='hdl_localization',
                    plugin='hdl_localization::HdlLocalizationNodelet',
                    name='hdl_localization',
                    remappings=[
                        ('/velodyne_points', points_topic),
                        ('/gpsimu_driver/imu_data', imu_topic),
                    ],
                    parameters=[{
                        'use_sim_time': sim_time_param,
                        'use_imu': ParameterValue(use_imu, value_type=bool),
                        'invert_acc': ParameterValue(invert_imu_acc, value_type=bool),
                        'invert_gyro': ParameterValue(invert_imu_gyro, value_type=bool),
                        'cool_time_duration': 0.2,
                        'enable_robot_odometry_prediction': False,
                        'send_tf_transforms': ParameterValue(send_tf_transforms, value_type=bool),
                        'odom_child_frame_id': ParameterValue(odom_child_frame_id, value_type=str),
                        'robot_odom_frame_id': ParameterValue(odom_child_frame_id, value_type=str),
                        'reg_method': 'NDT_OMP',
                        'ndt_neighbor_search_method': 'DIRECT7',
                        'ndt_neighbor_search_radius': 4.0,
                        'ndt_resolution': 0.5,
                        'downsample_resolution': 0.1,
                        'specify_init_pose': specify_init_pose,
                        'init_pos_x': init_pos_x,
                        'init_pos_y': init_pos_y,
                        'init_pos_z': init_pos_z,
                        'init_ori_w': init_ori_w,
                        'init_ori_x': init_ori_x,
                        'init_ori_y': init_ori_y,
                        'init_ori_z': init_ori_z,
                        'use_global_localization': ParameterValue(use_global_localization, value_type=bool),
                        'enable_auto_relocalize_monitor': enable_auto_relocalize_monitor,
                        'auto_relocalize_error_threshold': auto_relocalize_error_threshold,
                        'auto_relocalize_cooldown': auto_relocalize_cooldown,
                    }],
                ),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_loc = get_package_share_directory('hdl_localization')
    default_map = os.path.join(pkg_loc, 'data', 'scans8.pcd')

    if not os.path.isfile(default_map):
        raise RuntimeError(
            f'Global map not found: {default_map}\n'
            'Put map.pcd in src/hdl_localization/data/ then run:\n'
            '  colcon build --packages-select hdl_localization --symlink-install'
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Must be true when playing rosbag with --clock. '
                        'Note: matching Time in logs may show 0 ms with sim time because '
                        'the clock does not advance within one callback; this is expected.'),
        SetParameter(name='use_sim_time', value=sim_time_param),

        DeclareLaunchArgument('points_topic', default_value='/livox/pointcloud2'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('robot_odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('odom_child_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('send_tf_transforms', default_value='false'),
        DeclareLaunchArgument(
            'globalmap_pcd',
            default_value=default_map,
            description='PCD under <pkg>/share/<pkg>/data/scans8.pcd',
        ),
        DeclareLaunchArgument('use_global_localization', default_value='true'),
        DeclareLaunchArgument(
            'enable_auto_relocalize_monitor', default_value='false',
            description='If true, call /relocalize when scan matching RMSE exceeds threshold.'),
        DeclareLaunchArgument('auto_relocalize_error_threshold', default_value='0.2'),
        DeclareLaunchArgument('auto_relocalize_cooldown', default_value='5.0'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('invert_imu_acc', default_value='false'),
        DeclareLaunchArgument('invert_imu_gyro', default_value='false'),
        DeclareLaunchArgument(
            'specify_init_pose', default_value='true',
            description='If true, use init_pos_* / init_ori_* below. If false, wait for RViz 2D Pose Estimate.',
        ),
        DeclareLaunchArgument(
            'init_pos_x', default_value='0.0',
            description='Initial x (m) in map. Verify in node log: "init position (map)".',
        ),
        DeclareLaunchArgument('init_pos_y', default_value='0.0'),
        DeclareLaunchArgument('init_pos_z', default_value='0.0'),
        DeclareLaunchArgument('init_ori_w', default_value='1.0'),
        DeclareLaunchArgument('init_ori_x', default_value='0.0'),
        DeclareLaunchArgument('init_ori_y', default_value='0.0'),
        DeclareLaunchArgument('init_ori_z', default_value='0.0'),
        DeclareLaunchArgument('base_to_livox_x', default_value='-0.011000'),
        DeclareLaunchArgument('base_to_livox_y', default_value='-0.023290'),
        DeclareLaunchArgument('base_to_livox_z', default_value='0.044120'),
        DeclareLaunchArgument('base_to_livox_qx', default_value='0.0'),
        DeclareLaunchArgument('base_to_livox_qy', default_value='0.258819'),
        DeclareLaunchArgument('base_to_livox_qz', default_value='0.0'),
        DeclareLaunchArgument('base_to_livox_qw', default_value='0.965926'),

        OpaqueFunction(function=_launch_setup),
    ])
