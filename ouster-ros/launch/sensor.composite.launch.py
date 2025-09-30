# Copyright 2023 Ouster, Inc.
#

"""Launch ouster nodes using a composite container"""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

import yaml

def execute_script_and_launch(context):
    logs = []
    
    sensor_hostname = LaunchConfiguration("sensor_hostname").perform(context)
    if LaunchConfiguration("udp_dest").perform(context) == '':
        return [LogInfo(msg="'udp_dest' param is not set. Ouster lidar does not know where it should send the data.")]
    else:
        udp_dest = LaunchConfiguration("udp_dest").perform(context)

    combined_ns = LaunchConfiguration('uav_name').perform(context) + '/' + LaunchConfiguration('ouster_ns').perform(context)

    rviz_enable = LaunchConfiguration('viz')
    rviz_enable_arg = DeclareLaunchArgument('viz', default_value='True')

    auto_start = LaunchConfiguration('auto_start')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='True')

    _custom_config_file = LaunchConfiguration('custom_config').perform(context)
    remappings = []

    # pull remapping of the topics out of the yaml file
    if _custom_config_file != '':
        with open(_custom_config_file, 'r') as f:
            yaml_data = yaml.load(f, Loader=yaml.FullLoader)

            for prefix in yaml_data:
                #prefix = '/' + combined_ns + '/os_cloud'
                if prefix in yaml_data and 'ros__parameters' in yaml_data[prefix] and 'remappings' in yaml_data[prefix]['ros__parameters']:
                    remappings_subyaml = yaml_data[prefix]['ros__parameters']['remappings']
                    for orig_name in remappings_subyaml:
                        new_name = remappings_subyaml[orig_name]
                        remappings.append((orig_name, new_name))

    logs.append(LogInfo(msg=f"custom config file: {_custom_config_file}"))
    logs.append(LogInfo(msg=f"remappings:"))
    for remapping in remappings:
        logs.append(LogInfo(msg=f"\t{remapping[0]} -> {remapping[1]}"))

    default_config = LaunchConfiguration('default_config')
    default_config_arg = DeclareLaunchArgument('default_config', default_value=str(Path(get_package_share_directory('ouster_ros')) / 'config' / 'os_sensor_cloud_image_params.yaml'), description='')

    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=combined_ns,
        parameters=[
            default_config,
            _custom_config_file,
            {'auto_start': auto_start},
            {
             'sensor_hostname': sensor_hostname,
             'udp_dest': udp_dest,
             #'mtp_dest': mtp_dest
            }
        ],
        remappings=remappings
    )

    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=combined_ns,
        parameters=[default_config, _custom_config_file, {'frame_id_prefix': EnvironmentVariable('UAV_NAME')}],
        #parameters=[default_config, _custom_config_file, {'use_namespace_as_frame_id_prefix': True}],
        remappings=remappings
    )

    enable_image = LaunchConfiguration('enable_image')
    enable_image_arg = DeclareLaunchArgument('enable_image', default_value='False')

    os_image = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterImage',
        name='os_image',
        namespace=combined_ns,
        parameters=[default_config, _custom_config_file],
        remappings=remappings,
        condition=IfCondition(enable_image)
    )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace=combined_ns,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
            os_image
        ],
        output='screen'
    )

    rviz_launch_file_path = \
        Path(get_package_share_directory('ouster_ros')) / 'launch' / 'rviz.launch.py'
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(rviz_launch_file_path)]),
        condition=IfCondition(rviz_enable)
    )

    return logs + [
        default_config_arg,
        enable_image_arg,
        rviz_enable_arg,
        auto_start_arg,
        rviz_launch,
        os_container
    ]

def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """

    return launch.LaunchDescription([
        DeclareLaunchArgument('uav_name', default_value=EnvironmentVariable('UAV_NAME'), description='Camera namespace (used for node name and topic namespace)'),
        DeclareLaunchArgument('ouster_ns', default_value='ouster'),
        DeclareLaunchArgument('custom_config', default_value=str(Path(get_package_share_directory('ouster_ros')) / 'config' / 'os_sensor_cloud_image_params.yaml'), description='Camera namespace (used for node name and topic namespace)'),
        DeclareLaunchArgument('sensor_hostname', default_value='', description="IP address of the Ouster lidar"),
        DeclareLaunchArgument('udp_dest', default_value='', description='IP address of the machine, to which the data should be sent.'),
        OpaqueFunction(function=execute_script_and_launch)
    ])
