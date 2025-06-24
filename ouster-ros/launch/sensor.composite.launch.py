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
from launch.substitutions import LaunchConfiguration

import os
import subprocess

def run_ouster_discovery(script_path, quiet=False):
    try:
        # Run the script and capture output
        result = subprocess.run(
            ["python3", script_path, '-q' if quiet else ''],
            capture_output=True,
            text=True, 
            check=True
        )
        
        # Get the output (success case)
        output = result.stdout
        print("Script output:")
        output = output.split(' ')
        print(output)
        
        return output
        
    except subprocess.CalledProcessError as e:
        if not quiet:
            # When check=True, the exception contains the result info
            print(f"Script failed with return code {e.returncode}")
            #print(f"Error output: {e.stderr}")
            #print(f"Standard output: {e.stdout}")  # You can also access stdout from the exception
            print(f"Discovery script output:\n{e.stdout}")
        return []

def execute_script_and_launch(context):
    package_dir = get_package_share_directory('ouster_ros')
    script_path = os.path.join(package_dir, 'scripts', 'ouster_ip_route.py')
    
    sensor_hostname = None
    udp_dest = None
    
    if LaunchConfiguration("ouster_ip").perform(context) == '':
        print("User did not provide ip address of the Ouster lidar. Running automatic discovery script.")
        # Try to discover the Ouster. If not successful, then run discovery again, but with logging enabled, so the user can see the error.
        discovery_data = run_ouster_discovery(script_path, quiet=True)
        if len(discovery_data) == 0:
            run_ouster_discovery(script_path, quiet=False)
            
        #print("discovery_data: ", discovery_data)
        
        if len(discovery_data) == 3:
            sensor_hostname = discovery_data[1]
            udp_dest = discovery_data[2]
        else:
            return [LogInfo(msg="Automatic discovery of the Ouster lidar failed. You can try setting sensor ip and destination address manually.")]
    else:
        sensor_hostname = LaunchConfiguration("ouster_ip").perform(context)
        if LaunchConfiguration("udp_dest").perform(context) == '':
            return [LogInfo(msg="'udp_dest' param is not set. Ouster lidar does not know where it should send the data.")]
        else:
            udp_dest = LaunchConfiguration("udp_dest").perform(context)
        
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'os_sensor_cloud_image_params.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster')

    rviz_enable = LaunchConfiguration('viz')
    rviz_enable_arg = DeclareLaunchArgument('viz', default_value='True')

    auto_start = LaunchConfiguration('auto_start')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='True')
        
    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_ns,
        parameters=[
            params_file,
            {'auto_start': auto_start},
            {
             'sensor_hostname': sensor_hostname,
             'udp_dest': udp_dest,
             #'mtp_dest': mtp_dest
            }
        ]
    )

    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[params_file]
    )

    os_image = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterImage',
        name='os_image',
        namespace=ouster_ns,
        parameters=[params_file]
    )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace=ouster_ns,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
            os_image
        ],
        output='screen',
    )

    rviz_launch_file_path = \
        Path(ouster_ros_pkg_dir) / 'launch' / 'rviz.launch.py'
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(rviz_launch_file_path)]),
        condition=IfCondition(rviz_enable)
    )

    return [
        params_file_arg,
        ouster_ns_arg,
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
        DeclareLaunchArgument('ouster_ip', default_value='', description="IP address of the Ouster lidar"),
        DeclareLaunchArgument('udp_dest', default_value='', description='IP address of the machine, to which the data should be sent.'),
        OpaqueFunction(function=execute_script_and_launch)
    ])