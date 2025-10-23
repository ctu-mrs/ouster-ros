from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch.condition import Condition
from launch.launch_context import LaunchContext
from typing import List

class AndCondition(Condition):
    """
    Condition that evaluates to True only if all sub-conditions evaluate to True.
    """
    
    def __init__(self, conditions: List[Condition]):
        """
        Initialize the AndCondition.
        
        :param conditions: List of conditions that must all be True
        """
        super().__init__()
        self.__conditions = conditions
    
    def evaluate(self, context: LaunchContext) -> bool:
        """
        Evaluate all conditions and return True only if all are True.
        
        :param context: The launch context
        :return: True if all conditions are True, False otherwise
        """
        for condition in self.__conditions:
            if not condition.evaluate(context):
                return False
        return True
    
    def describe(self) -> str:
        """
        Return a description of this condition.
        """
        condition_descriptions = [condition.describe() for condition in self.__conditions]
        return f"AndCondition({', '.join(condition_descriptions)})"

def generate_launch_description():
    declare_loop = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='request loop playback'
    )
    
    declare_play_delay = DeclareLaunchArgument(
        'play_delay',
        default_value='0',
        description='playback start delay in seconds'
    )
    
    declare_play_rate = DeclareLaunchArgument(
        'play_rate',
        default_value='1.0'
    )
    
    declare_uav_name = DeclareLaunchArgument(
        'uav_name',
        description='Override the default namespace of all ouster nodes'
    )
    
    declare_ouster_ns = DeclareLaunchArgument(
        'ouster_ns',
        default_value=[LaunchConfiguration('uav_name'), '/ouster'],
        description='Override the default namespace of all ouster nodes'
    )
    
    declare_timestamp_mode = DeclareLaunchArgument(
        'timestamp_mode',
        default_value='TIME_FROM_ROS_TIME',
        description='method used to timestamp measurements; possible values: {TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, TIME_FROM_PTP_1588, TIME_FROM_ROS_TIME}'
    )
    
    declare_ptp_utc_tai_offset = DeclareLaunchArgument(
        'ptp_utc_tai_offset',
        default_value='-37.0',
        description='UTC/TAI offset in seconds to apply when using TIME_FROM_PTP_1588'
    )
    
    declare_metadata = DeclareLaunchArgument(
        'metadata',
        default_value='',
        description='path to write metadata file when receiving sensor data'
    )
    
    declare_bag_file = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='file name to use for the recorded bag file'
    )
    
    declare_viz = DeclareLaunchArgument(
        'viz',
        default_value='true',
        description='whether to run a rviz'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('ouster_ros'), 'config', 'viz-reliable.rviz']),
        description='optional rviz config file'
    )
    
    declare_sensor_frame = DeclareLaunchArgument(
        'sensor_frame',
        default_value='os_sensor',
        description='sets name of choice for the sensor_frame tf frame, value can not be empty'
    )
    
    declare_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='os_lidar',
        description='sets name of choice for the os_lidar tf frame, value can not be empty'
    )
    
    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame',
        default_value='os_imu',
        description='sets name of choice for the os_imu tf frame, value can not be empty'
    )
    
    declare_point_cloud_frame = DeclareLaunchArgument(
        'point_cloud_frame',
        default_value='',
        description='which frame to be used when publishing PointCloud2 or LaserScan messages. Choose between the value of sensor_frame or lidar_frame, leaving this value empty would set lidar_frame to be the frame used when publishing these messages.'
    )
    
    declare_pub_static_tf = DeclareLaunchArgument(
        'pub_static_tf',
        default_value='false',
        description='when this flag is set to True, the driver will broadcast the TF transforms for the imu/sensor/lidar frames. Prevent the driver from broadcasting TF transforms by setting this parameter to False.'
    )
    
    declare_use_system_default_qos = DeclareLaunchArgument(
        'use_system_default_qos',
        default_value='true',
        description='Use the default system QoS settings'
    )
    
    declare_proc_mask = DeclareLaunchArgument(
        'proc_mask',
        default_value='IMG|PCL|IMU|SCAN',
        description='use any combination of the 4 flags to enable or disable specific processors'
    )
    
    declare_scan_ring = DeclareLaunchArgument(
        'scan_ring',
        default_value='0',
        description='use this parameter in conjunction with the SCAN flag and choose a value the range [0, sensor_beams_count)'
    )
    
    declare_point_type = DeclareLaunchArgument(
        'point_type',
        default_value='original',
        description='point type for the generated point cloud; available options: {original, native, xyz, xyzi, o_xyzi, xyzir}'
    )
    
    declare_organized = DeclareLaunchArgument(
        'organized',
        default_value='true',
        description='generate an organzied point cloud'
    )
    
    declare_destagger = DeclareLaunchArgument(
        'destagger',
        default_value='true',
        description='enable or disable point cloud destaggering'
    )
    
    declare_min_range = DeclareLaunchArgument(
        'min_range',
        default_value='0.0',
        description='minimum lidar range to consider (meters)'
    )
    
    declare_max_range = DeclareLaunchArgument(
        'max_range',
        default_value='100000.0',
        description='maximum lidar range to consider (meters)'
    )
    
    declare_min_scan_valid_columns_ratio = DeclareLaunchArgument(
        'min_scan_valid_columns_ratio',
        default_value='0.0',
        description='The minimum ratio of valid columns for processing the LidarScan [0, 1]'
    )
    
    declare_v_reduction = DeclareLaunchArgument(
        'v_reduction',
        default_value='1',
        description='vertical beam reduction; available options: {1, 2, 4, 8, 16}'
    )
    
    declare_mask_path = DeclareLaunchArgument(
        'mask_path',
        default_value='',
        description='path to an image file that will be used to mask parts of the pointcloud'
    )

    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='true')

    loop = LaunchConfiguration('loop')
    play_delay = LaunchConfiguration('play_delay')
    play_rate = LaunchConfiguration('play_rate')
    uav_name = LaunchConfiguration('uav_name')
    ouster_ns = LaunchConfiguration('ouster_ns')
    timestamp_mode = LaunchConfiguration('timestamp_mode')
    ptp_utc_tai_offset = LaunchConfiguration('ptp_utc_tai_offset')
    metadata = LaunchConfiguration('metadata')
    bag_file = LaunchConfiguration('bag_file')
    viz = LaunchConfiguration('viz')
    rviz_config = LaunchConfiguration('rviz_config')
    sensor_frame = LaunchConfiguration('sensor_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    point_cloud_frame = LaunchConfiguration('point_cloud_frame')
    pub_static_tf = LaunchConfiguration('pub_static_tf')
    use_system_default_qos = LaunchConfiguration('use_system_default_qos')
    proc_mask = LaunchConfiguration('proc_mask')
    scan_ring = LaunchConfiguration('scan_ring')
    point_type = LaunchConfiguration('point_type')
    organized = LaunchConfiguration('organized')
    destagger = LaunchConfiguration('destagger')
    min_range = LaunchConfiguration('min_range')
    max_range = LaunchConfiguration('max_range')
    min_scan_valid_columns_ratio = LaunchConfiguration('min_scan_valid_columns_ratio')
    v_reduction = LaunchConfiguration('v_reduction')
    mask_path = LaunchConfiguration('mask_path')
    
    os_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_sensor_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'os_sensor', [LaunchConfiguration('uav_name'), '/os_sensor']]
    )

    os_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='os_lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'os_lidar', [LaunchConfiguration('uav_name'), '/os_lidar']]
    )

    os_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_imu_tf', 
        arguments=['0', '0', '0', '0', '0', '0', 'os_imu', [LaunchConfiguration('uav_name'), '/os_imu']]
    )

    os_replay_node = Node(
        package='ouster_ros',
        executable='os_replay',
        name='os_replay',
        namespace=ouster_ns,
        output='screen',
        parameters=[{
            'metadata': metadata,
            'use_system_default_qos': use_system_default_qos,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(PythonExpression(["'", metadata, "' != ''"])),
    )

    container = ComposableNodeContainer(
        name='os_container',
        namespace=[uav_name, '/ouster'],
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='ouster_ros',
                plugin='ouster_ros::OusterCloud',
                name='os_cloud',
                namespace=[uav_name, '/ouster'],
                parameters=[{
                    'frame_id_prefix': uav_name,
                    'sensor_frame': sensor_frame,
                    'lidar_frame': lidar_frame,
                    'imu_frame': imu_frame,
                    'point_cloud_frame': point_cloud_frame,
                    'pub_static_tf': pub_static_tf,
                    'timestamp_mode': timestamp_mode,
                    'ptp_utc_tai_offset': ptp_utc_tai_offset,
                    'use_system_default_qos': use_system_default_qos,
                    'proc_mask': proc_mask,
                    'scan_ring': scan_ring,
                    'point_type': point_type,
                    'organized': organized,
                    'destagger': destagger,
                    'min_range': min_range,
                    'max_range': max_range,
                    'v_reduction': v_reduction,
                    'mask_path': mask_path,
                    'min_scan_valid_columns_ratio': min_scan_valid_columns_ratio,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
            ),
            ComposableNode(
                package='ouster_ros',
                plugin='ouster_ros::OusterImage',
                name='os_image',
                namespace=[uav_name, '/ouster'],
                parameters=[{
                    'use_system_default_qos': use_system_default_qos,
                    'proc_mask': proc_mask,
                    'mask_path': mask_path,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
            ),
        ],
    )

    ouster_group = GroupAction([
        os_replay_node,
        container,
    ])

    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ouster_ros'), 'launch', 'rviz.launch.py']),
        launch_arguments={
            'ouster_ns': ouster_ns,
            'rviz_config': rviz_config,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(viz)
    )

    bag_play_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'sleep ', play_delay, '; ',
                'ros2 bag play ', bag_file, ' --clock ',
                PythonExpression(["'--loop' if '", loop, "' == 'true' else ''"]),
                ' --rate ', play_rate,
                ' --remap',
                ' /os_node/metadata:=/ouster/metadata',
                ' /os_node/imu_packets:=/ouster/imu_packets',
                ' /os_node/lidar_packets:=/ouster/lidar_packets',
                ' --qos-profile-overrides-path',
                ' ', PathJoinSubstitution([FindPackageShare('ouster_ros'), 'config', 'metadata-qos-override.yaml'])
            ]
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", bag_file, "' != ''"]))
    )

    return LaunchDescription([
        declare_loop,
        declare_play_delay,
        declare_play_rate,
        declare_uav_name,
        declare_ouster_ns,
        declare_timestamp_mode,
        declare_ptp_utc_tai_offset,
        declare_metadata,
        declare_bag_file,
        declare_viz,
        declare_rviz_config,
        declare_sensor_frame,
        declare_lidar_frame,
        declare_imu_frame,
        declare_point_cloud_frame,
        declare_pub_static_tf,
        declare_use_system_default_qos,
        declare_proc_mask,
        declare_scan_ring,
        declare_point_type,
        declare_organized,
        declare_destagger,
        declare_min_range,
        declare_max_range,
        declare_min_scan_valid_columns_ratio,
        declare_v_reduction,
        declare_mask_path,
        declare_use_sim_time,
        
        os_sensor_tf,
        os_lidar_tf,
        os_imu_tf,
        
        ouster_group,
        rviz_launch,
        bag_play_cmd,
    ])