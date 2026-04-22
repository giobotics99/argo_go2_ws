import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get directories
    pkg_share = get_package_share_directory('go2_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Paths
    slam_params_default = os.path.join(pkg_share, 'params', 'slam_toolbox_params.yaml')
    nav2_params_default = os.path.join(pkg_share, 'params', 'go2_nav_params_foxy.yaml')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Pointcloud args
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    target_frame = LaunchConfiguration('target_frame')

    # Lifecycle nodes for SLAM and Map Saver
    lifecycle_nodes = ['map_saver']

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_default,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/lidar_points',
        description='PointCloud2 topic from Hesai')

    declare_target_frame = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='Frame to project the LaserScan into')

    # ---- PointCloud Filter (PyTorch/CUDA)
    pc_filter = Node(
        package='go2_nav',
        executable='pointcloud_filter.py',
        name='pointcloud_filter_node',
        output='screen',
        parameters=[{
            'input_topic': pointcloud_topic,
            'output_topic': '/lidar_points_filtered',
            'bounding_box_min': [-0.5, -0.4, -0.2],
            'bounding_box_max': [0.5, 0.4, 0.5],
            'use_sim_time': use_sim_time
        }]
    )

    # ---- PointCloud -> LaserScan
    pc2las = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc2ls_from_cloud',
        output='screen',
        remappings=[('cloud_in', '/lidar_points_filtered'), ('output', '/scan')],
        parameters=[{
            'target_frame': target_frame,
            'transform_tolerance': 0.02,
            'min_height': -0.10,
            'max_height': 0.30,
            'range_min': 0.2,
            'range_max': 30.0,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'use_inf': True
        }]
    )

    # ---- SLAM Toolbox (Online Sync)
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': slam_params_default
        }.items()
    )

    # ---- Navigation
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # ---- Map Saver Server
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[configured_params]
    )

    # ---- Lifecycle Manager for Map Saver
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    # ---- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    
    # Set env var
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_pointcloud_topic)
    ld.add_action(declare_target_frame)
    
    # Add nodes and included launches
    # ld.add_action(pc_filter)
    # ld.add_action(pc2las)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(rviz)

    return ld