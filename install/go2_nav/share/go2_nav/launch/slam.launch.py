import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('go2_nav')   # <<< cambia se il file è in un altro package
    slam_params = os.path.join(pkg_share, 'params', 'slam_toolbox_params.yaml')
    rviz_cfg    = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')

    # ---- Launch args
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    use_voxel        = LaunchConfiguration('use_voxel')
    leaf_size        = LaunchConfiguration('leaf_size')
    target_frame     = LaunchConfiguration('target_frame')
    min_height       = LaunchConfiguration('min_height')
    max_height       = LaunchConfiguration('max_height')
    range_min        = LaunchConfiguration('range_min')
    range_max        = LaunchConfiguration('range_max')

    decl_args = [
        DeclareLaunchArgument('pointcloud_topic', default_value='/lidar_points',
                              description='PointCloud2 topic from Hesai'),
        DeclareLaunchArgument('use_voxel', default_value='false',
                              description='Enable voxel filtering before LaserScan'),
        DeclareLaunchArgument('leaf_size', default_value='0.05',
                              description='VoxelGrid leaf size (m)'),
        DeclareLaunchArgument('target_frame', default_value='base_link',
                              description='Frame to project the LaserScan into'),
        DeclareLaunchArgument('min_height', default_value='-0.10',
                              description='Min Z clip (m)'),
        DeclareLaunchArgument('max_height', default_value='0.30',
                              description='Max Z clip (m)'),
        DeclareLaunchArgument('range_min', default_value='0.2',
                              description='LaserScan min range (m)'),
        DeclareLaunchArgument('range_max', default_value='30.0',
                              description='LaserScan max range (m)'),
    ]

    # ---- PointCloud -> LaserScan (diretto dalla Hesai)
    pc2las = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc2ls_from_cloud',
        output='screen',
        remappings=[('input', pointcloud_topic), ('output', '/scan')],
        parameters=[{
            'target_frame': target_frame,
            'transform_tolerance': 0.02,
            'min_height': min_height,
            'max_height': max_height,
            'range_min': range_min,
            'range_max': range_max,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'use_inf': True
        }],
        condition=UnlessCondition(use_voxel)
    )

    # ---- SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',   # 'async_slam_toolbox_node' se preferisci
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params,
            {'base_frame': 'base_link'}
            ],
        remappings=[('/scan', '/scan')],
    )

    # ---- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    ld = LaunchDescription(decl_args)
    ld.add_action(pc2las)
    ld.add_action(slam)
    ld.add_action(rviz)
    return ld