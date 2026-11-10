import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_patrol_mission')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    waypoints_file = os.path.join(pkg_dir, 'config', 'waypoints.yaml')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': pkg_dir + '/pddl/patrol_domain.pddl',
          'namespace': namespace
          }.items())

    navigate_cmd = Node(
        package='go2_patrol_mission',
        executable='navigate_action_node',
        name='navigate_action_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'action_name': 'navigate',
            'waypoints_file': waypoints_file
        }])

    inspect_cmd = Node(
        package='go2_patrol_mission',
        executable='inspect_action_node',
        name='inspect_action_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'action_name': 'inspect'
        }])
        
    gemma_bridge_cmd = Node(
        package='go2_patrol_mission',
        executable='gemma_bridge.py',
        name='gemma_bridge',
        namespace=namespace,
        output='screen'
    )

    waypoint_markers_cmd = Node(
        package='go2_patrol_mission',
        executable='waypoint_markers.py',
        name='waypoint_markers',
        namespace=namespace,
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file
        }])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    ld.add_action(plansys2_cmd)
    ld.add_action(navigate_cmd)
    ld.add_action(inspect_cmd)
    ld.add_action(gemma_bridge_cmd)
    ld.add_action(waypoint_markers_cmd)

    return ld
