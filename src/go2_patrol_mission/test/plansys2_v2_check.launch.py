"""PlanSys2 (monolithic) with patrol_domain_v2, real and fake executors (tutorial steps 5 and 8).

    ros2 launch go2_patrol_mission plansys2_v2_check.launch.py                 # all 8 actions fake
    ros2 launch go2_patrol_mission plansys2_v2_check.launch.py real:=navigate,advance
        -> real navigate (Nav2 goals from ontology.yaml!) and advance; the other actions fake

Arguments: real (comma list of real executors: navigate, advance, inspect_room), duration_s
(seconds each fake action takes). Only "navigate" moves the robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ALL_ACTIONS = ['navigate', 'announce', 'chat', 'deliver_message', 'inspect_room',
               'ask_permission', 'recharge', 'advance']
# action name -> (executable, extra parameters) of the real executors available so far
REAL_EXECUTORS = {
    'navigate': ('navigate_action_node', {}),
    'advance': ('advance_action_node', {}),
    'deliver_message': ('deliver_message_action_node', {}),
    'announce': ('announce_action_node', {}),
    'chat': ('chat_action_node', {}),
}


def executors(context):
    pkg_dir = get_package_share_directory('go2_patrol_mission')
    real = [a.strip() for a in LaunchConfiguration('real').perform(context).split(',') if a.strip()]
    unknown = [a for a in real if a not in REAL_EXECUTORS]
    if unknown:
        raise RuntimeError(f'executor veri non ancora disponibili: {unknown} (disponibili: {list(REAL_EXECUTORS)})')
    duration_s = float(LaunchConfiguration('duration_s').perform(context))
    nodes = []
    fakes = [a for a in ALL_ACTIONS if a not in real]
    if fakes:
        nodes.append(Node(package='go2_patrol_mission', executable='fake_action_executors',
                          name='fake_action_executors', output='screen',
                          parameters=[{'duration_s': duration_s, 'actions': fakes}]))
    for action in real:
        executable, params = REAL_EXECUTORS[action]
        params = dict(params, action_name=action,
                      ontology_file=os.path.join(pkg_dir, 'config', 'ontology.yaml'))
        nodes.append(Node(package='go2_patrol_mission', executable=executable, name=f'{action}_executor',
                          output='screen', parameters=[params]))
    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_patrol_mission')
    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': os.path.join(pkg_dir, 'pddl', 'patrol_domain_v2.pddl')}.items())

    markers = Node(package='go2_patrol_mission', executable='waypoint_markers.py', name='waypoint_markers',
                   output='screen',
                   parameters=[{'ontology_file': os.path.join(pkg_dir, 'config', 'ontology.yaml')}])

    return LaunchDescription([
        DeclareLaunchArgument('real', default_value='', description='comma list of real executors'),
        DeclareLaunchArgument('duration_s', default_value='1.0', description='seconds each fake action takes'),
        plansys2,
        markers,
        OpaqueFunction(function=executors),
    ])
