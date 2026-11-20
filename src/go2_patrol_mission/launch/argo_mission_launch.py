"""Argo v2: the whole mission stack (README_instruction.md, Phase 6).

    ros2 launch go2_patrol_mission argo_mission_launch.py
    argo_cmd "di' a Vito che il caffè è pronto"

Starts: PlanSys2 (monolithic, patrol_domain_v2.pddl), the action executors, gemma_server,
waypoint_markers (ontology.yaml) and argo_mission_bt/mission_node (Groot on port 1666).
Needs the robot side already running (bringup, Nav2 in Docker, llama.cpp, argo_voice_node).

Arguments:
  real     comma list of real executors (default navigate,advance,deliver_message,announce,chat); every other
           action of the domain runs on a fake executor that succeeds after fake_duration_s
  fail_on  comma list of action strings the fake executors must fail (offline tests),
           e.g. "navigate go2 hub_west vito_door"
  groot    true/false
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
REAL_EXECUTORS = {   # action -> executable (README Phase 3); the others are still TODO
    'navigate': 'navigate_action_node',
    'advance': 'advance_action_node',
    'deliver_message': 'deliver_message_action_node',
    'announce': 'announce_action_node',
    'chat': 'chat_action_node',
}


def nodes(context):
    pkg = get_package_share_directory('go2_patrol_mission')
    ontology = os.path.join(pkg, 'config', 'ontology.yaml')
    split = lambda name: [a.strip() for a in LaunchConfiguration(name).perform(context).split(',') if a.strip()]
    real = split('real')
    unknown = [a for a in real if a not in REAL_EXECUTORS]
    if unknown:
        raise RuntimeError(f'executor veri non disponibili: {unknown} (disponibili: {list(REAL_EXECUTORS)})')
    out = []
    chat_time = float(LaunchConfiguration('chat_time').perform(context))
    for action in real:
        params = {'action_name': action, 'ontology_file': ontology}
        if action == 'chat' and chat_time > 0:
            params['chat_time'] = chat_time   # shorter chat for tests (the plan still says 180 s)
        out.append(Node(package='go2_patrol_mission', executable=REAL_EXECUTORS[action],
                        name=f'{action}_executor', output='screen', parameters=[params]))
    fakes = [a for a in ALL_ACTIONS if a not in real]
    if fakes:
        out.append(Node(package='go2_patrol_mission', executable='fake_action_executors',
                        name='fake_action_executors', output='screen',
                        parameters=[{'actions': fakes,
                                     'duration_s': float(LaunchConfiguration('fake_duration_s').perform(context)),
                                     'fail_on': split('fail_on') or ['']}]))
    out.append(Node(package='go2_patrol_mission', executable='gemma_server.py', name='gemma_server',
                    output='screen', parameters=[{
                        'ontology_file': ontology,
                        'gemma_api_base': LaunchConfiguration('gemma_api_base').perform(context)}]))
    out.append(Node(package='go2_patrol_mission', executable='waypoint_markers.py', name='waypoint_markers',
                    output='screen', parameters=[{'ontology_file': ontology}]))
    out.append(Node(package='argo_mission_bt', executable='mission_node', name='argo_mission', output='screen',
                    parameters=[{'ontology_file': ontology,
                                 'groot': LaunchConfiguration('groot').perform(context) == 'true'}]))
    return out


def generate_launch_description():
    pkg = get_package_share_directory('go2_patrol_mission')
    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': os.path.join(pkg, 'pddl', 'patrol_domain_v2.pddl')}.items())
    return LaunchDescription([
        DeclareLaunchArgument('real', default_value='navigate,advance,deliver_message,announce,chat'),
        DeclareLaunchArgument('chat_time', default_value='-1', description='s, -1 = ontology (180)'),
        DeclareLaunchArgument('gemma_api_base', default_value='http://127.0.0.1:8000/v1'),
        DeclareLaunchArgument('fail_on', default_value=''),
        DeclareLaunchArgument('fake_duration_s', default_value='2.0'),
        DeclareLaunchArgument('groot', default_value='true'),
        plansys2,
        OpaqueFunction(function=nodes),
    ])
