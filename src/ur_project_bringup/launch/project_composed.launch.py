"""
project_composed.launch.py – Composition-versjon av launch.

Kjører vision-noder (cube_detector + coordinate_transformer) i én prosess
og control-noder (robot_mover + coordinator) i én annen.
Totalt 2 prosesser i stedet for 4 → lavere overhead, raskere kommunikasjon.

Bruk:
  ros2 launch ur_project_bringup project_composed.launch.py
  ros2 launch ur_project_bringup project_composed.launch.py ur_type:=ur3e robot_ip:=143.25.150.4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('ur_project_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'project_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path til YAML-konfigurasjonsfilen'
    )

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type UR-robot (ur3, ur3e, ur5, ur5e, ur10, ur10e)'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='143.25.150.5',
        description='IP-adresse til robotkontrolleren'
    )

    # NB: Param-overrides fra launch-argumenter virker ikke med composition
    # fordi nodene inni har egne navn (cube_detector, coordinator, osv.)
    # som ikke matcher prosessnavnet. Bruk YAML-filen for å endre parametre.
    config_file = LaunchConfiguration('config_file')

    # Vision-prosess: cube_detector + coordinate_transformer i én prosess
    vision_composed = Node(
        package='cube_vision',
        executable='vision_composed',
        name='vision_composed',
        parameters=[config_file],
        output='screen',
    )

    # Control-prosess: robot_mover + coordinator i én prosess
    control_composed = Node(
        package='robot_control',
        executable='control_composed',
        name='control_composed',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        ur_type_arg,
        robot_ip_arg,
        LogInfo(msg='===== Starter UR-prosjekt (composition) ====='),
        vision_composed,
        control_composed,
    ])

