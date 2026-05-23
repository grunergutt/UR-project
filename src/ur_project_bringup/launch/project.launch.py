"""
Hoved-launch-fil for UR-prosjektet.
Starter kameranode, bevegelsesnode og koordineringsnode
med felles konfigurasjon fra project_params.yaml.

Bruk:
  ros2 launch ur_project_bringup project.launch.py
  ros2 launch ur_project_bringup project.launch.py ur_type:=ur5e robot_ip:=143.25.150.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Hent config-fil ---
    bringup_dir = get_package_share_directory('ur_project_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'project_params.yaml')

    # --- Launch-argumenter (kan overstyres fra terminal) ---
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

    # Parameter-overrides fra launch-argumenter
    # Disse overstyrer verdiene fra YAML-filen.
    param_overrides = {
        'robot.ur_type': LaunchConfiguration('ur_type'),
        'robot.robot_ip': LaunchConfiguration('robot_ip'),
    }

    # --- Noder ---
    cube_vision_node = Node(
        package='cube_vision',
        executable='cube_detector',
        name='cube_detector',
        parameters=[LaunchConfiguration('config_file'), param_overrides],
        output='screen',
    )

    coordinate_transformer_node = Node(
        package='cube_vision',
        executable='coordinate_transformer',
        name='coordinate_transformer',
        parameters=[LaunchConfiguration('config_file'), param_overrides],
        output='screen',
    )

    robot_control_node = Node(
        package='robot_control',
        executable='robot_mover',
        name='robot_mover',
        parameters=[LaunchConfiguration('config_file'), param_overrides],
        output='screen',
    )

    coordinator_node = Node(
        package='robot_control',
        executable='coordinator',
        name='coordinator',
        parameters=[LaunchConfiguration('config_file'), param_overrides],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        ur_type_arg,
        robot_ip_arg,
        LogInfo(msg='===== Starter UR-prosjekt ====='),
        cube_vision_node,
        coordinate_transformer_node,
        robot_control_node,
        coordinator_node,
    ])

