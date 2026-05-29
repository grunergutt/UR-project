"""
Hoved-launch-fil for UR-prosjektet.
Starter kameradriver, bevegelsesnode og koordineringsnode
med felles konfigurasjon fra project_params.yaml.

Bruk:
  ros2 launch ur_project_bringup project.launch.py
  ros2 launch ur_project_bringup project.launch.py ur_type:=ur5e robot_ip:=143.25.150.5
  ros2 launch ur_project_bringup project.launch.py video_device:=/dev/video0
  ros2 launch ur_project_bringup project.launch.py show_raw_camera:=true   # debug: viser rå kamerabilde
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Hent config-fil
    bringup_dir = get_package_share_directory('ur_project_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'project_params.yaml')

    # Launch-argumenter (kan overstyres fra terminal)
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

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video3',
        description='V4L2-enhet for kamera (f.eks. /dev/video0, /dev/video2)'
    )

    show_raw_camera_arg = DeclareLaunchArgument(
        'show_raw_camera',
        default_value='false',
        description='Åpne eget vindu med rå kamerabilde (debug). Bruk show_raw_camera:=true.'
    )

    # Parameter-overrides fra launch-argumenter
    # Disse overstyrer verdiene fra YAML-filen.
    param_overrides = {
        'robot.ur_type': LaunchConfiguration('ur_type'),
        'robot.robot_ip': LaunchConfiguration('robot_ip'),
    }

    # Kameradriver
    # Starter v4l2_camera_node og remapper image_raw til /camera/color/image_raw
    # slik at cube_detector finner bildestrømmen på riktig topic.
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
        }],
        remappings=[('image_raw', '/camera/color/image_raw')],
        output='screen',
    )

    # Vision-noder
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

    # Robot-noder
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

    # Kameravisning: prosessert bilde med deteksjoner
    # Viser /cube_detector/debug_image (kamera + innmalte kubeposisjoner).
    # Krever: sudo apt install ros-<distro>-image-view
    debug_image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='debug_image_view',
        remappings=[('image', '/cube_detector/debug_image')],
        output='screen',
    )

    # Kameravisning: rå kamerabilde (kun ved show_raw_camera:=true)
    raw_image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='raw_image_view',
        remappings=[('image', '/camera/color/image_raw')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_raw_camera')),
    )

    return LaunchDescription([
        config_file_arg,
        ur_type_arg,
        robot_ip_arg,
        video_device_arg,
        show_raw_camera_arg,
        LogInfo(msg='===== Starter UR-prosjekt ====='),
        camera_node,
        cube_vision_node,
        coordinate_transformer_node,
        robot_control_node,
        coordinator_node,
        debug_image_view_node,
        raw_image_view_node,
    ])
