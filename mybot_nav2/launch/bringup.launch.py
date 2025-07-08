# your_nav2_bringup/launch/bringup_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 参数声明
    pkg_share = os.path.join(
        os.getenv('ROS_PACKAGE_PATH').split(':')[0],
        'share', 'your_nav2_bringup'
    )
    default_map       = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_params    = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    default_bt_xml    = os.path.join(pkg_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    declare_map       = DeclareLaunchArgument('map',       default_value=default_map,    description='Full path to map file')
    declare_params    = DeclareLaunchArgument('params_file',default_value=default_params, description='Full path to ROS2 parameters file')
    declare_bt        = DeclareLaunchArgument('bt_xml',     default_value=default_bt_xml,  description='Behavior Tree XML file')
    declare_use_sim   = DeclareLaunchArgument('use_sim_time', default_value='false',      description='Use simulation (Gazebo) clock')

    # 包含 Nav2 核心 launch
    navigation_launch = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'navigation_launch.py')
    )
    include_nav2 = Node(
        package='launch', executable='launch', name='include_navigation_launch',
        arguments=[navigation_launch,
                   '--map',        LaunchConfiguration('map'),
                   '--params-file',LaunchConfiguration('params_file'),
                   '--bt-xml',     LaunchConfiguration('bt_xml'),
                   '--use-sim-time', LaunchConfiguration('use_sim_time')],
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_bt,
        declare_use_sim,
        include_nav2,
    ])
