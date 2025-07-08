# your_nav2_bringup/launch/navigation_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 启动参数
    map_yaml_file   = LaunchConfiguration('map')
    params_file     = LaunchConfiguration('params_file')
    bt_xml_file     = LaunchConfiguration('bt_xml')
    use_sim_time    = LaunchConfiguration('use_sim_time')

    # 允许在参数文件中使用 launch substitution
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': bt_xml_file,
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml_file}],
        ),
        # AMCL 定位
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen',
            parameters=[configured_params],
        ),
        # 恢复行为
        Node(
            package='nav2_recoveries', executable='recoveries_server', name='recoveries_server',
            output='screen',
            parameters=[configured_params],
        ),
        # 控制器
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=[configured_params],
        ),
        # 规划器
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        # 行为树导航器
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        # Lifecycle 管理
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
            output='screen',
            parameters=[configured_params,
                        {'autostart': True},
                        {'node_names': [
                            'map_server',
                            'amcl',
                            'recoveries_server',
                            'controller_server',
                            'planner_server',
                            'bt_navigator']}
            ]
        ),
    ])
