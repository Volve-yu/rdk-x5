import os
from launch import LaunchDescription, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
import launch_ros

def generate_launch_description():
    pkg_share = LaunchConfiguration('pkg_share')
    default_model_path = os.path.join(pkg_share.perform({}), 'src/description/sam_bot_description.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])

