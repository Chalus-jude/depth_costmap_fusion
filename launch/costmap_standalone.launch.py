import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('depth_costmap_fusion'),
        'config', 'stvl_costmap.yaml')

    params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        params_file, use_sim_time,
        costmap_node,
    ])
