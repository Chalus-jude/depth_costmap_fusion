 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_r1nav = get_package_share_directory('r1_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_params_file_path = os.path.join(pkg_r1nav, 'config', 'nav2_params_mapless.yaml')
    map_path = os.path.join(pkg_r1nav, 'maps', 'amd_to_amphi.yaml')
    rviz_config = os.path.join(pkg_r1nav, 'config', 'nav2_default.rviz')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_path,
            'params_file': nav2_params_file_path,
            'package_path': pkg_r1nav,
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + rviz_config
        ]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(static_transform_publisher_node)

    return ld
