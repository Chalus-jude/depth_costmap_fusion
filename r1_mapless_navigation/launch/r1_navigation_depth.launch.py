import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('r1_mapless_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_mapless.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg, 'config', 'r1_mapless.rviz')),
        # Fusion node inputs (defaults = RealSense on robot 1)
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/camera/depth/camera_info'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
    ]

    fusion = Node(
        package='r1_mapless_navigation',
        executable='fusion_node',
        name='depth_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
        }],
    )

    # Mapless Nav2: navigation stack only (no map_server / amcl)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [fusion, nav2, rviz])
