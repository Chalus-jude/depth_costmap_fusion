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

    params_file = os.path.join(pkg_r1nav, 'config', 'nav2_params_mapless.yaml')
    rviz_config = os.path.join(pkg_r1nav, 'config', 'nav2_default.rviz')

    # navigation_launch.py = planner/controller/behaviors/bt_navigator/smoother only.
    # No map_server, no amcl.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([navigation, rviz, map_to_odom])
    
    
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
        # Fusion node inputs (defaults = RealSense on robot 1)
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/camera/depth/camera_info'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        # Bag-replay helpers (leave false on the real robot / sim)
        DeclareLaunchArgument('odom_bridge', default_value='false'),
        DeclareLaunchArgument('static_camera_tf', default_value='false'),
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

    # Only for bag replay: wheel_odom -> TF (odom -> base_link)
    odom_bridge = Node(
        package='r1_mapless_navigation',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('odom_bridge')),
    )

    # Only for bag replay: base_link -> camera_link from the URDF
    static_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        arguments=['--x', '0.15065', '--y', '0.0', '--z', '0.08505',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('static_camera_tf')),
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

    return LaunchDescription(args + [fusion, odom_bridge, static_cam_tf, nav2])

