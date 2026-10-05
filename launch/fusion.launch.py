from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/camera/depth/image_rect_raw')
    camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/camera/depth/camera_info')
    output_topic = DeclareLaunchArgument(
        'output_topic', default_value='/filtered_points')
    odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/r1a006/wheel_odom')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    organized_output = DeclareLaunchArgument(
        'organized_output', default_value='true')

    fusion_node = Node(
        package='depth_costmap_fusion',
        executable='depth_fusion_node',     # for launching in py it is fusion_node
        name='depth_fusion_node',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'enable_confidence_filter': False,
            'min_range_m': 0.2,
            'max_range_m': 6.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'organized_output': LaunchConfiguration('organized_output'),
        }],
    )

    odom_tf_broadcaster_node = Node(
        package='depth_costmap_fusion',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        depth_topic, camera_info_topic, output_topic, odom_topic, use_sim_time, organized_output,
        fusion_node,
        odom_tf_broadcaster_node,
    ])
