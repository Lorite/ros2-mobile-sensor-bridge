import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    tf_parent_frame = LaunchConfiguration('tf_parent_frame', default='map')
    tf_child_frame = LaunchConfiguration('tf_child_frame', default='mobile_sensor')
    pose_map_position = LaunchConfiguration('pose_map_position', default='z,x,y')
    # Static camera transform (relative to tf_child_frame -> camera_frame)
    camera_static_tx = LaunchConfiguration('camera_static_tx', default='0.02')
    camera_static_ty = LaunchConfiguration('camera_static_ty', default='0.035')
    camera_static_tz = LaunchConfiguration('camera_static_tz', default='0.0')
    camera_static_roll = LaunchConfiguration('camera_static_roll', default='0.0')
    camera_static_pitch = LaunchConfiguration('camera_static_pitch', default='0.0')
    camera_static_yaw = LaunchConfiguration('camera_static_yaw', default='0.0')
    share_directory = get_package_share_directory('mobile_sensor')

    # Path to your main JS file (now in server folder)
    start_js_file = os.path.join(
        share_directory,
        'dist',
        'server',
        'index.js')
    
    cert_directory = os.path.join(share_directory, 'dist')

    # RViz2 configuration file path (prefer installed share, fallback to source tree)
    rviz_config = os.path.join(share_directory, 'rviz', 'mobile_sensor.rviz')
    if not os.path.exists(rviz_config):
        # Fallback to source-relative path
        src_rviz = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rviz', 'mobile_sensor.rviz'))
        if os.path.exists(src_rviz):
            rviz_config = src_rviz

    sensor_node = Node(
        name='mobile_sensor_node',
        executable='node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tf_parent_frame': tf_parent_frame,
            'tf_child_frame': tf_child_frame,
            'pose_map_position': pose_map_position,
        }],
        arguments=[
            start_js_file
        ],
        cwd=cert_directory)

    # Publish static transform using tf2_ros static_transform_publisher
    # Note: static_transform_publisher expects: x y z yaw pitch roll frame_id child_frame_id
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf',
        output='screen',
        arguments=[
            camera_static_tx,
            camera_static_ty,
            camera_static_tz,
            camera_static_yaw,
            camera_static_pitch,
            camera_static_roll,
            tf_child_frame,
            'camera_frame'
        ]
    )

    # RViz2 node to visualize TF and frames
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(sensor_node)
    ld.add_action(static_tf_node)
    ld.add_action(rviz_node)

    # Declare launch arguments to make static TF configurable at runtime
    ld.add_action(DeclareLaunchArgument('camera_static_tx', default_value='0.02'))
    ld.add_action(DeclareLaunchArgument('camera_static_ty', default_value='0.035'))
    ld.add_action(DeclareLaunchArgument('camera_static_tz', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('camera_static_roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('camera_static_pitch', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('camera_static_yaw', default_value='0.0'))

    return ld
