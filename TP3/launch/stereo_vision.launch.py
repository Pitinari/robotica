from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to ROS2 bag file to play'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag file'
    )
    
    # Stereo rectifier node
    stereo_rectifier_node = Node(
        package='stereo_vision',
        executable='stereo_rectifier_node',
        name='stereo_rectifier',
        output='screen',
        parameters=[
            {
                'left_calib_file': 'config/camera_info_left.yaml',
                'right_calib_file': 'config/camera_info_right.yaml',
                'extrinsics_file': 'config/extrinsics.yaml',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/cam0/image_raw', '/cam0/image_raw'),
            ('/cam1/image_raw', '/cam1/image_raw'),
        ]
    )
    
    # Feature extractor node
    feature_extractor_node = Node(
        package='stereo_vision',
        executable='feature_extractor_node',
        name='feature_extractor',
        output='screen',
        parameters=[
            {
                'detector_type': 'ORB',
                'descriptor_type': 'ORB',
                'max_features': 1000,
                'threshold': 10.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/stereo/left/rect', '/stereo/left/rect'),
        ]
    )
    
    # Feature matcher node
    feature_matcher_node = Node(
        package='stereo_vision',
        executable='feature_matcher_node',
        name='feature_matcher',
        output='screen',
        parameters=[
            {
                'distance_threshold': 30.0,
                'detector_type': 'ORB',
                'descriptor_type': 'ORB',
                'max_features': 1000,
                'threshold': 10.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )
    
    # Triangulator node
    triangulator_node = Node(
        package='stereo_vision',
        executable='triangulator_node',
        name='triangulator',
        output='screen',
        parameters=[
            {
                'left_calib_file': 'config/camera_info_left.yaml',
                'right_calib_file': 'config/camera_info_right.yaml',
                'extrinsics_file': 'config/extrinsics.yaml',
                'distance_threshold': 30.0,
                'max_features': 1000,
                'threshold': 10.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )
    
    # Static transform from map to cam0
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_cam0_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'cam0']
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'config/minimal_stereo.rviz'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Bag player (optional, only if bag_file is provided)
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file'),
             '--clock', '--rate', '0.5'],
        output='screen'
    )
    
    return LaunchDescription([
        bag_file_arg,
        use_sim_time_arg,
        stereo_rectifier_node,
        feature_extractor_node,
        feature_matcher_node,
        triangulator_node,
        static_transform,
        rviz_node,
        bag_player,
    ])

