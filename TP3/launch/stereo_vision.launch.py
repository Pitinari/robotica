from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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

    enable_triangulator_arg = DeclareLaunchArgument(
        'enable_triangulator',
        default_value='false',
        description='Enable triangulator_node'
    )

    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='false',
        description='Enable mapping_node'
    )

    enable_disparity_arg = DeclareLaunchArgument(
        'enable_disparity',
        default_value='true',
        description='Enable disparity_node'
    )

    enable_dense_reconstruction_arg = DeclareLaunchArgument(
        'enable_dense_reconstruction',
        default_value='true',
        description='Enable dense_reconstruction_node'
    )

    enable_dense_mapping_arg = DeclareLaunchArgument(
        'enable_dense_mapping',
        default_value='false',
        description='Enable dense_mapper_node'
    )

    enable_pose_estimation_arg = DeclareLaunchArgument(
        'enable_pose_estimation',
        default_value='false',
        description='Enable pose_estimation_node'
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
        ],
        condition=IfCondition(LaunchConfiguration('enable_triangulator'))
    )

    # Mapping node with ground-truth localization
    mapping_node = Node(
        package='stereo_vision',
        executable='mapping_node',
        name='mapping',
        output='screen',
        parameters=[
            {
                'left_calib_file': 'config/camera_info_left.yaml',
                'right_calib_file': 'config/camera_info_right.yaml',
                'extrinsics_file': 'config/extrinsics.yaml',
                'distance_threshold': 50.0,
                'max_features': 1000,
                'threshold': 10.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_mapping'))
    )

    # Disparity node
    disparity_node = Node(
        package='stereo_vision',
        executable='disparity_node',
        name='disparity',
        output='screen',
        parameters=[
            {
                'method': 'SGBM',
                'min_disparity': 0,
                'num_disparities': 64,
                'block_size': 15,
                'speckle_window_size': 100,
                'speckle_range': 32,
                'uniqueness_ratio': 10,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_disparity'))
    )

    # Dense reconstruction node
    dense_reconstruction_node = Node(
        package='stereo_vision',
        executable='dense_reconstruction_node',
        name='dense_reconstruction',
        output='screen',
        parameters=[
            {
                'left_calib_file': 'config/camera_info_left.yaml',
                'right_calib_file': 'config/camera_info_right.yaml',
                'extrinsics_file': 'config/extrinsics.yaml',
                'min_depth': 0.1,
                'max_depth': 50.0,
                'filter_invalid': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_dense_reconstruction'))
    )

    # Pose estimation node (monocular visual odometry)
    pose_estimation_node = Node(
        package='stereo_vision',
        executable='pose_estimation_node',
        name='pose_estimation',
        output='screen',
        parameters=[
            {
                'left_calib_file': 'config/camera_info_left.yaml',
                'right_calib_file': 'config/camera_info_right.yaml',
                'extrinsics_file': 'config/extrinsics.yaml',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_pose_estimation'))
    )

    # Dense mapping node with ground-truth localization
    dense_mapper_node = Node(
        package='stereo_vision',
        executable='dense_mapper_node',
        name='dense_mapper',
        output='screen',
        parameters=[
            {
                'left_calib_file': 'config/camera_info_left.yaml',
                'right_calib_file': 'config/camera_info_right.yaml',
                'extrinsics_file': 'config/extrinsics.yaml',
                'min_depth': 0.1,
                'max_depth': 20.0,  # Reduced from 50.0 to filter distant points
                'filter_invalid': True,
                'point_subsample_factor': 16,  # Keep only every 16th point (reduces to 6.25% of points)
                'max_distance': 20.0,  # Filter points beyond 20 meters in world frame
                'min_point_distance': 0.05,  # Minimum distance between points (5cm) to avoid duplicates
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_dense_mapping'))
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
        enable_triangulator_arg,
        enable_mapping_arg,
        enable_disparity_arg,
        enable_dense_reconstruction_arg,
        enable_dense_mapping_arg,
        enable_pose_estimation_arg,
        stereo_rectifier_node,
        feature_extractor_node,
        feature_matcher_node,
        triangulator_node,
        mapping_node,
        disparity_node,
        dense_reconstruction_node,
        dense_mapper_node,
        pose_estimation_node,
        static_transform,
        rviz_node,
        bag_player,
    ])

