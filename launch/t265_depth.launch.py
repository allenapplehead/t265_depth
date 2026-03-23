from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('t265_depth')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic_left',
            default_value='/camera/fisheye1/image_raw'),
        DeclareLaunchArgument(
            'input_topic_right',
            default_value='/camera/fisheye2/image_raw'),
        DeclareLaunchArgument(
            'output_frame_id',
            default_value='t265_depth'),
        DeclareLaunchArgument(
            'process_every_nth_frame',
            default_value='1'),
        DeclareLaunchArgument(
            'param_file_path',
            default_value=PathJoinSubstitution([pkg_share, 'cfg', 'full_res.yaml'])),
        DeclareLaunchArgument(
            'scale',
            default_value='1.0',
            description='Image scale factor: 1, 0.5, or 0.33'),
        DeclareLaunchArgument(
            'input_transport',
            default_value='raw',
            description='image_transport type: raw or compressed'),

        Node(
            package='t265_depth',
            executable='t265_depth',
            name='t265_depth',
            output='screen',
            parameters=[{
                'input_topic_left':        LaunchConfiguration('input_topic_left'),
                'input_topic_right':       LaunchConfiguration('input_topic_right'),
                'output_frame_id':         LaunchConfiguration('output_frame_id'),
                'param_file_path':         LaunchConfiguration('param_file_path'),
                'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                'scale':                   LaunchConfiguration('scale'),
                'input_transport':         LaunchConfiguration('input_transport'),

                # BM pre-filter parameters
                'sad_window_size':   3,
                'pre_filter_type':   'normalized_response',  # xsobel or normalized_response
                'pre_filter_cap':    1,
                'pre_filter_size':   5,

                # SGBM parameters
                'use_sgbm':          True,
                'sgbm_mode':         2,     # 0=SGBM, 1=HH, 2=3WAY
                'p1':                240,
                'p2':                960,
                'disp_12_max_diff':  -1,

                # Disparity search parameters
                'min_disparity':     0,
                'num_disparities':   64,

                # Post-filter parameters
                'uniqueness_ratio':   10,
                'texture_threshold':  10,
                'speckle_range':      32,
                'speckle_window_size': 1,
                'do_median_blur':     True,
            }],
        ),
    ])
