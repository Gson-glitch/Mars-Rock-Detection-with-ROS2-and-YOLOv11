from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value='0',  
        description='Video source (0 for webcam, or path to video file)'
    )

    return LaunchDescription([
        video_source_arg,
        Node(
            package='rock_detection',
            executable='video_publisher',
            name='video_publisher',
            parameters=[{
                'video_source': LaunchConfiguration('video_source')
            }]
        ),
        Node(
            package='rock_detection',
            executable='rock_detector',
            name='rock_detector',
            parameters=[{
                'min_distance_to_wall': 1.5,
                'turtle_linear_speed': 0.3,
                'turtle_angular_scale': 0.8,
                'confidence_threshold': 0.5
            }]
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    ])