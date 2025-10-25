from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_freenove_4wd')
    default_rviz = os.path.join(pkg_share, 'rviz', 'laptop_viz.rviz')

    rvizconfig = LaunchConfiguration('rvizconfig')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz,
            description='Absolute path to RViz config file'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizconfig],
        ),
    ])

