from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    ROS 2 Jazzy replacement for oakd_pcloud (ROS 1) using depthai_ros_driver.

    It exposes oakd_pcloud-compatible topic names via remaps so the rest of
    your stack keeps working:
      - /stereo_rgb_node/stereo/points (sensor_msgs/PointCloud2)
      - /stereo_rgb_node/stereo/image (sensor_msgs/Image, depth rect or metric)
      - /stereo_rgb_node/stereo/camera_info (sensor_msgs/CameraInfo)
      - /stereo_rgb_node/rectified_left/image, .../rectified_right/image

    Requires depthai_ros_driver installed on the robot.
    """

    # Which launch file inside depthai_ros_driver to use
    depthai_launch = LaunchConfiguration('depthai_launch')
    camera_name = LaunchConfiguration('camera_name')

    return LaunchDescription([
        DeclareLaunchArgument('depthai_launch', default_value='pointcloud.launch.py',
                              description='Launch file from depthai_ros_driver to start OAK-D with point cloud output'),
        DeclareLaunchArgument('camera_name', default_value='oak', description='Camera base name for frame ids'),

        # Include DepthAI ROS 2 pipeline (stereo + pointcloud)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('depthai_ros_driver'), '/launch/', depthai_launch
            ]),
            # Remap to oakd_pcloud-compatible topics
            launch_arguments={
                # If the upstream launch exposes arguments for topics, you can pass them here.
                # Otherwise, we add explicit remap nodes below.
            }.items(),
        ),

        # Remap depthai_ros_driver topics to oakd_pcloud-compatible names via relay nodes.
        # Adjust the source topics below to match your depthai_ros_driver setup if different.
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_points',
            arguments=['/stereo/points', '/stereo_rgb_node/stereo/points'],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_depth_image',
            arguments=['/stereo/image_rect', '/stereo_rgb_node/stereo/image'],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_stereo_info',
            arguments=['/stereo/camera_info', '/stereo_rgb_node/stereo/camera_info'],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_left_rect',
            arguments=['/stereo/left/image_rect', '/stereo_rgb_node/rectified_left/image'],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_right_rect',
            arguments=['/stereo/right/image_rect', '/stereo_rgb_node/rectified_right/image'],
            output='screen',
        ),
    ])

