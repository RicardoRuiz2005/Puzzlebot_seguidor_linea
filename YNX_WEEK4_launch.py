from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    line_follower = Node(
        name='line_follower',
        package='YNX_week4',
        executable='line_follower',
        output='screen',
    )

    color_detector = Node(
        name='color_detector',
        package='YNX_week4',
        executable='color_detector',
        output='screen',
    )

    controller_od = Node(
        name='controller_od',
        package='YNX_week4',
        executable='controller_od',
        output='screen',
    )

    image_viewer = Node(
        name='image_viewer',
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/processed_img'],
    )

    return LaunchDescription([
        line_follower,
        color_detector,
        controller_od,
        image_viewer,
    ])
