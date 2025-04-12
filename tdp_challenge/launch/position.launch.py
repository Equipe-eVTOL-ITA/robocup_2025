from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Declare the nodes

    vertical_classifier_node = Node(
        package='robocup_cv_utils',
        executable='bucket_detector',
        name='vertical_bucket_detector',
        parameters=[{'image_topic': '/vertical_camera/compressed', 'model': 'buckets_v0'}]
    )

    angled_classifier_node = Node(
        package='robocup_cv_utils',
        executable='bucket_detector',
        name='angled_bucket_detector',
        parameters=[{'image_topic': '/angled_camera/compressed', 'model': 'buckets_v0'}]
    )

    position_node = Node(
        package='tdp_challenge',
        executable='position',
        name='position_fsm'
    )

    # TimerAction to delay the launch of frtl_2024_fase4 by 15 seconds
    delayed_position_node = TimerAction(
        period=10.0,  # 15 seconds delay
        actions=[position_node]
    )

    # Create a LaunchDescription and add the nodes to it
    return LaunchDescription([
        vertical_classifier_node,
        angled_classifier_node,
        delayed_position_node
    ])
