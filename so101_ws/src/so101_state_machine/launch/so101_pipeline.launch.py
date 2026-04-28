from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '1.125', '0.097', '0.410',
                '0.447', '-0.472', '-0.539', '0.536',
                'base_link', 'camera_link'
            ],
            output='screen'
        ),

        Node(
            package='so101_state_machine',
            executable='isaac_state_relay',
            name='isaac_state_relay',
            output='screen'
        ),

        Node(
            package='so101_state_machine',
            executable='trajectory_to_isaac',
            name='trajectory_to_isaac',
            output='screen'
        ),

        Node(
            package='so101_state_machine',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'rgb_topic': '/rgb',
                'depth_topic': '/depth',
                'camera_info_topic': '/camera_info',
                'base_frame': 'base_link',
                'camera_frame': 'camera_link',
            }]
        ),

        Node(
            package='so101_state_machine',
            executable='movegroup_executor',
            name='movegroup_executor',
            output='screen'
        ),

        # Uncomment this if you want BT to start automatically
        Node(
            package='so101_state_machine',
            executable='bt_node',
            name='bt_node',
            output='screen'
        ),
    ])
