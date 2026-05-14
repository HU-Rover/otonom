import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'odometry'  # Your package name

    # Paths for RViz configuration
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'rviz_config.rviz'
    )

    return launch.LaunchDescription([
        # Odometry Publisher Node
        launch_ros.actions.Node(
            package=package_name,
            executable='odom',
            name='odometry_publisher',
            output='screen'
        ),

        # TF2 Static Transform Broadcaster (odom -> base_link)
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # Launch RViz2 with custom config
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])

