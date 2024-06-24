import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('plugins')
    map_dir = os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml')  # Use your map or an empty map
    
    param_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch',
            name='nav2_bringup',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_dir},
                param_file
            ],
            arguments=['--params-file', param_file]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
