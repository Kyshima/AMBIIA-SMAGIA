import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('plugins')
    world_file = os.path.join(package_dir, 'worlds', 'teste.world')

    # Launch Gazebo with the specified world file
    gazebo_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    model_path = os.path.join(package_dir, 'models')

    # Spawn the Prius Hybrid model in Gazebo
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', os.path.join(package_dir, 'models', 'robot', 'robot.urdf')],
        output='screen'
    )

    # Example node to publish commands to the car
    cmd_publisher_node = Node(
        package='plugins',
        executable='control_node',  # Seu nó que publica comandos
        output='screen'
    )

    return LaunchDescription([
        gazebo_node,
        spawn_node,
        cmd_publisher_node
    ])