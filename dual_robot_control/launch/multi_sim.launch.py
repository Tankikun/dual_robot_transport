import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Start World
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 1. Get the URDF (For Robot State Publisher - The Visuals/TF)
    urdf_file = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')
    
    # 2. Get the SDF (For Gazebo Spawn - The Physics/Motors)
    # This file contains the <plugin> tags that make the robot move!
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf')

    def spawn_turtlebot(ns, x, y):
        return [
            # State Publisher uses URDF
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=ns,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_file).read()}],
            ),
            # Gazebo Spawn uses SDF
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', ns,
                    '-file', sdf_file,   # <--- CHANGED TO SDF
                    '-x', x,
                    '-y', y,
                    '-z', '0.01',
                    '-robot_namespace', ns
                ],
                output='screen',
            ),
        ]

    ld = LaunchDescription()
    ld.add_action(world)

    # Spawn Robot 0 (Left)
    for node in spawn_turtlebot('tb3_0', '0.0', '0.25'):
        ld.add_action(node)

    # Spawn Robot 1 (Right)
    for node in spawn_turtlebot('tb3_1', '0.0', '-0.25'):
        ld.add_action(node)

    return ld
