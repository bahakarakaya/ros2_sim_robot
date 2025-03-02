import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    my_robot_description_pkg = get_package_share_directory('my_robot_description')
    my_robot_bringup_pkg = get_package_share_directory('my_robot_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # Paths
    urdf_path = os.path.join(my_robot_description_pkg, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(my_robot_bringup_pkg, 'rviz', 'urdf_config.rviz')
    world_path = os.path.join(my_robot_bringup_pkg, 'worlds', 'test_world.world')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', urdf_path]
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )
    
    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    # Launch Description
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        rviz_node
    ])