import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    my_robot_bringup_dir = get_package_share_directory('my_robot_bringup')
    
    # Include the gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(my_robot_bringup_dir, 'launch', 'my_robot_gazebo.launch.py')
        ])
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )
    
    # Teleop twist joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        parameters=[{
            'axis_linear.x': 1,
            'scale_linear.x': 0.5,
            'scale_linear_turbo.x': 1.0,
            'axis_angular.yaw': 0,
            'scale_angular.yaw': 0.5,
            'scale_angular_turbo.yaw': 1.0,
            'enable_button': 5,  # Hold 'RB' while driving
            'enable_turbo_button': 2,  # Hold 'X' while driving in turbo mode
            'require_enable_button': False
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_raw')
        ]
    )

    # Control switch node
    control_switch_node = Node(
        package='my_robot_control',
        executable='control_switch_node',
        output='screen'
    )
    
    # Launch Description
    return LaunchDescription([
        gazebo_launch,
        joy_node,
        teleop_node,
        control_switch_node
    ])