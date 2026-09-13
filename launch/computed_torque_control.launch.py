from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_meshes = LaunchConfiguration('use_meshes')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('omron_cobra_s600_description'),
                'launch', 'gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'use_meshes': use_meshes,
            'controller_name': 'effort_controller',
        }.items(),
    )

    controller = Node(
        package='omron_cobra_s600_description',
        executable='control_demo.py',
        name='computed_torque_control_demo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': 'computed_torque',
            'swing_amplitude': 0.8,
            'swing_period': 6.0,
            'kp': [18.0, 18.0, 45.0, 25.0],
            'kd': [8.0, 8.0, 14.0, 8.0],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_meshes', default_value='true'),
        gazebo_launch,
        TimerAction(period=6.0, actions=[controller]),
    ])
