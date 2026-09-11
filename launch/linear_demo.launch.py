from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_meshes = LaunchConfiguration('use_meshes')
    move_time = LaunchConfiguration('move_time')
    hold_time = LaunchConfiguration('hold_time')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('omron_cobra_s600_description'),
                'launch',
                'gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'use_meshes': use_meshes,
            'controller_name': 'effort_controller',
        }.items(),
    )

    linear_controller = Node(
        package='omron_cobra_s600_description',
        executable='control_demo.py',
        name='linear_control_demo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': 'linear',
            'move_time': move_time,
            'hold_time': hold_time,
            'target': [0.6, -0.8, 0.08, 0.6],
            'kp': [40.0, 40.0, 220.0, 2.0],
            'kd': [12.0, 12.0, 35.0, 0.5],
        }],
    )

    delayed_controller = TimerAction(period=6.0, actions=[linear_controller])

    return LaunchDescription([
        DeclareLaunchArgument('use_meshes', default_value='true'),
        DeclareLaunchArgument('move_time', default_value='5.0'),
        DeclareLaunchArgument('hold_time', default_value='1.0'),
        gazebo_launch,
        delayed_controller,
    ])
