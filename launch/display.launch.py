from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'omron_cobra_s600_description'
    use_meshes = LaunchConfiguration('use_meshes')
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'urdf', 'omron_cobra_s600.urdf.xacro'
    ])
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file, ' use_meshes:=', use_meshes
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_meshes',
            default_value='false',
            description='Use the four STL visuals. Leave false until the STL files are in meshes/.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
