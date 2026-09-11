from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'omron_cobra_s600_description'
    package_share = FindPackageShare(package_name)
    xacro_file = PathJoinSubstitution([package_share, 'urdf', 'omron_cobra_s600.urdf.xacro'])

    use_meshes = LaunchConfiguration('use_meshes')
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file, ' use_meshes:=', use_meshes
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'omron_cobra_s600'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    cobra_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cobra_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )
    load_cobra_controller = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[cobra_controller])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_meshes',
            default_value='true',
            description='Use the committed STL files for visual geometry.',
        ),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        load_joint_state_broadcaster,
        load_cobra_controller,
    ])
