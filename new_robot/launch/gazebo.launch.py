import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('new_robot')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'new_robot.urdf'])
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}

    new_robot_share = get_package_share_directory('new_robot')
    resource_path = os.path.dirname(new_robot_share)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_resource_path:
        resource_path = existing_resource_path + os.pathsep + resource_path

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'new_robot',
            '-x', '0', '-y', '0', '-z', '0.5',
            '-R', '-1.57079632679',
            '-P', '1.57079632679',
            '-Y', '0',
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/new_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen',
    )

    gimbal_roll_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/gimbal_roll_cmd@std_msgs/msg/Float64]gz.msgs.Double'],
        output='screen',
    )   
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
    )   
    gimbal_pitch_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/gimbal_pitch_cmd@std_msgs/msg/Float64]gz.msgs.Double'],
        output='screen',
    )
    imu_tray_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu_tray@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        clock_bridge,
        cmd_vel_bridge,
        odom_bridge,
        gimbal_roll_bridge,
        gimbal_pitch_bridge,
        imu_bridge,
        imu_tray_bridge,

    ])