import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('assembly_2')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'assembly_2.urdf'])
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}

    # assembly_2 패키지 share 디렉토리의 "부모"를 등록해야
    # gz-sim이 model://assembly_2/meshes/... 를 찾을 수 있음
    assembly_2_share = get_package_share_directory('assembly_2')
    resource_path = os.path.dirname(assembly_2_share)
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
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
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
            '-name', 'assembly_2',
            '-x', '0', '-y', '0', '-z', '0.5',
            '-R', '1.57079632679',   # ← GUI에서 찾은 roll 값(라디안)으로 교체
            '-P', '0',   # ← pitch 값으로 교체
            '-Y', '0',   # ← yaw 값으로 교체
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

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        clock_bridge,
        cmd_vel_bridge,
    ])