import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 기본값 '' -> 기존 동작과 동일. 가제보 상태 로그 녹화가 필요할 때만
    # 예: gz_extra_args:="--record-path /tmp/gz_record_run" 처럼 넘겨서
    # 나중에 `gz sim -r --playback <path>`로 그대로 재생할 수 있게 한다.
    gz_extra_args_arg = DeclareLaunchArgument('gz_extra_args', default_value='')
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
        launch_arguments={
            'gz_args': ['empty.sdf ', LaunchConfiguration('gz_extra_args')],
        }.items(),
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
            '-x', '0', '-y', '0', '-z', '0.15',
            '-R', '-1.57079632679',
            '-P', '1.57079632679',
            '-Y', '0',
        ],
        output='screen',
    )

    # 언덕(hill.sdf)은 반지름 2.5m 구를 z=-2.2에 파묻은 형태라, 지면과 만나는
    # 실제 돔 발자국 반지름은 sqrt(2.5^2 - 2.2^2)다. 5개를 로봇 진행방향(world +Y)을
    # 따라 순서대로 나열한다 — 로봇이 직진하면 차례로 넘게 된다.
    # 중심간 거리 = 지름의 2배 (= 발자국 사이에 지름 1개만큼 평지 간격).
    HILL_SPHERE_RADIUS = 2.5
    HILL_BURY_DEPTH = 2.2
    hill_footprint_radius = math.sqrt(HILL_SPHERE_RADIUS ** 2 - HILL_BURY_DEPTH ** 2)
    hill_diameter = 2 * hill_footprint_radius
    hill_spacing = 2 * hill_diameter
    hill_first_y = 2.0
    n_hills = 5

    spawn_hills = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', PathJoinSubstitution([pkg_share, 'worlds', 'hill.sdf']),
                '-name', f'hill_{i}',
                '-x', '0',
                '-y', str(hill_first_y + i * hill_spacing),
                '-z', str(-HILL_BURY_DEPTH),
            ],
            output='screen',
        )
        for i in range(n_hills)
    ]

    # 5번째(마지막) 언덕 다음에 삼각기둥형 다리(ramp_bridge.sdf)를 배치한다.
    # 밑변이 옆면이 되도록 눕힌 삼각기둥이라, 로봇이 그 위를 지나가면
    # 오르막(피치업) -> 정점 -> 내리막(피치다운)을 깨끗하게 겪는다.
    # ramp_bridge.sdf 자체 치수(2026-08-31 갱신 — 기존 대비 5배 크기,
    # 경사각 30도로 재설계): 슬로프 절반 길이 6.0m, 높이 6.0*tan(30도)
    # =3.4641m, 폭(X) 12.5m.
    RAMP_HALF_RUN = 6.0
    RAMP_WIDTH = 12.5
    RAMP_GAP_AFTER_LAST_HILL = 1.5  # 마지막 언덕 발자국 끝~다리 시작 사이 평지 간격
    last_hill_far_edge_y = hill_first_y + (n_hills - 1) * hill_spacing + hill_footprint_radius
    ramp_center_y = last_hill_far_edge_y + RAMP_GAP_AFTER_LAST_HILL + RAMP_HALF_RUN

    spawn_ramp_bridge = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([pkg_share, 'worlds', 'ramp_bridge.sdf']),
            '-name', 'ramp_bridge',
            '-x', str(-RAMP_WIDTH / 2.0),
            '-y', str(ramp_center_y),
            '-z', '0',
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
    tray_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/new_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen',
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/model/new_robot/joint_state'
                   '@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[('/world/empty/model/new_robot/joint_state', '/joint_states')],
        output='screen',
    )

    return LaunchDescription([
        gz_extra_args_arg,
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        *spawn_hills,
        spawn_ramp_bridge,
        clock_bridge,
        cmd_vel_bridge,
        odom_bridge,
        gimbal_roll_bridge,
        gimbal_pitch_bridge,
        imu_bridge,
        imu_tray_bridge,
        tray_pose_bridge,
        joint_state_bridge,

    ])