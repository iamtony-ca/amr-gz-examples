import os
import yaml
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction,
                            AppendEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 기본 경로 및 설정 ---
    bringup_dir = get_package_share_directory('nav2_bringup')
    tb4_sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    tb4_desc_dir = get_package_share_directory('nav2_minimal_tb4_description')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # --- 2. 런치 파라미터 선언 ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='True') # 'true' -> 'True'
    map_arg = DeclareLaunchArgument('map', 
        default_value=os.path.join(bringup_dir, 'maps', 'depot.yaml'))
    
    robot_config_path = os.path.join(bringup_dir, 'config', 'tb4_robots.yaml')
    with open(robot_config_path, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    declare_params_cmds = []
    for robot in robots:
        declare_params_cmds.append(DeclareLaunchArgument(
            f"{robot['name']}_params_file",
            # 이 부분이 올바르게 각 파라미터 파일을 가리키고 있는지 확인
            default_value=os.path.join(bringup_dir, 'params', f"nav2_multirobot_params_{robot['name'][-1]}.yaml"),
            description=f"Full path to the ROS2 parameters file for {robot['name']}"))

    # --- 3. Gazebo 실행 ---
    world = LaunchConfiguration('world', default=os.path.join(tb4_sim_dir, 'worlds', 'depot.sdf'))
    headless = LaunchConfiguration('headless', default='False')

    gz_resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
        os.path.join(tb4_desc_dir, 'meshes'))

    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    gazebo_server = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_sdf], output='screen')
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression([use_sim_time, ' and not ', headless])),
        launch_arguments={'gz_args': '-g -v4'}.items(),
    )
    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]))

    # --- 4. 각 로봇에 대한 노드 그룹 생성 (루프) ---
    robots_actions = []
    for robot in robots:
        robot_name = robot['name']
        namespace = robot_name
        params_file = LaunchConfiguration(f'{robot_name}_params_file')

        robot_group = GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': Command(['xacro', ' ', os.path.join(tb4_desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'), ' namespace:=', namespace])
                    # 'frame_prefix': f'{namespace}/'
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                namespace=namespace,
                arguments=[
                    '-name', namespace, '-topic', 'robot_description',
                    '-x', str(robot['x_pose']), '-y', str(robot['y_pose']),
                    '-z', '0.1', '-Y', str(robot['yaw']),
                ],
                output='screen',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_ros_gz',
                namespace=namespace,
                output='screen',
                arguments=[
                    # Odometry (Gz -> ROS)
                    f'/model/{namespace}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # Scan (Gz -> ROS)
                    f'/world/depot/model/{namespace}/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    # TF (Gz -> ROS)
                    f'/model/{namespace}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    # IMU (Gz -> ROS)
                    f'/world/depot/model/{namespace}/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    # Joint States (Gz -> ROS)
                    f'/world/depot/model/{namespace}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                    # Command Velocity (ROS -> Gz)
                    f'cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                ],
                remappings=[
                    # 브릿지의 ROS 토픽을 Nav2가 사용하는 표준 토픽 이름으로 리매핑합니다.
                    (f'/model/{namespace}/odometry', 'odom'),
                    (f'/world/depot/model/{namespace}/link/rplidar_link/sensor/rplidar/scan', 'scan'),
                    (f'/model/{namespace}/tf', 'tf'),
                    (f'/world/depot/model/{namespace}/link/imu_link/sensor/imu_sensor/imu', 'imu'),
                    (f'/world/depot/model/{namespace}/joint_state', 'joint_states'),
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
                launch_arguments={
                    'namespace': namespace, 'use_namespace': 'True', 'map': LaunchConfiguration('map'),
                    'use_sim_time': use_sim_time, 'params_file': params_file,
                    'autostart': 'True',  # <-- 'true'가 아닌 'True'로 수정된 부분입니다.
                    'use_simulator': 'False', 'use_rviz': 'False', 'use_robot_state_pub': 'False'
                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
                launch_arguments={
                    'namespace': namespace, 'use_namespace': 'True', 'use_sim_time': use_sim_time
                }.items())
        ])
        robots_actions.append(robot_group)

    # --- 5. 최종 LaunchDescription 조립 ---
    ld = LaunchDescription()
    
    ld.add_action(map_arg)
    for cmd in declare_params_cmds:
        ld.add_action(cmd)
        
    ld.add_action(gz_resource_path)
    ld.add_action(world_sdf_xacro)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(remove_temp_sdf_file)

    for group in robots_actions:
        ld.add_action(group)
    
    ld.add_action(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))

    return ld