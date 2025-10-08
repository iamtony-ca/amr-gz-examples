import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 경로 및 설정 ---
    bringup_dir = get_package_share_directory('nav2_bringup')
    robot_config_path = os.path.join(bringup_dir, 'config', 'tb4_robots.yaml')
    
    # --- 2. 런치 파라미터 선언 ---
    map_arg = DeclareLaunchArgument('map', 
        default_value=os.path.join(bringup_dir, 'maps', 'depot.yaml'))
    
    # --- 3. LaunchDescription 객체 생성 ---
    ld = LaunchDescription()
    ld.add_action(map_arg)

    # --- 4. 로봇 설정 파일 로드 ---
    with open(robot_config_path, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    # --- 5. 각 로봇에 대한 Nav2 및 RViz 실행 ---
    for robot in robots:
        namespace = robot['name']
        
        params_file_arg = DeclareLaunchArgument(f'{namespace}_params_file',
            default_value=os.path.join(bringup_dir, 'params', f'nav2_multirobot_params_{namespace[-1]}.yaml'))

        params_file = LaunchConfiguration(f'{namespace}_params_file')

        # Nav2 스택 실행
        nav2_stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'True',
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'True',
                'params_file': params_file,
                'autostart': 'True',
            }.items()
        )

        # RViz 실행
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        ld.add_action(params_file_arg)
        ld.add_action(nav2_stack)
        ld.add_action(rviz)

    return ld