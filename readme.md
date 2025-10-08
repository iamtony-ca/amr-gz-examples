# custom amr based on turtlebot4 in ros2 jazzy
  
```bash
colcon build --packages-select nav2_minimal_tb4_description nav2_bringup nav2_minimal_tb3_sim nav2_minimal_tb4_sim
```

```bash
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False
```
or  
```bash
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False x_pose:=0.0
```

```bash
ros2 run tf2_tools view_frames
```


# map to gz
https://github.com/Adlink-ROS/map2gazebo.git
  
# custom 3dmap
[turtlebot_ws/src/navigation2/nav2_bringup/maps]  
pgm, yaml 추가  
  
  
[turtlebot_ws/src/nav2_minimal_turtlebot_simulation/nav2_minimal_tb4_sim]  
models 폴더 및 하위파일들 추가  
CMakelists.txt -> models 추가  
worlds -> depot.sdf -> <uri>models://map</uri> + plane 추가(sandbox꺼로 복붙가능.)  

  
[turtlebot_ws/src/navigation2/nav2_bringup/launch/tb4_simulation_launch.py]  
아래 처럼 변경.  
```python
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(sim_dir, 'worlds')+ ':' + os.path.join(sim_dir, 'models'))
```
  
[turtlebot_ws/src/nav2_minimal_turtlebot_simulation/nav2_minimal_tb4_sim/launch/spawn_tb4.launch.py]
gazebo 상에 로봇 initial pose 변경 원할시 launch arg 로 하던가 혹은  
아래 내용들 변경 가능.  
```python
    pose = {'x': LaunchConfiguration('x_pose', default='-8.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

```