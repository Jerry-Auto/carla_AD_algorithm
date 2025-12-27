#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
import launch.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取相关包的路径
    carla_ros_scenario_runner_pkg = get_package_share_directory('carla_ros_scenario_runner')
    carla_spawn_objects_pkg = get_package_share_directory('carla_spawn_objects')
    scenario_pkg = get_package_share_directory('scenario')

    # 声明 Launch 参数
    scenario_runner_path_arg = launch.actions.DeclareLaunchArgument(
        name='scenario_runner_path',
        default_value=os.environ.get('SCENARIO_RUNNER_ROOT', ''),
        description='Path to the scenario_runner root directory'
    )

    scenario_file_arg = launch.actions.DeclareLaunchArgument(
        name='scenario_file',
        default_value=os.path.join(scenario_pkg, 'config', 'FollowLeadingVehicle.xosc'),
        description='Path to the OpenSCENARIO (.xosc) file to execute'
    )
    
    wait_for_ego_arg = launch.actions.DeclareLaunchArgument(
        name='wait_for_ego',
        default_value='True',
        description='Whether scenario runner should wait for ego vehicle'
    )

    auto_execute_arg = launch.actions.DeclareLaunchArgument(
        name='auto_execute',
        default_value='False',
        description='If True, automatically call /scenario_runner/execute_scenario after startup'
    )

    host_arg = launch.actions.DeclareLaunchArgument(
        name='host',
        default_value='localhost',
        description='CARLA server IP'
    )

    port_arg = launch.actions.DeclareLaunchArgument(
        name='port',
        default_value='2000',
        description='CARLA server port'
    )
    
    role_name_arg = launch.actions.DeclareLaunchArgument(
        name='role_name',
        default_value='ego_vehicle',
        description='Role name of the ego vehicle'
    )

    spawn_point_arg = launch.actions.DeclareLaunchArgument(
        name='spawn_point',
        default_value='127.4,-195.4,2,0,0,180',
        description='Spawn point for the ego vehicle'
    )

    # 1. 启动 Scenario Runner 节点
    scenario_runner_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(carla_ros_scenario_runner_pkg, 'carla_ros_scenario_runner.launch.py')
        ),
        launch_arguments={
            'host': launch.substitutions.LaunchConfiguration('host'),
            'port': launch.substitutions.LaunchConfiguration('port'),
            'role_name': launch.substitutions.LaunchConfiguration('role_name'),
            'scenario_runner_path': launch.substitutions.LaunchConfiguration('scenario_runner_path'),
            'wait_for_ego': launch.substitutions.LaunchConfiguration('wait_for_ego')
        }.items()
    )

    # 2. 使用官方的carla_example_ego_vehicle.launch.py来生成车辆
    # 这会同时生成车辆、传感器和设置初始位姿
    ego_vehicle_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(carla_spawn_objects_pkg, 'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'role_name': launch.substitutions.LaunchConfiguration('role_name'),
            'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point'),
            'objects_definition_file': os.path.join(scenario_pkg, 'config', 'objects.json')
        }.items()
    )

    # 3. 自动调用服务执行场景 (延迟几秒等待节点就绪)
    execute_scenario_cmd = launch.actions.TimerAction(
        period=8.0, # 等待8秒让车辆信息完全发布
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('auto_execute')
        ),
        actions=[
            launch.actions.ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', 
                    '/scenario_runner/execute_scenario', 
                    'carla_ros_scenario_runner_types/srv/ExecuteScenario',
                    launch.substitutions.PythonExpression([
                        '"', "{scenario: {scenario_file: '", 
                        launch.substitutions.LaunchConfiguration('scenario_file'), 
                        "'}}", '"'
                    ])
                ],
                output='screen',
                name='execute_scenario_service_call'
            )
        ]
    )

    # 4. 车辆信息检查节点 - 确保vehicle_info话题被发布
    # 创建一个简单的节点来检查vehicle_info话题是否存在
    vehicle_info_checker = launch_ros.actions.Node(
        package='run_all',  # 假设我们在run_all包中创建这个节点
        executable='vehicle_info_checker',
        name='vehicle_info_checker',
        output='screen',
        parameters=[{
            'role_name': launch.substitutions.LaunchConfiguration('role_name'),
            'timeout': 10.0
        }]
    )

    # 5. 发布可用场景列表（用于RViz界面）
    available_scenarios_publisher = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--qos-durability', 'transient_local',
            '--rate', '1',
            '/carla/available_scenarios',
            'carla_ros_scenario_runner_types/msg/CarlaScenarioList',
            launch.substitutions.PythonExpression([
                '"{scenarios: [{name: \\"FollowLeadingVehicle\\", scenario_file: \\"', 
                launch.substitutions.LaunchConfiguration('scenario_file'), 
                '\\"}]}"'
            ])
        ],
        output='screen',
        name='available_scenarios_publisher'
    )

    return launch.LaunchDescription([
        scenario_runner_path_arg,
        scenario_file_arg,
        wait_for_ego_arg,
        auto_execute_arg,
        host_arg,
        port_arg,
        role_name_arg,
        spawn_point_arg,
        
        # 启动顺序：
        # 1. 先启动scenario runner（准备好等待车辆）
        scenario_runner_launch,
        
        # 2. 延迟2秒后生成车辆（确保scenario runner已准备好）
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                ego_vehicle_launch,
                available_scenarios_publisher,
            ]
        ),
        
        # 3. 延迟4秒后检查车辆信息
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                vehicle_info_checker,
            ]
        ),
        
        # 4. 最后执行场景（如果需要）
        execute_scenario_cmd,
    ])