#!/usr/bin/env python3

import glob
import os

import launch
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

def generate_launch_description():
    # 获取相关包的路径
    carla_ros_scenario_runner_pkg = get_package_share_directory('carla_ros_scenario_runner')
    carla_spawn_objects_pkg = get_package_share_directory('carla_spawn_objects')
    scenario_pkg = get_package_share_directory('scenario')

    def _find_scenarios():
        config_dir = os.path.join(scenario_pkg, 'config')
        if not os.path.isdir(config_dir):
            return None, []
        return config_dir, glob.glob(os.path.join(config_dir, '*.xosc'))

    def _prepare_and_publish_scenarios(context, *args, **kwargs):
        config_dir, xosc_files = _find_scenarios()

        scenarios = []
        if config_dir:
            for f in sorted(xosc_files):
                name = os.path.splitext(os.path.basename(f))[0]
                scenarios.append(f"{{name: '{name}', scenario_file: '{f}'}}")

        # Fallback: if config dir is missing or empty, at least publish the selected scenario_file
        if not scenarios:
            scenario_file = launch.substitutions.LaunchConfiguration('scenario_file').perform(context)
            if scenario_file:
                name = os.path.splitext(os.path.basename(scenario_file))[0]
                scenarios = [f"{{name: '{name}', scenario_file: '{scenario_file}'}}"]

        if scenarios:
            yaml_str = "{scenarios: [\n" + ", \n".join(scenarios) + "\n]}"
        else:
            yaml_str = '{scenarios: []}'

        cmd = [
            'ros2', 'topic', 'pub',
            '--rate', '1.0',
            '--qos-durability', 'transient_local',
            '/carla/available_scenarios',
            'carla_ros_scenario_runner_types/msg/CarlaScenarioList',
            yaml_str,
        ]

        return [
            launch.actions.ExecuteProcess(
                cmd=cmd,
                output='screen',
                name='publish_available_scenarios',
            )
        ]

    # 声明 Launch 参数
    scenario_runner_path_arg = launch.actions.DeclareLaunchArgument(
        name='scenario_runner_path',
        default_value=os.environ.get('SCENARIO_RUNNER', os.environ.get('SCENARIO_RUNNER_ROOT', '')),
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

    enable_scenario_runner_arg = launch.actions.DeclareLaunchArgument(
        name='enable_scenario_runner',
        default_value='True',
        description='Enable carla_ros_scenario_runner launch'
    )

    enable_spawn_ego_arg = launch.actions.DeclareLaunchArgument(
        name='enable_spawn_ego',
        default_value='True',
        description='Enable spawning ego vehicle (carla_example_ego_vehicle)'
    )

    enable_available_scenarios_arg = launch.actions.DeclareLaunchArgument(
        name='enable_available_scenarios',
        default_value='True',
        description='Enable publishing /carla/available_scenarios list for RViz'
    )

    enable_vehicle_info_checker_arg = launch.actions.DeclareLaunchArgument(
        name='enable_vehicle_info_checker',
        default_value='True',
        description='Enable vehicle_info_checker (helps ensure ego info is available)'
    )

    enable_road_width_pub_arg = launch.actions.DeclareLaunchArgument(
        name='enable_road_width_pub',
        default_value='True',
        description='Enable road width publishing (/carla/road_boundaries)'
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
            'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point')
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

    # 6. 启动道路宽度发布节点
    road_width_pub_node = launch_ros.actions.Node(
        package='scenario',
        executable='road_width_pub.py',
        name='road_width_publisher',
        output='screen',
        parameters=[{
            'carla_host': launch.substitutions.LaunchConfiguration('host'),
            'carla_port': launch.substitutions.LaunchConfiguration('port'),
            'role_name': launch.substitutions.LaunchConfiguration('role_name'),
            'publish_topic': '/carla/road_boundaries'
        }]
    )

    scenario_runner_launch = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('enable_scenario_runner')
        ),
        actions=[scenario_runner_launch],
    )

    spawn_and_publish_group = launch.actions.GroupAction(
        actions=[
            launch.actions.TimerAction(
                period=2.0,
                actions=[
                    launch.actions.GroupAction(
                        condition=launch.conditions.IfCondition(
                            launch.substitutions.LaunchConfiguration('enable_spawn_ego')
                        ),
                        actions=[ego_vehicle_launch],
                    ),
                    launch.actions.GroupAction(
                        condition=launch.conditions.IfCondition(
                            launch.substitutions.LaunchConfiguration('enable_available_scenarios')
                        ),
                        actions=[OpaqueFunction(function=_prepare_and_publish_scenarios)],
                    ),
                ],
            )
        ]
    )

    post_spawn_group = launch.actions.GroupAction(
        actions=[
            launch.actions.TimerAction(
                period=4.0,
                actions=[
                    launch.actions.GroupAction(
                        condition=launch.conditions.IfCondition(
                            launch.substitutions.LaunchConfiguration('enable_vehicle_info_checker')
                        ),
                        actions=[vehicle_info_checker],
                    ),
                    launch.actions.GroupAction(
                        condition=launch.conditions.IfCondition(
                            launch.substitutions.LaunchConfiguration('enable_road_width_pub')
                        ),
                        actions=[road_width_pub_node],
                    ),
                ],
            )
        ]
    )

    execute_group = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('auto_execute')
        ),
        actions=[execute_scenario_cmd],
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
        enable_scenario_runner_arg,
        enable_spawn_ego_arg,
        enable_available_scenarios_arg,
        enable_vehicle_info_checker_arg,
        enable_road_width_pub_arg,

        # 1) Scenario runner (optional)
        scenario_runner_launch,
        # 2) Spawn ego + publish scenario list (optional)
        spawn_and_publish_group,
        # 3) Post-spawn helpers (optional)
        post_spawn_group,
        # 4) Auto execute scenario (optional)
        execute_group,
    ])