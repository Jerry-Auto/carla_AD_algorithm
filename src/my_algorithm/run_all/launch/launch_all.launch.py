#!/usr/bin/env python3

import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def launch_custom_spawn_object(context, *args, **kwargs):
    """自定义车辆生成函数，确保车辆信息正确发布"""
    role_name = launch.substitutions.LaunchConfiguration('role_name').perform(context)
    spawn_point_param_name = 'spawn_point_' + role_name
    
    # 使用自定义的objects.json文件
    scenario_pkg = get_package_share_directory('scenario')
    custom_objects_json = os.path.join(scenario_pkg, 'config', 'objects.json')
    
    # 如果自定义文件不存在，使用默认文件
    if not os.path.exists(custom_objects_json):
        custom_objects_json = get_package_share_directory('carla_spawn_objects') + '/config/objects.json'
    
    # 使用carla_spawn_objects.launch.py而不是carla_example_ego_vehicle.launch.py
    # 这样可以传递更多参数
    carla_spawn_objects_pkg = get_package_share_directory('carla_spawn_objects')
    
    # 查找正确的launch文件
    spawn_objects_launch_file = os.path.join(carla_spawn_objects_pkg, 'carla_spawn_objects.launch.py')
    if not os.path.exists(spawn_objects_launch_file):
        spawn_objects_launch_file = os.path.join(carla_spawn_objects_pkg, 'launch', 'carla_spawn_objects.launch.py')
    
    carla_spawn_objects_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            spawn_objects_launch_file
        ),
        launch_arguments={
            'objects_definition_file': custom_objects_json,
            'role_name': role_name,
            spawn_point_param_name: launch.substitutions.LaunchConfiguration('spawn_point'),
            'spawn_sensors_only': 'False'
        }.items()
    )

    return [carla_spawn_objects_launch]

def launch_target_speed_publisher(context, *args, **kwargs):
    topic_name = "/carla/" + launch.substitutions.LaunchConfiguration('role_name').perform(context) + "/target_speed"
    data_string = "{'data': " + launch.substitutions.LaunchConfiguration('target_speed').perform(context) + "}"
    return [
        launch.actions.ExecuteProcess(
            output="screen",
            cmd=["ros2", "topic", "pub", topic_name,
                 "std_msgs/msg/Float64", data_string, "--qos-durability", "transient_local"],
            name='topic_pub_target_speed')]

def generate_launch_description():
    # 获取run_all包的路径，用于RViz配置文件
    run_all_pkg = get_package_share_directory('run_all')
    
    # 3. 启动车辆信息发布器（确保planning_agent能收到车辆信息）
    vehicle_info_publisher_node = launch_ros.actions.Node(
        package='run_all',
        executable='vehicle_info_publisher',
        name='vehicle_info_publisher',
        output='screen',
        parameters=[{
            'role_name': launch.substitutions.LaunchConfiguration('role_name'),
            'publish_frequency': 1.0
        }]
    )
    
    #  添加一个检查车辆信息是否发布的节点
    check_vehicle_info_node = launch_ros.actions.Node(
        package='run_all',
        executable='check_vehicle_info',
        name='check_vehicle_info',
        output='screen',
        parameters=[{
            'role_name': launch.substitutions.LaunchConfiguration('role_name'),
            'timeout': 10.0
        }]
    )

    # 6. 延迟6秒后启动RViz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(run_all_pkg, 'config/carla_ad_demo_ros2.rviz')],
        remappings=[
            ("carla/ego_vehicle/spectator_pose", "/carla/ego_vehicle/rgb_view/control/set_transform")
        ],
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('rviz')
        ),
        on_exit=launch.actions.Shutdown()
    )

    ld = launch.LaunchDescription([
        # 基本参数
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='True'  # 改为True以确保同步
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value='127.4,-195.4,2,0,0,180'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_speed',
            default_value='8.33' # in m/s
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sigterm_timeout',
            default_value='15'
        ),
        # RViz控制参数
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='是否启动RViz'
        ),

    
        # 1. 启动 CARLA ROS Bridge
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds'),
                'register_all_sensors': 'True',
                'register_all_actors': 'True'
            }.items()
        ),
        
        # 2. 延迟2秒后生成车辆（确保bridge已启动）
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch.actions.OpaqueFunction(function=launch_custom_spawn_object),
                launch.actions.OpaqueFunction(function=launch_target_speed_publisher),
            ]
        ),
        
        # 4. 延迟3秒后启动其他节点（确保车辆信息已发布）
        launch.actions.TimerAction(
            period=3.0,
            actions=[
                vehicle_info_publisher_node,
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory(
                            'carla_ad_agent'), 'carla_ad_agent.launch.py')
                    ),
                    launch_arguments={
                        'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                        'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk')
                    }.items()
                ),
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory(
                            'carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
                    ),
                    launch_arguments={
                        'host': launch.substitutions.LaunchConfiguration('host'),
                        'port': launch.substitutions.LaunchConfiguration('port'),
                        'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                        'role_name': launch.substitutions.LaunchConfiguration('role_name')
                    }.items()
                ),
            ]
        ),
        
        # 5. 手动控制节点（可选）
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),
        
        launch.actions.TimerAction(
            period=6.0,
            actions=[rviz_node]
        )
    ])
    
    return ld


if __name__ == '__main__':
    generate_launch_description()