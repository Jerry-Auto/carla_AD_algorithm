import os
import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

def find_scenarios():
    """
    Locate the config directory of scenario package and find .xosc files.
    """
    try:
        pkg_share = get_package_share_directory('scenario')
        config_dir = os.path.join(pkg_share, 'config')
        if os.path.isdir(config_dir):
            return config_dir, glob.glob(os.path.join(config_dir, "*.xosc"))
    except Exception:
        pass
    return None, []

def prepare_and_publish_scenarios(context):
    """
    Collect .xosc files from scenario package and publish scenario list.
    """
    config_dir, xosc_files = find_scenarios()
    
    scenarios = []
    if config_dir:
        for f in sorted(xosc_files):
            name = os.path.splitext(os.path.basename(f))[0]
            scenarios.append(f"{{name: '{name}', scenario_file: '{f}'}}")
        print(f"📁 Found {len(scenarios)} scenario(s) in: {config_dir}")
    else:
        print("⚠️  No scenario directory found.")

    # Generate YAML string for CarlaScenarioList
    if scenarios:
        yaml_str = "{scenarios: [\n" + ", \n".join(scenarios) + "\n]}"
    else:
        yaml_str = "{scenarios: []}"

    # Safely publish via ros2 topic pub
    # We use a separate process to publish this continuously so RViz can pick it up anytime
    cmd = [
        "ros2", "topic", "pub", 
        "--rate", "1.0",
        "--qos-durability", "transient_local",
        "/carla/available_scenarios",
        "carla_ros_scenario_runner_types/msg/CarlaScenarioList",
        yaml_str
    ]
    
    return [launch.actions.ExecuteProcess(
        cmd=cmd,
        output='screen',
        name='publish_available_scenarios'
    )]

def launch_custom_spawn_object(context, *args, **kwargs):
    """Spawn vehicle using objects.json from scenario package"""
    role_name = launch.substitutions.LaunchConfiguration('role_name').perform(context)
    spawn_point_param_name = 'spawn_point_' + role_name
    
    # Use objects.json from scenario package
    try:
        scenario_pkg = get_package_share_directory('scenario')
        custom_objects_json = os.path.join(scenario_pkg, 'config', 'objects.json')
    except:
        custom_objects_json = ''

    # Fallback to default if not found
    if not os.path.exists(custom_objects_json):
        print(f"⚠️  Custom objects.json not found at {custom_objects_json}, using default.")
        custom_objects_json = os.path.join(get_package_share_directory('carla_spawn_objects'), 'config', 'objects.json')
    
    carla_spawn_objects_pkg = get_package_share_directory('carla_spawn_objects')
    spawn_objects_launch_file = os.path.join(carla_spawn_objects_pkg, 'carla_spawn_objects.launch.py')
    if not os.path.exists(spawn_objects_launch_file):
         spawn_objects_launch_file = os.path.join(carla_spawn_objects_pkg, 'launch', 'carla_spawn_objects.launch.py')

    return [launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(spawn_objects_launch_file),
        launch_arguments={
            'objects_definition_file': custom_objects_json,
            'role_name': role_name,
            spawn_point_param_name: launch.substitutions.LaunchConfiguration('spawn_point'),
            'spawn_sensors_only': 'False'
        }.items()
    )]

def generate_launch_description():
    run_all_pkg = get_package_share_directory('run_all')
    
    ld = launch.LaunchDescription([
        # Arguments
        launch.actions.DeclareLaunchArgument(name='host', default_value='localhost'),
        launch.actions.DeclareLaunchArgument(name='port', default_value='2000'),
        launch.actions.DeclareLaunchArgument(name='town', default_value='Town04'),      #这里修改地图信息
        launch.actions.DeclareLaunchArgument(name='timeout', default_value='10'),
        launch.actions.DeclareLaunchArgument(name='synchronous_mode_wait_for_vehicle_control_command', default_value='False'),
        launch.actions.DeclareLaunchArgument(name='fixed_delta_seconds', default_value='0.05'),
        launch.actions.DeclareLaunchArgument(name='role_name', default_value='ego_vehicle'),
        launch.actions.DeclareLaunchArgument(name='spawn_point', default_value='127.4,-195.4,2,0,0,180'),
        launch.actions.DeclareLaunchArgument(
            name='scenario_runner_path',
            default_value=os.environ.get('SCENARIO_RUNNER', '/ros-bridge/carla/scenario_runner-0.9.13'),
            description='Path to CARLA ScenarioRunner root directory'
        ),
        launch.actions.DeclareLaunchArgument(name='wait_for_ego', default_value='True'),

        # 1. Publish available scenarios
        OpaqueFunction(function=prepare_and_publish_scenarios),

        # 2. CARLA ROS Bridge
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command':
                    launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds'),
                'register_all_sensors': 'True',
                'register_all_actors': 'True'
            }.items()
        ),

        # 3. Spawn Objects (Vehicle) - Delayed to ensure bridge is up
        launch.actions.TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=launch_custom_spawn_object)]
        ),

        # 4. Waypoint Publisher
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),

        # 5. Scenario Runner
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_ros_scenario_runner'), 'carla_ros_scenario_runner.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'scenario_runner_path': launch.substitutions.LaunchConfiguration('scenario_runner_path'),
                'wait_for_ego': launch.substitutions.LaunchConfiguration('wait_for_ego')
            }.items()
        ),

        # 6. User's PNC Agent (Planning & Control)
        # Ensure vehicle info is published
        launch_ros.actions.Node(
            package='run_all',
            executable='vehicle_info_publisher',
            name='vehicle_info_publisher',
            output='screen',
            parameters=[{
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'publish_frequency': 1.0
            }]
        ),
        launch_ros.actions.Node(
            package='run_all',
            executable='vehicle_info_checker',
            name='check_vehicle_info',
            output='screen',
            parameters=[{
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'timeout': 10.0
            }]
        ),
        
        # RViz Respawn Tool (2D Pose Estimate -> Teleport)
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_spawn_objects'), 'set_initial_pose.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'control_id': 'vehicle_control'
            }.items()
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pnc_agent'), 'launch', 'my_pnc_launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),

        # 7. RViz
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # Use the config from run_all or fallback to carla_ad_demo
            arguments=['-d', os.path.join(run_all_pkg, 'config/carla_ad_demo_ros2.rviz')],
            remappings=[
                ("carla/ego_vehicle/spectator_pose", "/carla/ego_vehicle/rgb_view/control/set_transform")
            ],
            on_exit=launch.actions.Shutdown()
        ),
        
        # 8. TF Publisher (Map to Carla World)
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'carla_world'],
            name='map_to_carla_world_tf'
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
