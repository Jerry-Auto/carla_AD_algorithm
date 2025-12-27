import os
import glob
import subprocess
import launch
import yaml
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction


def find_config_dir():
    """
    Try to locate the config directory of carla_ad_demo.
    Prefer source layout if available, otherwise use installed share directory.
    """
    pkg_share = get_package_share_directory('carla_ad_demo')
    
    # Heuristic: try common source locations
    possible_roots = [
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..')),  # back to workspace root
        '/ros-bridge/carla-ros-bridge',
        os.path.expanduser('~/carla-ros-bridge'),
        os.getcwd()
    ]

    for root in possible_roots:
        candidate = os.path.join(root, 'src', 'ros-bridge', 'carla_ad_demo', 'config')
        if os.path.isdir(candidate) and glob.glob(os.path.join(candidate, "*.xosc")):
            return candidate

        candidate2 = os.path.join(root, 'carla_ad_demo', 'config')
        if os.path.isdir(candidate2) and glob.glob(os.path.join(candidate2, "*.xosc")):
            return candidate2

    # Fallback: use installed config
    installed_config = os.path.join(pkg_share, 'config')
    if os.path.isdir(installed_config) and glob.glob(os.path.join(installed_config, "*.xosc")):
        return installed_config

    return None


def prepare_and_publish_scenarios(context):
    """
    Locate config dir (source preferred), collect .xosc files, and publish scenario list.
    NO file modification is performed.
    """
    config_dir = find_config_dir()
    if config_dir is None:
        print("⚠️  No config directory with .xosc files found. Publishing empty scenario list.")
        scenarios = []
    else:
        xosc_files = glob.glob(os.path.join(config_dir, "*.xosc"))
        scenarios = []
        for f in sorted(xosc_files):
            name = os.path.splitext(os.path.basename(f))[0]
            scenarios.append({'name': name, 'file': f})
        print(f"📁 Found {len(scenarios)} scenario(s) in: {config_dir}")

    # Generate YAML
    if scenarios:
        scenario_list = {
            'scenarios': [
                {'name': s['name'], 'scenario_file': s['file']}
                for s in scenarios
            ]
        }
        yaml_str = yaml.dump(scenario_list, default_flow_style=False)
    else:
        yaml_str = "scenarios: []\n"

    # Safely publish via ros2 topic pub
    safe_yaml = yaml_str.strip().replace("'", "'\"'\"'")
    cmd = [
        "sh", "-c",
        f"ros2 topic pub --once /carla/available_scenarios "
        f"carla_ros_scenario_runner_types/CarlaScenarioList "
        f"'{safe_yaml}'"
    ]
    env = os.environ.copy()
    subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("📤 Published available scenarios to /carla/available_scenarios")
    return []


def generate_launch_description():
    ld = launch.LaunchDescription([
        OpaqueFunction(function=prepare_and_publish_scenarios),

        launch.actions.DeclareLaunchArgument(name='host', default_value='localhost'),
        launch.actions.DeclareLaunchArgument(name='port', default_value='2000'),
        launch.actions.DeclareLaunchArgument(name='town', default_value='Town01'),
        launch.actions.DeclareLaunchArgument(name='timeout', default_value='2'),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(name='fixed_delta_seconds', default_value='0.05'),
        launch.actions.DeclareLaunchArgument(
            name='scenario_runner_path',
            default_value=os.environ.get('SCENARIO_RUNNER', '/ros-bridge/carla/scenario_runner-0.9.13'),
            description='Path to CARLA ScenarioRunner root directory'
        ),
        launch.actions.DeclareLaunchArgument(name='role_name', default_value='ego_vehicle'),

        launch_ros.actions.Node(
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name='carla_twist_to_control',
            remappings=[
                (
                    ["/carla/", launch.substitutions.LaunchConfiguration('role_name'), "/vehicle_control_cmd"],
                    ["/carla/", launch.substitutions.LaunchConfiguration('role_name'), "/vehicle_control_cmd_manual"]
                )
            ],
            parameters=[{'role_name': launch.substitutions.LaunchConfiguration('role_name')}]
        ),

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
                'tf_sensor.publish_sensor_tf': 'true'
            }.items()
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'object_definition_file': os.path.join(
                    get_package_share_directory('carla_spawn_objects'), 'config', 'objects.json'
                ),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),

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

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_ros_scenario_runner'), 'carla_ros_scenario_runner.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'scenario_runner_path': launch.substitutions.LaunchConfiguration('scenario_runner_path'),
                'wait_for_ego': 'True',
                'traffic_manager_port': '8000'
            }.items()
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                ("carla/ego_vehicle/spectator_pose", "/carla/ego_vehicle/rgb_view/control/set_transform")
            ],
            arguments=['-d', os.path.join(get_package_share_directory('carla_ad_demo'), 'config/carla_ad_demo_ros2.rviz')],
            on_exit=launch.actions.Shutdown()
        ),
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