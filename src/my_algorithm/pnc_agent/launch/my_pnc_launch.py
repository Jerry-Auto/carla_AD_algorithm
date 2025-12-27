import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pnc_agent',
            executable='planning_agent',
            name='planning_agent',
            output='screen',
            parameters=[{
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'planning_time_step': 0.2
            }]
        )
        ,
        launch_ros.actions.Node(
            package='pnc_agent',
            executable='control_agent',
            name='control_agent',
            output='screen',
            parameters=[{
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'control_time_step': 0.02
            }]
        )
    ])