from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Resurrect the Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            namespace='j100_0000',  # <--- THIS IS THE FIX
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # 2. Resurrect the Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner',
            namespace='j100_0000',  # <--- THIS IS THE FIX
            arguments=['platform_velocity_controller', '--controller-manager-timeout', '60'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])