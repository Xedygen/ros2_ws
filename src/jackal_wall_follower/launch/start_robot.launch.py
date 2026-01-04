from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    wall_follow_node = Node(
        package='jackal_wall_follower',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{'use_sim_time': True}], 
        remappings=[
            ('/scan', '/j100_0000/sensors/lidar3d_0/scan'),
            ('/cmd_vel', '/j100_0000/cmd_vel'),
            ('/odom', '/j100_0000/platform/odom')
        ]
    )

    return LaunchDescription([
        wall_follow_node
    ])