from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # set_camera_pose = SetEnvironmentVariable(
    #     name='GZ_GUI_CAMERA_POSE',
    #     value='-10 -10 15 0 0.7 0.8'
    # )
    
    clearpath_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('clearpath_gz'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': '~/ros2_ws/worlds/generated_maze',
            'x': '-6.3',
            'y': '-6.3'
            }.items()
    )

    wall_follow_node = Node(
        package='jackal_wall_follower',
        executable='main_node',
        name='main_node',
        output='screen',
        remappings=[
            ('/scan', '/j100_0000/sensors/lidar3d_0/scan'),
            ('/cmd_vel', '/j100_0000/cmd_vel')
        ]
    )

    delayed_wall_follow = TimerAction(
        period=5.0,
        actions=[wall_follow_node]
    )

    return LaunchDescription([
        # set_camera_pose,
        clearpath_launch,
        delayed_wall_follow
    ])