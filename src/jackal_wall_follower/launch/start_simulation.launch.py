from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_path = os.path.join(os.environ['HOME'], 'ros2_ws/worlds/generated_maze')

    clearpath_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('clearpath_gz'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'x': '-17.25',
            'y': '-17.25',
            'paused': 'true' 
        }.items()
    )

    return LaunchDescription([
        clearpath_launch
    ])