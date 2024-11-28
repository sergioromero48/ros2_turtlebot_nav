import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories for packages
    custom_package_dir = get_package_share_directory('turtlebot_slam')
    cartographer_package_dir = get_package_share_directory('turtlebot3_cartographer')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include your custom world launch file
    custom_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(custom_package_dir, 'launch', 'turtlebot3_custom_tb3wrld.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Include Cartographer launch file for SLAM
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_package_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Launch description
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(custom_world_cmd)
    ld.add_action(cartographer_cmd)

    return ld
