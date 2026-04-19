from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to Yahboom's original X3 launch file
    yahboom_pkg_path = get_package_share_directory('yahboomcar_bringup')
    launch_file_path = os.path.join(yahboom_pkg_path, 'launch', 'yahboomcar_bringup_X3_launch.py')

    return LaunchDescription([
        # ROBOT 1 Group
        GroupAction(
            actions=[
                PushRosNamespace('robot1'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file_path)
                ),
            ]
        ),

        # ROBOT 2 Group
        GroupAction(
            actions=[
                PushRosNamespace('robot2'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file_path)
                ),
            ]
        ),
    ])