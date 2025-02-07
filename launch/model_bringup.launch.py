from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the display_zed_cam.launch.py file
    # zed_display_launch_file = os.path.join(
    #     get_package_share_directory('zed_display_rviz2'),
    #     'launch',
    #     'display_zed_cam.launch.py'
    # )

    # Get the path to the parameters.yaml file
    config_file_path = os.path.join(
        get_package_share_directory('human_kinematics_ros'),
        'config',
        'parameters.yaml'
    )

    return LaunchDescription([
        # # Launch the display_zed_cam.launch.py file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(zed_display_launch_file),
        #     launch_arguments={'camera_model': 'zed2'}.items()
        # ),

        # Launch the human_kinematics_node
        Node(
            package='human_kinematics_ros',
            executable='human_kinematics_node',
            name='human_kinematics_publisher',
            output='screen',
            parameters=[config_file_path],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    ])