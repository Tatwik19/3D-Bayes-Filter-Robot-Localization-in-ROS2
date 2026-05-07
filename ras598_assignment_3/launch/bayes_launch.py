from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    stage_share = get_package_share_directory('stage_ros2')
    assignment_share = get_package_share_directory('ras598_assignment_3')

    stage_launch = os.path.join(stage_share, 'launch', 'demo.launch.py')
    rviz_config = os.path.join(assignment_share, 'config', 'bayes.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage_launch),
            launch_arguments={
                'world': 'cave',
                'use_stamped_velocity': 'true',
            }.items(),
        ),

        Node(
            package='ras598_assignment_3',
            executable='bayes',
            name='bayes_filter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])