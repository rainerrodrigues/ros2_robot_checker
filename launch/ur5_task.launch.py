import os
import subprocess

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'ros_robot_checker'   # 👈 THIS is your package
    pkg_path = get_package_share_directory(pkg_name)

    # -------- World --------
    world_file = os.path.join(pkg_path, 'worlds', 'task_scene.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r', world_file]
        }.items(),
    )

    # -------- Robot Description --------
    robot_description = subprocess.check_output([
        'xacro',
        os.path.join(
            get_package_share_directory('ur_description'),
            'urdf',
            'ur.urdf.xacro'
        ),
        'ur_type:=ur5'
    ]).decode()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # -------- Spawn Robot --------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ur5',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])

