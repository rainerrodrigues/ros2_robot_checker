import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('your_sim_package'), 'worlds', 'task_scene.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_description_content = subprocess.check_output([
        'xacro', os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.urdf.xacro'),
        'ur_type:=ur5'
    ]).decode('utf-8')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    #Spawning the robot in Gazebo environment
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur5_arm', '-z', '0.1']
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot
    ])
