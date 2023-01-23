import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='preliminary-simulation').find('preliminary-simulation')
    urdf_path = os.path.join(pkg_share, 'models/lleap_exo.urdf.xacro')
    gazebo_sim_file = os.path.join(pkg_share, 'models', 'gazebo_sim.xacro')
    return LaunchDescription([
        DeclareLaunchArgument(
            'paused', default_value='False',
            description='Start gazebo in paused state'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation time when running gazebo'),
        DeclareLaunchArgument(
            'gui', default_value='True',
            description='Start gazebo with GUI'),
        DeclareLaunchArgument(
            'headless', default_value   ='False',
            description='Start gazebo in headless mode'),
        DeclareLaunchArgument(
            'debug', default_value='False',
            description='Start gazebo in debug mode'),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', LaunchConfiguration('gui'), LaunchConfiguration('paused'), LaunchConfiguration('use_sim_time'), LaunchConfiguration('headless'), LaunchConfiguration('debug'), '-s', 'libgazebo_ros_init.so'],
            output='screen'),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_model',
        #     name='spawn_model',
        #     arguments=['-urdf', '-model', 'lleap_exo', '-param', 'robot_description', urdf_path, '-x', '0', '-y', '0', '-z', '0', '-R', '0', '-P', '0', '-Y', '0']
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=['-robot_namespace', 'lleap_exo', '-urdf', '-use_gui', 'True', urdf_path]
        ),
        Node(
            package='gazebo_ros',
            executable='gazebo_ros',
            name='gazebo_ros',
            arguments=[gazebo_sim_file]
        )
    ])