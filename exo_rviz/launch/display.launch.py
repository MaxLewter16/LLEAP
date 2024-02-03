# Author: Max Lewter
# Date: November 10, 2022
# Description: Launch a basic mobile robot URDF file using Rviz.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare(package='exo_description').find('exo_description')
    launch_pkg_share = FindPackageShare(package='exo_rviz').find('exo_rviz')
    model = os.path.join(pkg_share, 'urdf/lleap_exo.urdf.xacro')
    robot_name_in_urdf = 'exo'
    rviz_config = os.path.join(launch_pkg_share, 'rviz/exo.rviz')
    use_sim_time = True
 
    # Publish the joint state values for the non-fixed joints in the URDF file.
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')
    
    # A GUI to manipulate the joint state values
    jspgui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)}],
        arguments=[model])
    
    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}])
    
    return LaunchDescription([
        jsp,
        jspgui,
        rsp,
        rviz
    ])

