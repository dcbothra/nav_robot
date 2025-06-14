import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
# from launch.actions import TimerAction
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory('nav_robot'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    rviz_path = os.path.join(pkg_path, 'config', 'view_bot.rviz')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package ='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters = [params]
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', 
                default_value='true',
                description='Use sim time if true'
            ),
            # joint_state_publisher_node,
            node_robot_state_publisher,
            rviz_node,
        ]
    )