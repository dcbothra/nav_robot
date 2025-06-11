import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    pkg_nav      = get_package_share_directory('nav_robot')
    pkg_nav2_br  = get_package_share_directory('nav2_bringup')

    map_yaml     = os.path.join(pkg_nav, 'maps', 'amcl_map.yaml')
    amcl_params  = os.path.join(pkg_nav, 'config', 'amcl.yaml')
    nav2_params  = os.path.join(pkg_nav2_br, 'params', 'nav2_params.yaml')

    world = os.path.join(pkg_nav, 'worlds', 'gazebo.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'use_sim_time' : 'true','world': world}.items()
             )

    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_nav,'launch','display.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-topic', 'robot_description','-entity', 'my_bot'],
        output = 'screen'
    )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
    #     output='screen'
    # )

    # initial_pose_node = Node(
    #     package='nav_robot',
    #     executable='set_initial_pose.py',
    #     name='set_initial_pose',
    #     output='screen'
    # )

    amcl_node = Node(
        package = 'nav2_amcl',
        executable = 'amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': True}, amcl_params]
    )

    nav_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
                launch_arguments={
                    'use_sim_time' : 'true',
                    'map' : map_yaml,
                    'params_file' : nav2_params
                }.items()
             )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}]
    ) 

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            # 'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator', 'waypoint_follower']
            'node_names': ['map_server', 'amcl']
        }]
    )

    delayed_lifecycle_node = TimerAction(
        period=10.0,  # seconds
        actions=[lifecycle_node]
    )

    delayed_nav_node = TimerAction(
        period=5.0,  # seconds
        actions=[nav_node]
    )

    # delayed_initial_pose_node = TimerAction(
    #     period=5.0,  # seconds
    #     actions=[initial_pose_node]
    # )

    battery_node = Node(
            package='nav_robot',
            executable='battery_sim_node',
            name='battery_simulator',
            output='screen',
            parameters=[{
                'battery_capacity': 100.0,  # optional parameters
                'battery_drain_rate': 0.05
            }]
        )

    return LaunchDescription(
        [
            gazebo,
            rsp,
            spawn_entity,
            map_server_node,
            battery_node,
            # delayed_initial_pose_node,
            # static_tf,
            amcl_node,
            delayed_lifecycle_node,
            delayed_nav_node,
        ]
    )