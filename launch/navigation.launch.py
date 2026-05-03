import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    pkg            = get_package_share_directory('my_robot')
    nav2_bringup   = get_package_share_directory('nav2_bringup')
    nav2_params    = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file       = os.path.join(pkg, 'maps',   'my_map.yaml')

    # ── Launch arguments (override from CLI if needed) ──────────────────
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file',
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to nav2 params yaml',
    )
    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz?',
    )

    # ── Nav2 bringup (localization + navigation) ─────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          LaunchConfiguration('map'),
            'params_file':  LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
        }.items(),
    )

    # ── RViz with Nav2 default config ────────────────────────────────────
    rviz_config = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Your custom A* planner node ──────────────────────────────────────
    astar = Node(
        package='my_robot',
        executable='astar_planner.py',
        name='astar_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_rviz,
        nav2,
        rviz,
        astar,
    ])