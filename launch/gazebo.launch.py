import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription,
                             ExecuteProcess,
                             RegisterEventHandler,
                             TimerAction)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg         = get_package_share_directory('my_robot')
    gazebo_ros  = get_package_share_directory('gazebo_ros')

    # Process xacro → urdf string
    xacro_file  = os.path.join(pkg, 'urdf', 'my_robot.urdf.xacro')
    robot_desc  = xacro.process_file(xacro_file).toxml()

    world_file  = os.path.join(pkg, 'worlds', 'obstacles.world')

    # ── Step 1: Start gzserver with ROS factory plugin ────────────────
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',       # ROS init plugin
            '-s', 'libgazebo_ros_factory.so',     # spawn_entity plugin ← KEY
            world_file
        ],
        output='screen',
    )

    # ── Step 2: Start gzclient (the visual window) ────────────────────
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
    )

    # ── Step 3: Robot state publisher ─────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen',
    )

    # ── Step 4: Spawn robot — wait 5s for Gazebo to fully load ────────
    spawn_robot = TimerAction(
        period=5.0,    # wait 5 seconds before spawning
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.10',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        robot_state_pub,
        gzserver,
        gzclient,
        spawn_robot,
    ])