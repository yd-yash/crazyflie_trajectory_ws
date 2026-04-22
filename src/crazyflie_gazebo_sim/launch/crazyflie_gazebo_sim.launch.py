"""
Launch file: crazyflie_gazebo_sim.launch.py

Starts the complete Crazyflie 2.1 Gazebo simulation:
  1. Gazebo Classic 11 with trajectory arena world
  2. Robot State Publisher (TF2 transforms)
  3. RViz2 for visualization

Usage:
  ros2 launch crazyflie_gazebo_sim crazyflie_gazebo_sim.launch.py

Optional args:
  ros2 launch crazyflie_gazebo_sim crazyflie_gazebo_sim.launch.py gui:=false
  ros2 launch crazyflie_gazebo_sim crazyflie_gazebo_sim.launch.py rviz:=false
"""

import os
from ament_python import get_package_share_directory  # noqa: F401
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────
    pkg_crazyflie_gazebo_sim = FindPackageShare('crazyflie_gazebo_sim')
    pkg_gazebo_ros            = FindPackageShare('gazebo_ros')

    # ── Launch arguments ───────────────────────────────────────────
    arg_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo with GUI (true/false)')

    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 (true/false)')

    arg_world = DeclareLaunchArgument(
        'world', default_value='crazyflie_trajectory_arena.world',
        description='Gazebo world file name')

    # ── Gazebo model path: tells Gazebo where our models are ───────
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([
            pkg_crazyflie_gazebo_sim, 'models'
        ])
    )

    # ── World file path ────────────────────────────────────────────
    world_file = PathJoinSubstitution([
        pkg_crazyflie_gazebo_sim, 'worlds',
        LaunchConfiguration('world')
    ])

    # ── Gazebo server + client ─────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'gui':   LaunchConfiguration('gui'),
            'verbose': 'false',
        }.items()
    )

    # ── Robot State Publisher (publishes TF from URDF) ─────────────
    urdf_file = PathJoinSubstitution([
        pkg_crazyflie_gazebo_sim, 'urdf', 'crazyflie_2_1.urdf'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cf21_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    os.path.dirname(__file__), '..', 'urdf', 'crazyflie_2_1.urdf'
                )
            ).read()
        }]
    )

    # ── RViz2 ──────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        pkg_crazyflie_gazebo_sim, 'rviz', 'crazyflie_sim.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='cf21_rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        arg_gui,
        arg_rviz,
        arg_world,
        set_gazebo_model_path,
        gazebo_launch,
        robot_state_publisher_node,
        rviz_node,
    ])
