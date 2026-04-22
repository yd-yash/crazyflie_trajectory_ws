"""
Launch file: gazebo_cf1_sim.launch.py
======================================
Launches the complete Gazebo simulation environment for one Crazyflie 2.1.

What this launches:
  1. Gazebo Classic 11 with crazyflie_trajectory_world
  2. robot_state_publisher with cf1 URDF
  3. Spawns the Crazyflie 2.1 SDF model into Gazebo

Usage:
  ros2 launch crazyflie_gazebo_sim gazebo_cf1_sim.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --- Package paths ---
    cf_gazebo_sim_pkg = get_package_share_directory('crazyflie_gazebo_sim')
    gazebo_ros_pkg     = get_package_share_directory('gazebo_ros')

    # --- File paths ---
    world_file  = os.path.join(cf_gazebo_sim_pkg, 'worlds',
                               'crazyflie_trajectory_world.world')
    xacro_file  = os.path.join(cf_gazebo_sim_pkg, 'urdf',
                               'crazyflie_2_1_gazebo.urdf.xacro')
    model_path  = os.path.join(cf_gazebo_sim_pkg, 'models')

    # --- Launch arguments ---
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='cf1',
                        description='Name of the Crazyflie robot instance')
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                        description='Initial X position (m)')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                        description='Initial Y position (m)')
    z_arg = DeclareLaunchArgument('z', default_value='0.05',
                        description='Initial Z position (m)')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                        description='Launch Gazebo GUI')

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    gui = LaunchConfiguration('gui')

    # --- Set GAZEBO_MODEL_PATH ---
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # --- Generate URDF from xacro ---
    # ParameterValue with value_type=str fixes the yaml parsing error in Humble
    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' robot_name:=', robot_name,
            ' initial_x:=', x,
            ' initial_y:=', y,
            ' initial_z:=', z,
        ]),
        value_type=str
    )

    # --- Node 1: robot_state_publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cf1_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True},
        ],
    )

    # --- Node 2: Gazebo server ---
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'false',
        }.items(),
    )

    # --- Node 3: Gazebo client (GUI) ---
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # --- Node 4: Spawn Crazyflie SDF model into Gazebo ---
    spawn_cf1_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='cf1_spawner',
        output='screen',
        arguments=[
            '-entity',          'cf1',
            '-file',            os.path.join(model_path, 'crazyflie_2_1', 'model.sdf'),
            '-robot_namespace', 'cf1',
            '-x', x, '-y', y, '-z', z,
            '-R', '0', '-P', '0', '-Y', '0',
        ],
    )

    return LaunchDescription([
        set_gazebo_model_path,
        robot_name_arg, x_arg, y_arg, z_arg, gui_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_cf1_node,
    ])

