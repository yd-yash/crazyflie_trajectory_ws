"""
cf1_full_simulation.launch.py
==============================
Single launch file for the complete Crazyflie 2.1 simulation.

Starts:
  1. Gazebo Classic 11 with crazyflie_trajectory_world
  2. Crazyflie 2.1 model (with cf_motor_plugin)
  3. robot_state_publisher (TF tree)
  4. cf_controller_node (cascaded PID / active controller)

Usage:
  ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py

Optional args:
  controller:=cascaded_pid     (default)
  trajectory:=figure8          (default)
  gui:=true                    (default)
  x:=0.0 y:=0.0 z:=0.05       (initial position)

To run headless (no GUI):
  ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Package paths ---
    cf_gazebo_sim_pkg     = get_package_share_directory('crazyflie_gazebo_sim')
    cf_controller_pkg     = get_package_share_directory('cf_trajectory_controller')

    # --- Config file ---
    controller_config = os.path.join(
        cf_controller_pkg, 'config', 'controller_params.yaml'
    )

    # --- Launch arguments ---
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='cascaded_pid',
        description='Controller type: cascaded_pid | conventional_smc | super_twisting_smc | nst_smc'
    )
    trajectory_arg = DeclareLaunchArgument(
        'trajectory', default_value='figure8',
        description='Trajectory type: figure8 | circle | helix | hover'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI (false for headless)'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                   description='Initial X [m]')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                   description='Initial Y [m]')
    z_arg = DeclareLaunchArgument('z', default_value='0.05',
                                   description='Initial Z [m]')

    controller = LaunchConfiguration('controller')
    trajectory  = LaunchConfiguration('trajectory')
    gui         = LaunchConfiguration('gui')
    x           = LaunchConfiguration('x')
    y           = LaunchConfiguration('y')
    z           = LaunchConfiguration('z')

    # =========================================================================
    # Node 1+2+3: Gazebo + model spawn + robot_state_publisher
    # (all handled by gazebo_cf1_sim.launch.py)
    # =========================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_gazebo_sim_pkg, 'launch', 'gazebo_cf1_sim.launch.py')
        ),
        launch_arguments={
            'gui':  gui,
            'x':    x,
            'y':    y,
            'z':    z,
        }.items(),
    )

    # =========================================================================
    # Node 4: Controller node
    # Delayed 5 seconds to allow Gazebo to fully initialize and spawn model
    # =========================================================================
    controller_node = Node(
        package='cf_trajectory_controller',
        executable='cf_controller_node',
        name='cf_controller_node',
        output='screen',
        parameters=[
            controller_config,
            {
                'active_controller': controller,
                'active_trajectory': trajectory,
            }
        ],
    )

    delayed_controller = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[cf1_full_simulation] Starting controller node...'),
            controller_node,
        ]
    )

    return LaunchDescription([
        # Arguments
        controller_arg,
        trajectory_arg,
        gui_arg,
        x_arg, y_arg, z_arg,
        # Nodes
        gazebo_launch,
        delayed_controller,
    ])
