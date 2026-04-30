"""
cf_controller_node.py
======================
Main ROS2 control node for Crazyflie 2.1 trajectory tracking.

This node is controller-agnostic — it loads whichever controller
is specified in controller_params.yaml via active_controller parameter.

To switch controllers: change active_controller in YAML — no code changes.

Subscriptions:
  /cf1/ground_truth/odom  (nav_msgs/Odometry) — state feedback from Gazebo

Publications:
  /cf1/cmd_full_state     (crazyflie_interfaces/FullState) — to Gazebo/hardware
  /cf1/trajectory_ref     (nav_msgs/Odometry)              — for visualization
  /cf1/control_debug      (std_msgs/Float64MultiArray)     — u1,u2,u3,u4 logging

Control loop: Timer at control_frequency Hz
"""

from fastapi import params
import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from crazyflie_interfaces.msg import FullState

# --- Core modules ---
from cf_trajectory_controller.core.cf21_parameters import (
    GRAVITY, MASS, CONTROL_FREQUENCY
)

# --- Controller imports ---
from cf_trajectory_controller.controllers.cascaded_pid import CascadedPID
from cf_trajectory_controller.controllers.conventional_smc import ConventionalSMC
from cf_trajectory_controller.controllers.super_twisting_smc import SuperTwistingSMC
from cf_trajectory_controller.controllers.nstt_smc import NSTTSlidingModeController

# --- Trajectory imports ---
from cf_trajectory_controller.trajectories.figure8_trajectory import Figure8Trajectory


# Registry — add new controllers here when implemented
CONTROLLER_REGISTRY = {
    'cascaded_pid': CascadedPID,
    'conventional_smc': ConventionalSMC,
    'super_twisting_smc': SuperTwistingSMC,
    'nstt_smc': NSTTSlidingModeController,
}

# Registry — add new trajectories here
TRAJECTORY_REGISTRY = {
    'figure8': Figure8Trajectory,
}


class CrazyflieControllerNode(Node):
    """
    ROS2 node that runs the trajectory tracking control loop.
    Plug-and-play: swap controllers via YAML parameter.
    """

    def __init__(self):
        super().__init__('cf_controller_node')

        # --- Declare ROS2 parameters ---
        self.declare_parameter('active_controller', 'cascaded_pid')
        self.declare_parameter('active_trajectory', 'figure8')
        self.declare_parameter('control_frequency', CONTROL_FREQUENCY)

        # --- Read parameters ---
        controller_name = self.get_parameter('active_controller').value
        trajectory_name = self.get_parameter('active_trajectory').value
        ctrl_freq       = self.get_parameter('control_frequency').value
        self.dt = 1.0 / ctrl_freq

        self.get_logger().info(
            f'Controller : {controller_name}\n'
            f'Trajectory : {trajectory_name}\n'
            f'Frequency  : {ctrl_freq} Hz  (dt = {self.dt:.4f} s)'
        )

        # --- Load controller ---
        controller_params = self._load_controller_params(controller_name)
        if controller_name not in CONTROLLER_REGISTRY:
            self.get_logger().error(
                f'Unknown controller: {controller_name}. '
                f'Available: {list(CONTROLLER_REGISTRY.keys())}')
            raise ValueError(f'Unknown controller: {controller_name}')

        self.controller = CONTROLLER_REGISTRY[controller_name](controller_params, self.dt)
        self.get_logger().info(
            f'Loaded controller: {self.controller.get_controller_name()}')

        # --- Load trajectory ---
        trajectory_params = self._load_trajectory_params(trajectory_name)
        if trajectory_name not in TRAJECTORY_REGISTRY:
            self.get_logger().error(f'Unknown trajectory: {trajectory_name}')
            raise ValueError(f'Unknown trajectory: {trajectory_name}')

        self.trajectory = TRAJECTORY_REGISTRY[trajectory_name](trajectory_params)
        self.get_logger().info(f'Loaded trajectory: {trajectory_name}')

        # --- State storage ---
        self.current_state = np.zeros(12)
        self.state_received = False
        self.sim_time = 0.0

        # --- ROS2 Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry,
            '/cf1/ground_truth/odom',
            self.odom_callback,
            10
        )

        # --- ROS2 Publishers ---
        self.cmd_pub = self.create_publisher(
            FullState,
            '/cf1/cmd_full_state',
            10
        )

        self.ref_pub = self.create_publisher(
            Odometry,
            '/cf1/trajectory_ref',
            10
        )

        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            '/cf1/control_debug',
            10
        )

        # --- Control timer ---
        self.control_timer = self.create_timer(
            self.dt,
            self.control_loop_callback
        )

        self.get_logger().info(
            'CrazyflieControllerNode initialized. Waiting for state...')

    # =========================================================================
    # PARAMETER LOADING
    # =========================================================================

    # def _load_controller_params(self, controller_name: str) -> dict:
    #     """
    #     Load controller gains from ROS2 parameters.
    #     Parameters are namespaced under controller_name in the YAML file.
    #     Falls back to defaults in the controller class if not set.
    #     """
    #     params = {}
    #     gain_names = [
    #         'Kp_x',     'Ki_x',     'Kd_x',
    #         'Kp_y',     'Ki_y',     'Kd_y',
    #         'Kp_z',     'Ki_z',     'Kd_z',
    #         'Kp_phi',   'Ki_phi',   'Kd_phi',
    #         'Kp_theta', 'Ki_theta', 'Kd_theta',
    #         'Kp_psi',   'Ki_psi',   'Kd_psi',
    #     ]
    #     for gain in gain_names:
    #         full_name = f'{controller_name}.{gain}'
    #         self.declare_parameter(full_name, 0.0)
    #         val = self.get_parameter(full_name).value
    #         if val != 0.0:
    #             params[gain] = val
    #     self.get_logger().info(
    #         f'Loaded {len(params)} gains for {controller_name} from params')
    #     return params

    def _load_controller_params(self, controller_name: str) -> dict:
        from cf_trajectory_controller.core.cf21_parameters import (
            MASS, GRAVITY, IXX, IYY, IZZ,       # ← uppercase, matches cf21_parameters.py
            U1_MAX, U2_MAX, U3_MAX, U4_MAX,
        )

        params = {
            'mass': MASS,
            'g':    GRAVITY,
            'Ixx':  IXX,        # SMC controller uses lowercase keys internally
            'Iyy':  IYY,
            'Izz':  IZZ,
            'U1_MAX': U1_MAX,
            'U2_MAX': U2_MAX,
            'U3_MAX': U3_MAX,
            'U4_MAX': U4_MAX,
        }

        PID_GAINS = [
            'Kp_x',     'Ki_x',     'Kd_x',
            'Kp_y',     'Ki_y',     'Kd_y',
            'Kp_z',     'Ki_z',     'Kd_z',
            'Kp_phi',   'Ki_phi',   'Kd_phi',
            'Kp_theta', 'Ki_theta', 'Kd_theta',
            'Kp_psi',   'Ki_psi',   'Kd_psi',
        ]

        SMC_GAINS = [
            'c_z',  'ks_z',  'kl_z',
            'c_x',  'ks_x',  'kl_x',
            'c_y',  'ks_y',  'kl_y',
            'c_phi',   'ks_phi',   'kl_phi',
            'c_theta', 'ks_theta', 'kl_theta',
            'c_psi',   'ks_psi',   'kl_psi',
            'sat_bl',
        ]

        ST_SMC_GAINS = [
            'kpz', 'kdz', 'kiz',
            'kpx', 'kpy', 'kdx', 'kdy',
            'kp_x', 'kd_x', 'ki_x',
            'kp_y', 'kd_y', 'ki_y',
            'kp', 'kd', 'ki',
            'k1', 'k2',
            'k1_x', 'k2_x',
            'k1_y', 'k2_y',
        ]

        NSTT_SMC_GAINS = [
            'p_exp', 'q_exp',
            'beta_z', 'beta_phi', 'beta_theta', 'beta_psi', 'beta_x', 'beta_y',
            'mu_min',
            'alpha_f_phi', 'alpha_f_theta',
            'kpx', 'kpy', 'kdx', 'kdy',
            'k1_z',  'k2_z',
            'k1_att','k2_att',
            'k1_psi','k2_psi',
            'k1_x',  'k2_x',
            'k1_y',  'k2_y',
        ]

        GAIN_MAP = {
            'cascaded_pid':       PID_GAINS,
            'conventional_smc':   SMC_GAINS,
            'super_twisting_smc': ST_SMC_GAINS,   # ← add this line
            'nstt_smc':           NSTT_SMC_GAINS,   # ← add this line
        }


        gain_names = GAIN_MAP.get(controller_name, [])
        smc_subdict = {}

        for gain in gain_names:
            full_name = f'{controller_name}.{gain}'
            self.declare_parameter(full_name, -9999.0)
            val = self.get_parameter(full_name).value
            if val != -9999.0:
                if controller_name == 'cascaded_pid':
                    params[gain] = val
                else:
                    smc_subdict[gain] = val

        # if controller_name == 'conventional_smc' and smc_subdict:
        #     params['conventional_smc'] = smc_subdict
                if controller_name in ('conventional_smc', 'super_twisting_smc', 'nstt_smc') and smc_subdict:
                    params[controller_name] = smc_subdict
    
        self.get_logger().info(
            f'Loaded {len(gain_names)} gains for {controller_name} from params')
        return params
        

    def _load_trajectory_params(self, trajectory_name: str) -> dict:
        """
        Load trajectory parameters from ROS2 parameters.
        Parameters are namespaced under trajectory_name in the YAML file.
        Falls back to defaults in the trajectory class if not set.
        """
        params = {}
        traj_param_names = [
            'amplitude', 'omega_x', 'omega_y', 'z_const', 'psi_des'
        ]
        for name in traj_param_names:
            full_name = f'{trajectory_name}.{name}'
            self.declare_parameter(full_name, -999.0)
            val = self.get_parameter(full_name).value
            if val != -999.0:
                params[name] = val
        self.get_logger().info(
            f'Loaded {len(params)} params for {trajectory_name} trajectory')
        return params

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def odom_callback(self, msg: Odometry):
        """
        Convert ROS2 Odometry message to our 12-state numpy vector.

        State layout matches MATLAB:
        [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot,
         phi_dot, theta_dot, psi_dot]
        """
        # Position
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        self.current_state[2] = msg.pose.pose.position.z

        # Orientation: quaternion → euler (ZYX = yaw, pitch, roll)
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('ZYX')     # [yaw, pitch, roll]
        self.current_state[3] = euler[2]    # phi   (roll)
        self.current_state[4] = euler[1]    # theta (pitch)
        self.current_state[5] = euler[0]    # psi   (yaw)

        # Linear velocity (world frame)
        self.current_state[6] = msg.twist.twist.linear.x
        self.current_state[7] = msg.twist.twist.linear.y
        self.current_state[8] = msg.twist.twist.linear.z

        # Angular velocity (body frame)
        self.current_state[9]  = msg.twist.twist.angular.x    # phi_dot
        self.current_state[10] = msg.twist.twist.angular.y    # theta_dot
        self.current_state[11] = msg.twist.twist.angular.z    # psi_dot

        # Replace NaN values with 0 (occurs when drone is static on ground)
        self.current_state = np.nan_to_num(self.current_state, nan=0.0)
        # Replace NaN values with 0 (occurs when drone is static on ground)
        self.current_state = np.nan_to_num(self.current_state, nan=0.0)
        self.state_received = True

        # Track simulation time from message header
        self.sim_time = (msg.header.stamp.sec
                         + msg.header.stamp.nanosec * 1e-9)

    def control_loop_callback(self):
        """
        Main control loop — runs at control_frequency Hz.
        1. Get trajectory reference at current time
        2. Compute control from active controller
        3. Publish command
        4. Publish debug info
        """
        if not self.state_received:
            return

        # --- Step 1: Get reference state at current time ---
        reference = self.trajectory.get_reference(self.sim_time)

        # --- Step 2: Compute control ---
        u = self.controller.compute_control(
            self.current_state,
            reference,
            self.dt
        )

        # --- Step 3: Publish FullState command ---
        self._publish_full_state_cmd(reference, u)

        # --- Step 4: Publish trajectory reference for visualization ---
        self._publish_trajectory_ref(reference)

        # --- Step 5: Publish debug info ---
        self._publish_debug(u, reference)

    # =========================================================================
    # PUBLISHERS
    # =========================================================================

    def _publish_full_state_cmd(self,
                                 reference: np.ndarray,
                                 u: np.ndarray):
        """Publish FullState command to Gazebo/hardware."""
        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # Desired position
        msg.pose.position.x = float(reference[0])
        msg.pose.position.y = float(reference[1])
        msg.pose.position.z = float(reference[2])

        # Desired velocity
        msg.twist.linear.x = float(reference[6])
        msg.twist.linear.y = float(reference[7])
        msg.twist.linear.z = float(reference[8])

        # Desired acceleration derived from thrust
        msg.acc.x = 0.0
        msg.acc.y = 0.0
        msg.acc.z = float(u[0] / MASS - GRAVITY)

        # Yaw
        # yaw encoded in quaternion above

        self.cmd_pub.publish(msg)

    def _publish_trajectory_ref(self, reference: np.ndarray):
        """Publish reference trajectory as Odometry for RViz visualization."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.pose.position.x = float(reference[0])
        msg.pose.pose.position.y = float(reference[1])
        msg.pose.pose.position.z = float(reference[2])
        self.ref_pub.publish(msg)

    def _publish_debug(self,
                        u: np.ndarray,
                        reference: np.ndarray):
        """
        Publish control inputs and tracking errors for logging/plotting.
        Array layout: [u1, u2, u3, u4, ex, ey, ez, sim_time]
        """
        state = self.current_state
        msg = Float64MultiArray()
        msg.data = [
            float(u[0]),                          # u1 thrust      [N]
            float(u[1]),                          # u2 roll torque  [N.m]
            float(u[2]),                          # u3 pitch torque [N.m]
            float(u[3]),                          # u4 yaw torque   [N.m]
            float(reference[0] - state[0]),       # ex position error x
            float(reference[1] - state[1]),       # ey position error y
            float(reference[2] - state[2]),       # ez position error z
            float(self.sim_time),                 # simulation time [s]
        ]
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
