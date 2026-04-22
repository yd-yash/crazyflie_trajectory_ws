"""
cascaded_pid.py
================
Cascaded PID controller for Crazyflie 2.1 trajectory tracking.

Translated exactly from MATLAB script: PID_3D_05.m
Controller structure mirrors quadrotor3D_dynamics() function.

Outer loop: Position → Desired attitude (roll, pitch)
  x_ddot_cmd = Kp_x*ex + Kd_x*ex_dot + Ki_x*int_ex   [MATLAB line 71]
  y_ddot_cmd = Kp_y*ey + Kd_y*ey_dot + Ki_y*int_ey   [MATLAB line 72]
  z_ddot_cmd = Kp_z*ez + Kd_z*ez_dot + Ki_z*int_ez   [MATLAB line 73]

  phi_des   = -(1/g) * y_ddot_cmd                     [MATLAB line 76]
  theta_des =  (1/g) * x_ddot_cmd                     [MATLAB line 77]

Thrust:
  u1 = m * (g + z_ddot_cmd)                           [MATLAB line 86]

Inner loop: Attitude → Torques
  u2 = Kp_phi*(phi_des-phi) + Kd_phi*(0-phi_dot) + Ki_phi*int_ephi
  u3 = Kp_theta*(theta_des-theta) + Kd_theta*(0-theta_dot) + Ki_theta*int_etheta
  u4 = Kp_psi*(psi_des-psi) + Kd_psi*(0-psi_dot) + Ki_psi*int_epsi

Retuning rationale (MATLAB m=0.5kg → CF2.1 m=0.034kg):
  - Outer loop: similar Kp/Kd, reduced Ki (less integral windup)
  - Inner loop: much lower Kp (small inertia, sensitive to high gains)
  - Torque limits increased to physics-based values
  - Control rate reduced to 100Hz for stability
"""

import numpy as np
from cf_trajectory_controller.controllers.base_controller import BaseController
from cf_trajectory_controller.core.cf21_parameters import (
    MASS, GRAVITY,
    PHI_MAX, THETA_MAX, PSI_MAX,
    U1_MAX, U1_MIN,
    U2_MAX, U2_MIN,
    U3_MAX, U3_MIN,
    U4_MAX, U4_MIN
)


class CascadedPID(BaseController):
    """
    Cascaded PID controller.
    Outer loop: position → desired attitude
    Inner loop: attitude → torques
    """

    def __init__(self, params: dict):
        super().__init__(params)

        # --- Outer loop gains (position) ---
        # Tuned for CF2.1 at 100Hz control rate
        self.Kp_x = params.get('Kp_x', 4.0)
        self.Ki_x = params.get('Ki_x', 0.1)
        self.Kd_x = params.get('Kd_x', 4.0)

        self.Kp_y = params.get('Kp_y', 4.0)
        self.Ki_y = params.get('Ki_y', 0.1)
        self.Kd_y = params.get('Kd_y', 4.0)

        self.Kp_z = params.get('Kp_z', 8.0)
        self.Ki_z = params.get('Ki_z', 0.3)
        self.Kd_z = params.get('Kd_z', 2.0)

        # --- Inner loop gains (attitude) ---
        # Key change: much lower than MATLAB (small inertia = sensitive system)
        self.Kp_phi   = params.get('Kp_phi',   0.004)
        self.Ki_phi   = params.get('Ki_phi',    0.0001)
        self.Kd_phi   = params.get('Kd_phi',    0.0006)

        self.Kp_theta = params.get('Kp_theta',  0.004)
        self.Ki_theta = params.get('Ki_theta',   0.0001)
        self.Kd_theta = params.get('Kd_theta',   0.0006)

        self.Kp_psi   = params.get('Kp_psi',    0.002)
        self.Ki_psi   = params.get('Ki_psi',    0.00005)
        self.Kd_psi   = params.get('Kd_psi',    0.0005)

        # --- Anti-windup limits for integrators ---
        self.int_limit_pos = 0.5    # [m]   max position integral
        self.int_limit_att = 0.1    # [rad] max attitude integral

        self.reset()

    def reset(self):
        """Reset all integral states to zero."""
        self.int_ex = 0.0
        self.int_ey = 0.0
        self.int_ez = 0.0
        self.int_ephi   = 0.0
        self.int_etheta = 0.0
        self.int_epsi   = 0.0
        self._is_initialized = True

    def compute_control(self,
                        state: np.ndarray,
                        reference: np.ndarray,
                        dt: float) -> np.ndarray:
        """
        Compute cascaded PID control inputs.

        Parameters
        ----------
        state : np.ndarray, shape (12,)
            [x, y, z, phi, theta, psi,
             x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]
        reference : np.ndarray, shape (12,)
            [xd, yd, zd, 0, 0, psi_des,
             xd_dot, yd_dot, zd_dot, 0, 0, 0]
        dt : float — timestep [s]

        Returns
        -------
        u : np.ndarray, shape (4,)
            [u1_thrust, u2_roll_torque, u3_pitch_torque, u4_yaw_torque]
        """

        # =====================================================================
        # UNPACK STATE
        # =====================================================================
        phi       = state[3]
        theta     = state[4]
        psi       = state[5]
        x_dot     = state[6]
        y_dot     = state[7]
        z_dot     = state[8]
        phi_dot   = state[9]
        theta_dot = state[10]
        psi_dot   = state[11]

        # =====================================================================
        # UNPACK REFERENCE
        # =====================================================================
        x_des     = reference[0]
        y_des     = reference[1]
        z_des     = reference[2]
        psi_des   = reference[5]
        x_dot_des = reference[6]
        y_dot_des = reference[7]
        z_dot_des = reference[8]

        # =====================================================================
        # POSITION ERRORS (MATLAB lines 54-59)
        # =====================================================================
        e_x     = x_des - state[0]
        e_x_dot = x_dot_des - x_dot
        e_y     = y_des - state[1]
        e_y_dot = y_dot_des - y_dot
        e_z     = z_des - state[2]
        e_z_dot = z_dot_des - z_dot

        # =====================================================================
        # INTEGRAL ERRORS with anti-windup clamping
        # =====================================================================
        self.int_ex = np.clip(self.int_ex + e_x * dt,
                              -self.int_limit_pos, self.int_limit_pos)
        self.int_ey = np.clip(self.int_ey + e_y * dt,
                              -self.int_limit_pos, self.int_limit_pos)
        self.int_ez = np.clip(self.int_ez + e_z * dt,
                              -self.int_limit_pos, self.int_limit_pos)

        # =====================================================================
        # OUTER LOOP — Position PID → Commanded accelerations
        # (MATLAB lines 71-73)
        # =====================================================================
        x_ddot_cmd = (self.Kp_x * e_x
                      + self.Kd_x * e_x_dot
                      + self.Ki_x * self.int_ex)

        y_ddot_cmd = (self.Kp_y * e_y
                      + self.Kd_y * e_y_dot
                      + self.Ki_y * self.int_ey)

        z_ddot_cmd = (self.Kp_z * e_z
                      + self.Kd_z * e_z_dot
                      + self.Ki_z * self.int_ez)

        # =====================================================================
        # DESIRED ROLL AND PITCH (MATLAB lines 76-77)
        # phi_des   = -(1/g) * y_ddot_cmd
        # theta_des =  (1/g) * x_ddot_cmd
        # =====================================================================
        phi_des   = -(1.0 / GRAVITY) * y_ddot_cmd
        theta_des =  (1.0 / GRAVITY) * x_ddot_cmd

        phi_des   = np.clip(phi_des,   -PHI_MAX,   PHI_MAX)
        theta_des = np.clip(theta_des, -THETA_MAX, THETA_MAX)
        psi_des   = np.clip(psi_des,   -PSI_MAX,   PSI_MAX)

        # =====================================================================
        # ATTITUDE ERRORS
        # =====================================================================
        e_phi   = phi_des   - phi
        e_theta = theta_des - theta
        e_psi   = psi_des   - psi

        # Integral with anti-windup
        self.int_ephi = np.clip(self.int_ephi + e_phi * dt,
                                -self.int_limit_att, self.int_limit_att)
        self.int_etheta = np.clip(self.int_etheta + e_theta * dt,
                                  -self.int_limit_att, self.int_limit_att)
        self.int_epsi = np.clip(self.int_epsi + e_psi * dt,
                                -self.int_limit_att, self.int_limit_att)

        # =====================================================================
        # CONTROL INPUTS
        # =====================================================================

        # Thrust (MATLAB line 86)
        u1 = MASS * (GRAVITY + z_ddot_cmd)

        # Roll torque (MATLAB line 90)
        u2 = (self.Kp_phi   * e_phi
              + self.Kd_phi   * (0.0 - phi_dot)
              + self.Ki_phi   * self.int_ephi)

        # Pitch torque (MATLAB line 91)
        u3 = (self.Kp_theta * e_theta
              + self.Kd_theta * (0.0 - theta_dot)
              + self.Ki_theta * self.int_etheta)

        # Yaw torque (MATLAB line 92)
        u4 = (self.Kp_psi   * e_psi
              + self.Kd_psi   * (0.0 - psi_dot)
              + self.Ki_psi   * self.int_epsi)

        # =====================================================================
        # SATURATION
        # =====================================================================
        u1 = np.clip(u1, U1_MIN, U1_MAX)
        u2 = np.clip(u2, U2_MIN, U2_MAX)
        u3 = np.clip(u3, U3_MIN, U3_MAX)
        u4 = np.clip(u4, U4_MIN, U4_MAX)

        return np.array([u1, u2, u3, u4])
