"""
figure8_trajectory.py
======================
Figure-8 (Lemniscate) trajectory generator.

Translated exactly from MATLAB script: PID_3D_05.m
  x_des(t) = A * sin(omega_x * t)         [MATLAB line 13]
  y_des(t) = A * sin(omega_y * t)          [MATLAB line 14]  (omega_y = 2*omega_x)
  z_des(t) = z_const                       [MATLAB line 15]

Default parameters match MATLAB:
  A = 2.0 m, omega_x = 0.4 rad/s, omega_y = 0.8 rad/s, z = 2.0 m

Scaled for Crazyflie 2.1 flight envelope:
  A_cf = 0.5 m (reduced from 2.0m for indoor lab)
  z_cf = 0.5 m (reduced from 2.0m for safety)
  Timescale unchanged.

To use original MATLAB scale: set scale_for_crazyflie=False
"""

import numpy as np


class Figure8Trajectory:
    """
    Figure-8 trajectory for quadrotor tracking experiments.

    Provides position, velocity, and acceleration references
    at any time t — matches the MATLAB anonymous function approach.
    """

    def __init__(self, params: dict):
        """
        Parameters
        ----------
        params : dict
            amplitude   : float — trajectory amplitude [m]      (MATLAB: 2.0)
            omega_x     : float — x angular frequency [rad/s]   (MATLAB: 0.4)
            omega_y     : float — y angular frequency [rad/s]   (MATLAB: 0.8)
            z_const     : float — constant altitude [m]         (MATLAB: 2.0)
            psi_des     : float — desired yaw [rad]             (MATLAB: 0.0)
        """
        self.A       = params.get('amplitude', 0.5)     # scaled from MATLAB 2.0
        self.omega_x = params.get('omega_x',   0.4)     # same as MATLAB
        self.omega_y = params.get('omega_y',   0.8)     # same as MATLAB
        self.z_const = params.get('z_const',   0.5)     # scaled from MATLAB 2.0
        self.psi_des = params.get('psi_des',   0.0)     # same as MATLAB

    def get_reference(self, t: float) -> np.ndarray:
        """
        Get full reference state at time t.

        Parameters
        ----------
        t : float — current time [s]

        Returns
        -------
        reference : np.ndarray, shape (12,)
            [xd, yd, zd, 0, 0, psi_des,
             xd_dot, yd_dot, zd_dot, 0, 0, 0]
        """
        # --- Position --- (MATLAB lines: 13-15)
        # x_des = A * sin(omega_x * t)
        x_des = self.A * np.sin(self.omega_x * t)
        # y_des = A * sin(omega_y * t)
        y_des = self.A * np.sin(self.omega_y * t)
        # z_des = z_const
        z_des = self.z_const

        # --- Velocity --- (MATLAB lines: 18-20)
        # x_dot_des = A * omega_x * cos(omega_x * t)
        x_dot_des = self.A * self.omega_x * np.cos(self.omega_x * t)
        # y_dot_des = A * omega_y * cos(omega_y * t)
        y_dot_des = self.A * self.omega_y * np.cos(self.omega_y * t)
        z_dot_des = 0.0

        # --- Acceleration --- (MATLAB lines: 23-25)
        # x_ddot_des = -A * omega_x^2 * sin(omega_x * t)
        x_ddot_des = -self.A * self.omega_x**2 * np.sin(self.omega_x * t)
        # y_ddot_des = -A * omega_y^2 * sin(omega_y * t)
        y_ddot_des = -self.A * self.omega_y**2 * np.sin(self.omega_y * t)
        z_ddot_des = 0.0

        reference = np.array([
            x_des,     y_des,     z_des,       # positions [0:3]
            0.0,       0.0,       self.psi_des, # euler angles [3:6]
            x_dot_des, y_dot_des, z_dot_des,   # velocities [6:9]
            0.0,       0.0,       0.0           # angular rates [9:12]
        ])

        return reference

    def get_acceleration_reference(self, t: float) -> np.ndarray:
        """
        Get reference accelerations (used for feedforward in some controllers).

        Returns
        -------
        acc_ref : np.ndarray, shape (3,)  [x_ddot, y_ddot, z_ddot]
        """
        x_ddot_des = -self.A * self.omega_x**2 * np.sin(self.omega_x * t)
        y_ddot_des = -self.A * self.omega_y**2 * np.sin(self.omega_y * t)
        return np.array([x_ddot_des, y_ddot_des, 0.0])

