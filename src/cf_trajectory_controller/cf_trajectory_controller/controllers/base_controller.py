"""
base_controller.py
==================
Abstract base class that every controller must implement.

To add a new controller:
  1. Create a new file in controllers/ (e.g. conventional_smc.py)
  2. Inherit from BaseController
  3. Implement compute_control()
  4. That's it — the ROS2 node will load it automatically

Interface contract:
  - Input:  state (12,), reference (12,), dt (float)
  - Output: u (4,) = [u1_thrust, u2_roll_torque, u3_pitch_torque, u4_yaw_torque]
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseController(ABC):
    """
    Abstract base class for all quadrotor trajectory controllers.

    Every controller (PID, SMC, etc.) must inherit this class and
    implement the compute_control() method.
    """

    def __init__(self, params: dict):
        """
        Parameters
        ----------
        params : dict
            Controller-specific gains and settings loaded from YAML config.
        """
        self.params = params
        self._is_initialized = False

    @abstractmethod
    def compute_control(self,
                        state: np.ndarray,
                        reference: np.ndarray,
                        dt: float) -> np.ndarray:
        """
        Compute control inputs given current state and reference.

        Parameters
        ----------
        state : np.ndarray, shape (12,)
            Current state:
            [x, y, z, phi, theta, psi,
             x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]

        reference : np.ndarray, shape (12,)
            Desired state (same layout as state vector).
            For trajectory tracking: [xd, yd, zd, 0, 0, psid,
                                      xd_dot, yd_dot, zd_dot, 0, 0, 0]

        dt : float
            Timestep [seconds]

        Returns
        -------
        u : np.ndarray, shape (4,)
            Control inputs:
            u[0] = u1 : total thrust     [N]
            u[1] = u2 : roll torque      [N.m]
            u[2] = u3 : pitch torque     [N.m]
            u[3] = u4 : yaw torque       [N.m]
        """
        pass

    @abstractmethod
    def reset(self):
        """
        Reset all internal controller states (integrators, filters, etc.).
        Called at the start of each experiment.
        """
        pass

    def get_controller_name(self) -> str:
        """Return the name of this controller for logging."""
        return self.__class__.__name__
