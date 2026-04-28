"""
Conventional Sliding Mode Controller (SMC) for Crazyflie 2.1
Translated from MATLAB smc_controller.m + smc_params.m

Control structure (cascaded):
  Outer loop  : z, x, y  → U1, Ux, Uy
  Middle layer: Ux/Uy + ψ → φ_cmd, θ_cmd
  Inner loop  : φ, θ, ψ   → U2, U3, U4

Sliding surface (per channel):  s = ė + c·e
Reaching law:  u_smc = ks·sat(s, sat_bl) + kl·s

All physical constants are pulled from cf21_parameters.py (CF2.1 values).
Gains are loaded from controller_params.yaml under the 'conventional_smc' key.
"""

import math
import numpy as np
from .base_controller import BaseController


# ── helpers ────────────────────────────────────────────────────────────────

def _wrap(x: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(x), math.cos(x))


def _asin_norm(u: float) -> float:
    """Safe asin that clamps input to [-1, 1]."""
    return math.asin(max(-1.0, min(1.0, u)))


def _sat(s: float, bl: float) -> float:
    """Saturation function. Degenerates to sign(s) when bl <= 0."""
    if bl <= 0.0:
        return math.copysign(1.0, s)
    return max(-1.0, min(1.0, s / bl))


# ── controller ─────────────────────────────────────────────────────────────

class ConventionalSMC(BaseController):
    """
    Conventional SMC — direct port of MATLAB smc_controller.m.

    Gains (set in controller_params.yaml → conventional_smc):
      c_z, ks_z, kl_z        — altitude sliding surface
      c_x, ks_x, kl_x        — x-position sliding surface
      c_y, ks_y, kl_y        — y-position sliding surface
      c_phi, ks_phi, kl_phi  — roll sliding surface
      c_theta, ks_theta, kl_theta — pitch sliding surface
      c_psi, ks_psi, kl_psi  — yaw sliding surface
      sat_bl                  — boundary layer thickness (0 → pure sign / chattering)
    """

    def __init__(self, params: dict, dt: float = 0.01):
        super().__init__(params)

        # ── physical params (CF2.1) ──────────────────────────────────────
        p = params
        self.m    = p['mass']           # 0.034 kg
        self.g    = p['g']              # 9.81
        self.Ixx  = p['Ixx']           # 16.571e-6
        self.Iyy  = p['Iyy']           # 16.655e-6
        self.Izz  = p['Izz']           # 29.261e-6
        self.Ir   = p.get('Ir', 0.0)   # rotor inertia (small, often 0 for CF2.1)
        self.Kx   = p.get('Kx', 0.0)   # translational drag
        self.Ky   = p.get('Ky', 0.0)
        self.Kz   = p.get('Kz', 0.0)
        self.Kphi   = p.get('Kphi',   0.0)  # rotational drag
        self.Ktheta = p.get('Ktheta', 0.0)
        self.Kpsi   = p.get('Kpsi',   0.0)

        # ── control limits (from handoff doc) ────────────────────────────
        self.U1_MAX  = p.get('U1_MAX',  1.3)
        self.U2_MAX  = p.get('U2_MAX',  0.002)
        self.U3_MAX  = p.get('U3_MAX',  0.002)
        self.U4_MAX  = p.get('U4_MAX',  0.001)
        self.UXY_MAX = p.get('UXY_MAX', 0.3)   # sin(~17°) practical limit

        # ── SMC gains (from yaml conventional_smc section) ───────────────
        g = p.get('conventional_smc', {})

        self.c_z  = g.get('c_z',  5.0)
        self.ks_z = g.get('ks_z', 5.0)
        self.kl_z = g.get('kl_z', 3.0)

        self.c_x  = g.get('c_x',  5.0)
        self.ks_x = g.get('ks_x', 5.0)
        self.kl_x = g.get('kl_x', 1.0)

        self.c_y  = g.get('c_y',  5.0)
        self.ks_y = g.get('ks_y', 5.0)
        self.kl_y = g.get('kl_y', 1.0)

        self.c_phi   = g.get('c_phi',   10.0)
        self.ks_phi  = g.get('ks_phi',   3.0)
        self.kl_phi  = g.get('kl_phi',   0.0)

        self.c_theta   = g.get('c_theta',   10.0)
        self.ks_theta  = g.get('ks_theta',   3.0)
        self.kl_theta  = g.get('kl_theta',   0.0)

        self.c_psi   = g.get('c_psi',   5.0)
        self.ks_psi  = g.get('ks_psi',  0.1)
        self.kl_psi  = g.get('kl_psi',  5.0)

        self.sat_bl = g.get('sat_bl', 0.5)   # CF2.1 tuned: tighter than MATLAB (2.0)

        # ── integrator / memory state ────────────────────────────────────
        self.reset()

    # -----------------------------------------------------------------------

    def reset(self):
        """Reset all internal memory (call before each new episode)."""
        self.Ux_prev       = 0.0
        self.Uy_prev       = 0.0
        self.phi_cmd_prev      = 0.0
        self.phi_cmd_dot_prev  = 0.0
        self.theta_cmd_prev      = 0.0
        self.theta_cmd_dot_prev  = 0.0
        self.Omega_r = 0.0   # net rotor speed (gyroscopic coupling); updated externally

    # -----------------------------------------------------------------------

    def compute_control(self,
                        state: np.ndarray,
                        reference: dict,
                        dt: float) -> np.ndarray:
        """
        Parameters
        ----------
        state : np.ndarray (12,)
            [x, y, z, φ, θ, ψ,  ẋ, ẏ, ż,  φ̇, θ̇, ψ̇]
        reference : dict
            Keys: x_d, y_d, z_d, psi_d,
                  x_dot_d, y_dot_d, z_dot_d, psi_dot_d,
                  x_ddot_d, y_ddot_d, z_ddot_d, psi_ddot_d
        dt : float
            Control timestep (seconds)

        Returns
        -------
        U : np.ndarray (4,)
            [U1 (N), U2 (N·m), U3 (N·m), U4 (N·m)]
        """

        # ── unpack state ────────────────────────────────────────────────
        x_pos, y_pos, z     = state[0], state[1], state[2]
        phi, theta, psi     = state[3], state[4], state[5]
        x_dot, y_dot, z_dot = state[6], state[7], state[8]
        phi_dot, theta_dot, psi_dot = state[9], state[10], state[11]

        # ── unpack reference (numpy array from figure8_trajectory.py) ───────
        # [x, y, z, phi_des, theta_des, psi_des, x_dot, y_dot, z_dot, p, q, r]
        Xd        = float(reference[0])
        Yd        = float(reference[1])
        Zd        = float(reference[2])
        psid      = float(reference[5])
        Xd_dot    = float(reference[6])
        Yd_dot    = float(reference[7])
        Zd_dot    = float(reference[8])
        psid_dot  = 0.0
        Xd_ddot   = 0.0
        Yd_ddot   = 0.0
        Zd_ddot   = 0.0
        psid_ddot = 0.0

        # ── guard cos(φ)cos(θ) from zero ───────────────────────────────
        cphi   = math.cos(phi)
        ctheta = math.cos(theta)
        cphi_ctheta = cphi * ctheta
        if abs(cphi_ctheta) < 1e-4:
            cphi_ctheta = math.copysign(1e-4, cphi_ctheta)

        # ══════════════════════════════════════════════════════════════
        # ALTITUDE — U1
        # ══════════════════════════════════════════════════════════════
        e_z     = Zd - z
        e_z_dot = Zd_dot - z_dot
        s_z     = e_z_dot + self.c_z * e_z

        U1 = (self.m / cphi_ctheta) * (
            self.g
            + (self.Kz / self.m) * z_dot
            + Zd_ddot
            + self.c_z * e_z_dot
            + self.ks_z * _sat(s_z, self.sat_bl)
            + self.kl_z * s_z
        )
        U1 = max(0.0, min(self.U1_MAX, U1))

        # ══════════════════════════════════════════════════════════════
        # X / Y VIRTUAL INPUTS → φ_cmd, θ_cmd
        # ══════════════════════════════════════════════════════════════
        # — X —
        e_x     = Xd - x_pos
        e_x_dot = Xd_dot - x_dot
        s_x     = e_x_dot + self.c_x * e_x

        if U1 < 1e-4:
            Ux = 0.0
        else:
            Ux = (self.m / U1) * (
                (self.Kx / self.m) * x_dot
                + Xd_ddot
                + self.c_x * e_x_dot
                + self.ks_x * _sat(s_x, self.sat_bl)
                + self.kl_x * s_x
            )
        Ux = max(-self.UXY_MAX, min(self.UXY_MAX, Ux))

        # — Y —
        e_y     = Yd - y_pos
        e_y_dot = Yd_dot - y_dot
        s_y     = e_y_dot + self.c_y * e_y

        if U1 < 1e-4:
            Uy = 0.0
        else:
            Uy = (self.m / U1) * (
                (self.Ky / self.m) * y_dot
                + Yd_ddot
                + self.c_y * e_y_dot
                + self.ks_y * _sat(s_y, self.sat_bl)
                + self.kl_y * s_y
            )
        Uy = max(-self.UXY_MAX, min(self.UXY_MAX, Uy))

        # ── attitude commands ────────────────────────────────────────
        spsi = math.sin(psid)
        cpsi = math.cos(psid)

        phi_cmd      = _asin_norm(Ux * spsi - Uy * cpsi)
        phi_cmd_dot  = _wrap(phi_cmd - self.phi_cmd_prev) / dt
        phi_cmd_ddot = (phi_cmd_dot - self.phi_cmd_dot_prev) / dt

        cos_phi_cmd = math.cos(phi_cmd)
        if abs(cos_phi_cmd) < 1e-4:
            cos_phi_cmd = math.copysign(1e-4, cos_phi_cmd)

        theta_cmd      = _asin_norm((Ux * cpsi + Uy * spsi) / cos_phi_cmd)
        theta_cmd_dot  = _wrap(theta_cmd - self.theta_cmd_prev) / dt
        theta_cmd_ddot = (theta_cmd_dot - self.theta_cmd_dot_prev) / dt

        # ══════════════════════════════════════════════════════════════
        # ROLL — U2
        # ══════════════════════════════════════════════════════════════
        e_phi     = _wrap(phi_cmd - phi)
        e_phi_dot = _wrap(phi_cmd_dot - phi_dot)
        s_phi     = e_phi_dot + self.c_phi * e_phi

        U2 = self.Ixx * (
            - ((self.Iyy - self.Izz) / self.Ixx) * theta_dot * psi_dot
            + (self.Kphi / self.Ixx) * phi_dot * abs(phi_dot)
            + (self.Ir   / self.Ixx) * self.Omega_r * theta_dot
            + phi_cmd_ddot
            + self.c_phi * e_phi_dot
            + self.ks_phi  * _sat(s_phi, self.sat_bl)
            + self.kl_phi  * s_phi
        )

        # ══════════════════════════════════════════════════════════════
        # PITCH — U3
        # ══════════════════════════════════════════════════════════════
        e_theta     = _wrap(theta_cmd - theta)
        e_theta_dot = _wrap(theta_cmd_dot - theta_dot)
        s_theta     = e_theta_dot + self.c_theta * e_theta

        U3 = self.Iyy * (
            - ((self.Izz - self.Ixx) / self.Iyy) * phi_dot * psi_dot
            + (self.Ktheta / self.Iyy) * theta_dot * abs(theta_dot)
            - (self.Ir     / self.Iyy) * self.Omega_r * phi_dot
            + theta_cmd_ddot
            + self.c_theta * e_theta_dot
            + self.ks_theta * _sat(s_theta, self.sat_bl)
            + self.kl_theta * s_theta
        )

        # ══════════════════════════════════════════════════════════════
        # YAW — U4
        # ══════════════════════════════════════════════════════════════
        e_psi     = _wrap(psid - psi)
        e_psi_dot = _wrap(psid_dot - psi_dot)
        s_psi     = e_psi_dot + self.c_psi * e_psi

        U4 = self.Izz * (
            - ((self.Ixx - self.Iyy) / self.Izz) * phi_dot * theta_dot
            + (self.Kpsi / self.Izz) * psi_dot * abs(psi_dot)
            + psid_ddot
            + self.c_psi * e_psi_dot
            + self.ks_psi  * _sat(s_psi, self.sat_bl)
            + self.kl_psi  * s_psi
        )

        # ── clamp torques ────────────────────────────────────────────
        U2 = max(-self.U2_MAX, min(self.U2_MAX, U2))
        U3 = max(-self.U3_MAX, min(self.U3_MAX, U3))
        U4 = max(-self.U4_MAX, min(self.U4_MAX, U4))

        # ── update memory ────────────────────────────────────────────
        self.Ux_prev           = Ux
        self.Uy_prev           = Uy
        self.phi_cmd_prev      = phi_cmd
        self.phi_cmd_dot_prev  = phi_cmd_dot
        self.theta_cmd_prev      = theta_cmd
        self.theta_cmd_dot_prev  = theta_cmd_dot

        return np.array([U1, U2, U3, U4])
