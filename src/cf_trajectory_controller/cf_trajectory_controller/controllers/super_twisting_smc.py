"""
Super-Twisting Sliding Mode Controller (ST-SMC) for Crazyflie 2.1
==================================================================
Translated from MATLAB: v1_quad_discrete_xy_ux_uy_fault.m
                         quad_discrete_function.m

Theory
------
The Super-Twisting algorithm (Levant, 1993) is a second-order SMC that
eliminates chattering by replacing the discontinuous sign(s) term with a
continuous sqrt(|s|)·sign(s) term plus an integral of sign(s).

For each channel the reaching law is:
    u_sw = k1 · sqrt(|s|) · sign(s)  +  k2 · ∫ sign(s) dt

The integral ∫ sign(s) dt is accumulated in a dedicated state variable
(I_u) that is updated at each timestep:
    I_u += sign(s) · dt

This gives finite-time convergence and continuous control action — key
advantages over conventional SMC for the lightweight CF2.1 airframe.

References
----------
[1] Levant, A. (1993). Sliding order and sliding accuracy in sliding mode
    control. International Journal of Control, 58(6), 1247–1263.
    https://doi.org/10.1080/00207179308923053

[2] Shtessel, Y., Edwards, C., Fridman, L., & Levant, A. (2014).
    Sliding Mode Control and Observation. Birkhäuser.
    Chapter 6 — Super-Twisting Algorithm.

[3] Besnard, L., Shtessel, Y. B., & Landrum, B. (2012). Quadrotor vehicle
    control via sliding mode controller driven by sliding mode disturbance
    observer. Journal of the Franklin Institute, 349(2), 658–684.
    https://doi.org/10.1016/j.jfranklin.2011.06.031

[4] IMRCLab/crazyswarm2 — CF2.1 physical parameters
    https://github.com/IMRCLab/crazyswarm2

Control Structure (cascaded)
-----------------------------
Outer loop  : z          → U1  (altitude,  ST-SMC with integral state I_u1)
XY loop     : x, y       → Ux, Uy (virtual inputs, ST-SMC with I_ux, I_uy)
Middle layer: Ux/Uy + ψd → φ_cmd, θ_cmd  (geometric inversion)
Inner loop  : φ, θ, ψ   → U2, U3, U4  (ST-SMC with I_u2, I_u3, I_u4)

Sliding surfaces
----------------
Altitude : s_z     = kpz·ez  + kdz·ez_dot  + kiz·∫ez dt
X pos    : s_x     = kp_x·ex + kd_x·ex_dot + ki_x·∫ex dt
Y pos    : s_y     = kp_y·ey + kd_y·ey_dot + ki_y·∫ey dt
Roll     : s_phi   = kp·ephi   + kd·ephi_dot   + ki·∫ephi dt
Pitch    : s_theta = kp·etheta + kd·etheta_dot + ki·∫etheta dt
Yaw      : s_psi   = kp·epsi   + kd·epsi_dot   + ki·∫epsi dt
"""

import math
import numpy as np
from .base_controller import BaseController


# ── helpers ────────────────────────────────────────────────────────────────

def _wrap(x: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(x), math.cos(x))


def _asin_norm(u: float) -> float:
    """Safe asin clamped to [-1, 1]."""
    return math.asin(max(-1.0, min(1.0, u)))


def _st_term(s: float, I: float, k1: float, k2: float) -> float:
    """
    Super-twisting switching term:
        u_sw = k1·sqrt(|s|)·sign(s) + k2·I
    where I = ∫ sign(s) dt (updated externally each step).
    """
    return k1 * math.sqrt(abs(s)) * math.copysign(1.0, s) + k2 * I


# ── controller ─────────────────────────────────────────────────────────────

class SuperTwistingSMC(BaseController):
    """
    Super-Twisting SMC — direct port of MATLAB v1_quad_discrete_xy_ux_uy_fault.m.

    Key difference from ConventionalSMC:
      - Switching term is k1·sqrt(|s|)·sign(s) + k2·I  (continuous)
      - Six integral states I_u1..I_u4, I_ux, I_uy accumulated each step
      - Sliding surface includes integral of error (PID-like surface)
      - No boundary layer / saturation function needed — inherently smooth

    Gains (controller_params.yaml → super_twisting_smc):
      kpz, kdz, kiz          — altitude surface coefficients
      kp_x, kd_x, ki_x      — x-position surface coefficients
      kp_y, kd_y, ki_y      — y-position surface coefficients
      kp, kd, ki             — attitude surface coefficients (φ, θ, ψ shared)
      k1, k2                 — ST gains for altitude + attitude channels
      k1_x, k2_x            — ST gains for x-position channel
      k1_y, k2_y            — ST gains for y-position channel
      kpx, kpy, kdx, kdy     — velocity/accel reference scaling gains
    """

    def __init__(self, params: dict, dt: float = 0.01):
        super().__init__(params)
        self.dt = dt

        # ── physical params (CF2.1) ──────────────────────────────────────
        p = params
        self.m   = p['mass']        # 0.034 kg
        self.g   = p['g']           # 9.81
        self.Ixx = p['Ixx']         # 16.571e-6
        self.Iyy = p['Iyy']         # 16.655e-6
        self.Izz = p['Izz']         # 29.261e-6

        # ── control limits ───────────────────────────────────────────────
        self.U1_MAX  = p.get('U1_MAX',  1.3)
        self.U2_MAX  = p.get('U2_MAX',  0.002)
        self.U3_MAX  = p.get('U3_MAX',  0.002)
        self.U4_MAX  = p.get('U4_MAX',  0.001)
        self.UXY_MAX = p.get('UXY_MAX', 0.3)   # practical tilt limit ~17°

        # ── ST-SMC gains (from yaml super_twisting_smc section) ──────────
        g = p.get('super_twisting_smc', {})

        # Altitude surface: s_z = kpz·ez + kdz·ez_dot + kiz·∫ez
        self.kpz = g.get('kpz', 0.02)
        self.kdz = g.get('kdz', 0.05)
        self.kiz = g.get('kiz', 0.00001)

        # XY velocity/accel reference scaling
        self.kpx = g.get('kpx', 0.7)
        self.kpy = g.get('kpy', 0.7)
        self.kdx = g.get('kdx', 0.5)
        self.kdy = g.get('kdy', 0.5)

        # X surface: s_x = kp_x·ex + kd_x·ex_dot + ki_x·∫ex
        self.kp_x = g.get('kp_x', 0.7)
        self.kd_x = g.get('kd_x', 0.5)
        self.ki_x = g.get('ki_x', 0.5)

        # Y surface: s_y = kp_y·ey + kd_y·ey_dot + ki_y·∫ey
        self.kp_y = g.get('kp_y', 0.7)
        self.kd_y = g.get('kd_y', 0.5)
        self.ki_y = g.get('ki_y', 0.5)

        # Attitude surface (shared φ, θ, ψ): s = kp·e + kd·e_dot + ki·∫e
        self.kp = g.get('kp', 0.2)
        self.kd = g.get('kd', 0.05)
        self.ki = g.get('ki', 0.05)

        # Super-twisting gains
        self.k1   = g.get('k1',   0.2)    # altitude + attitude channels
        self.k2   = g.get('k2',   0.1)
        self.k1_x = g.get('k1_x', 0.1)   # x-position channel
        self.k2_x = g.get('k2_x', 0.35)
        self.k1_y = g.get('k1_y', 0.1)   # y-position channel
        self.k2_y = g.get('k2_y', 0.35)

        # ── integral states & memory ─────────────────────────────────────
        self.reset()

    # -----------------------------------------------------------------------

    def reset(self):
        """Reset all integral states and memory (call before each episode)."""
        # Super-twisting integral states  ∫ sign(s) dt
        self.I_u1  = 0.0   # altitude
        self.I_u2  = 0.0   # roll
        self.I_u3  = 0.0   # pitch
        self.I_u4  = 0.0   # yaw
        self.I_ux  = 0.0   # x position
        self.I_uy  = 0.0   # y position

        # Error integral states  ∫ e dt  (for the PID-like sliding surface)
        self.Is_z     = 0.0
        self.Is_phi   = 0.0
        self.Is_theta = 0.0
        self.Is_psi   = 0.0
        self.Is_x     = 0.0
        self.Is_y     = 0.0

        # Attitude command memory (for finite-difference derivatives)
        self.phi_cmd_prev      = 0.0
        self.phi_cmd_dot_prev  = 0.0
        self.theta_cmd_prev      = 0.0
        self.theta_cmd_dot_prev  = 0.0

        # Previous xd_dot, yd_dot for accel reference
        self.xd_dot_prev = 0.0
        self.yd_dot_prev = 0.0

    # -----------------------------------------------------------------------
    def compute_control(self,
                        state: np.ndarray,
                        reference: np.ndarray,
                        dt: float) -> np.ndarray:

        # ── unpack state ────────────────────────────────────────────────
        x_pos, y_pos, z     = state[0], state[1], state[2]
        phi, theta, psi     = state[3], state[4], state[5]
        x_dot, y_dot, z_dot = state[6], state[7], state[8]
        phi_dot, theta_dot, psi_dot = state[9], state[10], state[11]

        # ── unpack reference ────────────────────────────────────────────
        xd       = float(reference[0])
        yd       = float(reference[1])
        zd       = float(reference[2])
        psid     = float(reference[5])
        xd_dot   = float(reference[6])
        yd_dot   = float(reference[7])
        zd_dot   = float(reference[8])

        # ── guard cos(φ)cos(θ) ──────────────────────────────────────────
        cphi_ctheta = math.cos(phi) * math.cos(theta)
        if abs(cphi_ctheta) < 1e-4:
            cphi_ctheta = math.copysign(1e-4, cphi_ctheta)

        # ══════════════════════════════════════════════════════════════
        # ALTITUDE — U1
        # Surface: s_z = ez_dot + c_z*ez   (same as conv. SMC)
        # ST reaching: k1*sqrt(|s|)*sign(s) + k2*I_u1
        # ══════════════════════════════════════════════════════════════
        ez     = zd - z
        ez_dot = zd_dot - z_dot
        s_z    = ez_dot + self.kpz * ez

        self.I_u1 += math.copysign(1.0, s_z) * dt
        # Clamp integral to prevent windup
        self.I_u1 = max(-50.0, min(50.0, self.I_u1))

        U1 = (self.m / cphi_ctheta) * (
            self.g
            + self.kpz * ez_dot
            + _st_term(s_z, self.I_u1, self.k1, self.k2)
        )
        U1 = max(0.0, min(self.U1_MAX, U1))

        # ══════════════════════════════════════════════════════════════
        # X VIRTUAL INPUT
        # Surface: s_x = ex_dot + c_x*ex   (same structure as conv. SMC)
        # ST reaching replaces sat()
        # ══════════════════════════════════════════════════════════════
        ex     = xd - x_pos
        ex_dot = xd_dot - x_dot
        s_x    = ex_dot + self.kp_x * ex

        self.I_ux += math.copysign(1.0, s_x) * dt
        self.I_ux = max(-50.0, min(50.0, self.I_ux))

        if U1 < 1e-4:
            ux = 0.0
        else:
            ux = (self.m / U1) * (
                self.kp_x * ex_dot
                + _st_term(s_x, self.I_ux, self.k1_x, self.k2_x)
            )
        ux = max(-self.UXY_MAX, min(self.UXY_MAX, ux))

        # ══════════════════════════════════════════════════════════════
        # Y VIRTUAL INPUT
        # ══════════════════════════════════════════════════════════════
        ey     = yd - y_pos
        ey_dot = yd_dot - y_dot
        s_y    = ey_dot + self.kp_y * ey

        self.I_uy += math.copysign(1.0, s_y) * dt
        self.I_uy = max(-50.0, min(50.0, self.I_uy))

        if U1 < 1e-4:
            uy = 0.0
        else:
            uy = (self.m / U1) * (
                self.kp_y * ey_dot
                + _st_term(s_y, self.I_uy, self.k1_y, self.k2_y)
            )
        uy = max(-self.UXY_MAX, min(self.UXY_MAX, uy))

        # ── attitude commands ────────────────────────────────────────
        spsi = math.sin(psid)
        cpsi = math.cos(psid)

        phi_cmd      = _asin_norm(ux * spsi - uy * cpsi)
        phi_cmd_dot  = _wrap(phi_cmd - self.phi_cmd_prev) / dt
        phi_cmd_ddot = (phi_cmd_dot - self.phi_cmd_dot_prev) / dt

        cos_phi_cmd = math.cos(phi_cmd)
        if abs(cos_phi_cmd) < 1e-4:
            cos_phi_cmd = math.copysign(1e-4, cos_phi_cmd)

        theta_cmd      = _asin_norm((ux * cpsi + uy * spsi) / cos_phi_cmd)
        theta_cmd_dot  = _wrap(theta_cmd - self.theta_cmd_prev) / dt
        theta_cmd_ddot = (theta_cmd_dot - self.theta_cmd_dot_prev) / dt

        # ══════════════════════════════════════════════════════════════
        # ROLL — U2
        # Surface: s_phi = ephi_dot + c_phi*ephi
        # ══════════════════════════════════════════════════════════════
        ephi     = _wrap(phi_cmd - phi)
        ephi_dot = _wrap(phi_cmd_dot - phi_dot)
        s_phi    = ephi_dot + self.kp * ephi

        self.I_u2 += math.copysign(1.0, s_phi) * dt
        self.I_u2 = max(-50.0, min(50.0, self.I_u2))

        U2 = self.Ixx * (
            - ((self.Iyy - self.Izz) / self.Ixx) * theta_dot * psi_dot
            + phi_cmd_ddot
            + self.kp * ephi_dot
            + _st_term(s_phi, self.I_u2, self.k1, self.k2)
        )

        # ══════════════════════════════════════════════════════════════
        # PITCH — U3
        # ══════════════════════════════════════════════════════════════
        etheta     = _wrap(theta_cmd - theta)
        etheta_dot = _wrap(theta_cmd_dot - theta_dot)
        s_theta    = etheta_dot + self.kp * etheta

        self.I_u3 += math.copysign(1.0, s_theta) * dt
        self.I_u3 = max(-50.0, min(50.0, self.I_u3))

        U3 = self.Iyy * (
            - ((self.Izz - self.Ixx) / self.Iyy) * phi_dot * psi_dot
            + theta_cmd_ddot
            + self.kp * etheta_dot
            + _st_term(s_theta, self.I_u3, self.k1, self.k2)
        )

        # ══════════════════════════════════════════════════════════════
        # YAW — U4
        # ══════════════════════════════════════════════════════════════
        epsi     = _wrap(psid - psi)
        epsi_dot = _wrap(0.0 - psi_dot)
        s_psi    = epsi_dot + self.kp * epsi

        self.I_u4 += math.copysign(1.0, s_psi) * dt
        self.I_u4 = max(-50.0, min(50.0, self.I_u4))

        U4 = self.Izz * (
            - ((self.Ixx - self.Iyy) / self.Izz) * phi_dot * theta_dot
            + self.kp * epsi_dot
            + _st_term(s_psi, self.I_u4, self.k1, self.k2)
        )

        # ── clamp torques ────────────────────────────────────────────
        U2 = max(-self.U2_MAX, min(self.U2_MAX, U2))
        U3 = max(-self.U3_MAX, min(self.U3_MAX, U3))
        U4 = max(-self.U4_MAX, min(self.U4_MAX, U4))

        # ── update memory ────────────────────────────────────────────
        self.phi_cmd_prev       = phi_cmd
        self.phi_cmd_dot_prev   = phi_cmd_dot
        self.theta_cmd_prev     = theta_cmd
        self.theta_cmd_dot_prev = theta_cmd_dot

        return np.array([U1, U2, U3, U4])