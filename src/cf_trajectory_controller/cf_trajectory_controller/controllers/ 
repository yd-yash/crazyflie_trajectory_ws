"""
Nonsingular Super-Twisting Terminal Sliding Mode Controller (NSTT-SMC)
for Crazyflie 2.1
======================================================================
Translated from MATLAB: v9_quad_discrete_xy_ux_uy_fault.m

Theory
------
Terminal SMC (TSMC) uses a nonlinear sliding surface that guarantees
FINITE-TIME convergence (unlike linear surfaces which are asymptotic).

The nonsingular terminal sliding surface for each channel is:

    s = e  +  (1 / beta^gamma) * |ė|^gamma * sign(ė)

where gamma = p/q > 1  (here p=5, q=3 → gamma=5/3)

The "nonsingular" modification replaces the original TSMC term
ė^(2-gamma) (which becomes singular when ė=0 and gamma>1) with:

    |ė|^(2-gamma) * sign(ė)      ← always well-defined

The reaching law uses the Super-Twisting algorithm:

    u_sw = k1 * sqrt(|s|) * sign(s)  +  k2 * ∫ sign(s) dt

Combined control law (altitude example):

    U1 = (m / cos(φ)cos(θ)) * [
           z̈d + g
           + beta_z^gamma * (1/gamma) * |ėz|^(2-gamma) * sign(ėz)
           + k1_z * sqrt(|s_z|) * sign(s_z)
           + k2_z * I_u1
         ]

Key properties vs conventional SMC and ST-SMC:
  - Finite-time convergence (vs asymptotic for linear surface SMC)
  - No singularity (vs standard TSMC where ė^(2-gamma) → ∞ at ė=0)
  - Continuous control (ST reaching eliminates chattering)
  - Stronger disturbance rejection (terminal attraction)

References
----------
[1] Feng, Y., Yu, X., & Man, Z. (2002). Non-singular terminal sliding
    mode control of rigid manipulators. Automatica, 38(12), 2159-2167.
    https://doi.org/10.1016/S0005-1098(02)00147-4

[2] Zhu, Z., Xia, Y., & Fu, M. (2011). Adaptive sliding mode control
    for attitude stabilization with actuator saturation. IEEE Trans.
    Industrial Electronics, 58(10), 4898-4907.
    https://doi.org/10.1109/TIE.2011.2107719

[3] Shtessel, Y., Edwards, C., Fridman, L., & Levant, A. (2014).
    Sliding Mode Control and Observation. Birkhäuser.
    Chapter 6 (Super-Twisting) + Chapter 7 (Terminal SMC).

[4] Besnard, L., Shtessel, Y. B., & Landrum, B. (2012). Quadrotor
    vehicle control via sliding mode controller driven by sliding mode
    disturbance observer. Journal of the Franklin Institute, 349(2).
    https://doi.org/10.1016/j.jfranklin.2011.06.031

[5] IMRCLab/crazyswarm2 — CF2.1 physical parameters
    https://github.com/IMRCLab/crazyswarm2

Surface Definition (per channel)
---------------------------------
    s_i = e_i  +  (1/beta_i^gamma) * |ė_i|^gamma * sign(ė_i)

Control Law
-----------
Altitude:
    U1 = (m/cos(φ)cos(θ)) * [z̈d + g
           + beta_z^gamma*(1/gamma)*|ėz|^(2-gamma)*sign(ėz)
           + k1_z*sqrt(|s_z|)*sign(s_z) + k2_z*I_u1]

Roll (U2), Pitch (U3), Yaw (U4): analogous with Ixx/Iyy/Izz scaling.

XY: virtual inputs ux, uy computed via NTSM then projected to φ_cmd, θ_cmd.

Filtered derivatives
---------------------
phi_cmd_dot and theta_cmd_dot are passed through a first-order low-pass
filter (alpha_f) before differentiation to suppress numerical noise from
the finite-difference:
    filtered = alpha_f * raw  +  (1 - alpha_f) * prev_filtered
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


def _signed_pow(x: float, exp: float) -> float:
    """
    Compute |x|^exp * sign(x) — the nonsingular power function.
    Avoids complex numbers when x < 0 and exp is fractional.
    Always well-defined, even at x = 0.
    """
    if x == 0.0:
        return 0.0
    return math.pow(abs(x), exp) * math.copysign(1.0, x)


def _ntsm_surface(e: float, e_dot: float,
                  beta: float, gamma: float) -> float:
    """
    Nonsingular Terminal Sliding Surface:
        s = e  +  (1/beta^gamma) * |ė|^gamma * sign(ė)
    """
    return e + (1.0 / (beta ** gamma)) * _signed_pow(e_dot, gamma)


def _ntsm_term(e_dot: float, beta: float, gamma: float) -> float:
    """
    Nonsingular terminal reaching term (cancels the surface derivative):
        beta^gamma * (1/gamma) * |ė|^(2-gamma) * sign(ė)
    """
    return (beta ** gamma) * (1.0 / gamma) * _signed_pow(e_dot, 2.0 - gamma)


def _st_term(s: float, I: float, k1: float, k2: float) -> float:
    """
    Super-Twisting switching term:
        k1 * sqrt(|s|) * sign(s)  +  k2 * I
    """
    return k1 * math.sqrt(abs(s)) * math.copysign(1.0, s) + k2 * I


# ── controller ─────────────────────────────────────────────────────────────

class NSTTSlidingModeController(BaseController):
    """
    Nonsingular Super-Twisting Terminal SMC.

    Key differences from SuperTwistingSMC:
      - Nonlinear terminal sliding surface (finite-time convergence)
      - gamma = p/q > 1 introduces terminal attraction
      - Per-channel beta parameters shape convergence speed
      - Low-pass filtered phi_cmd/theta_cmd derivatives (alpha_f)
      - Singularity protection via mu_min on |ė|^(2-gamma)
      - ux division uses max(|U1|, m*g*0.3) for safety

    Gains (controller_params.yaml → nstt_smc):
      p, q                    — terminal exponent numerator/denominator
      beta_z, beta_phi, beta_theta, beta_psi, beta_x, beta_y
                              — surface shape per channel
      mu_min                  — singularity protection floor
      alpha_f_phi, alpha_f_theta — low-pass filter coefficients
      kpx, kpy, kdx, kdy      — velocity/accel reference scaling
      k1_z, k2_z             — ST gains altitude
      k1_att, k2_att         — ST gains roll + pitch
      k1_psi, k2_psi         — ST gains yaw
      k1_x, k2_x            — ST gains x position
      k1_y, k2_y            — ST gains y position
    """

    def __init__(self, params: dict, dt: float = 0.01):
        super().__init__(params)
        self.dt = dt

        # ── physical params (CF2.1) ──────────────────────────────────────
        p = params
        self.m   = p['mass']
        self.g   = p['g']
        self.Ixx = p['Ixx']
        self.Iyy = p['Iyy']
        self.Izz = p['Izz']

        # ── control limits ───────────────────────────────────────────────
        self.U1_MAX  = p.get('U1_MAX',  1.3)
        self.U2_MAX  = p.get('U2_MAX',  0.002)
        self.U3_MAX  = p.get('U3_MAX',  0.002)
        self.U4_MAX  = p.get('U4_MAX',  0.001)
        self.UXY_MAX = p.get('UXY_MAX', 0.3)

        # ── NSTT-SMC gains ───────────────────────────────────────────────
        g = p.get('nstt_smc', {})

        # Terminal exponent: gamma = p/q  (must be > 1 for terminal attraction)
        self.p_exp = g.get('p_exp', 5)
        self.q_exp = g.get('q_exp', 3)
        self.gamma = self.p_exp / self.q_exp   # 5/3 ≈ 1.667

        # Surface shape parameters per channel
        self.beta_z     = g.get('beta_z',     3.0)
        self.beta_phi   = g.get('beta_phi',   3.0)
        self.beta_theta = g.get('beta_theta', 2.0)
        self.beta_psi   = g.get('beta_psi',   1.0)
        self.beta_x     = g.get('beta_x',     2.0)
        self.beta_y     = g.get('beta_y',     2.0)

        # Singularity protection: clamps |ė|^(2-gamma) away from ∞
        self.mu_min = g.get('mu_min', 0.05)

        # Low-pass filter coefficients for φ_cmd, θ_cmd derivatives
        self.alpha_f_phi   = g.get('alpha_f_phi',   0.5)
        self.alpha_f_theta = g.get('alpha_f_theta', 0.5)

        # XY velocity/accel reference scaling
        self.kpx = g.get('kpx', 0.9)
        self.kpy = g.get('kpy', 0.85)
        self.kdx = g.get('kdx', 0.58)
        self.kdy = g.get('kdy', 0.60)

        # Super-twisting gains
        self.k1_z   = g.get('k1_z',   3.0)
        self.k2_z   = g.get('k2_z',   2.5)
        self.k1_att = g.get('k1_att', 2.6)
        self.k2_att = g.get('k2_att', 2.0)
        self.k1_psi = g.get('k1_psi', 0.5)
        self.k2_psi = g.get('k2_psi', 0.6)
        self.k1_x   = g.get('k1_x',   0.45)
        self.k2_x   = g.get('k2_x',   0.42)
        self.k1_y   = g.get('k1_y',   0.35)
        self.k2_y   = g.get('k2_y',   0.40)

        self.reset()

    # -----------------------------------------------------------------------

    def reset(self):
        """Reset all integral and filter states."""
        # ST integral states  ∫ sign(s) dt
        self.I_u1  = 0.0
        self.I_u2  = 0.0
        self.I_u3  = 0.0
        self.I_u4  = 0.0
        self.I_ux  = 0.0
        self.I_uy  = 0.0

        # Attitude command memory
        self.phi_cmd_prev      = 0.0
        self.theta_cmd_prev    = 0.0

        # Filtered derivatives (low-pass state)
        self.phi_cmd_dot_filt   = 0.0
        self.phi_cmd_ddot_filt  = 0.0
        self.theta_cmd_dot_filt  = 0.0
        self.theta_cmd_ddot_filt = 0.0

        # Previous filtered dot values for ddot computation
        self.phi_cmd_dot_prev_filt   = 0.0
        self.theta_cmd_dot_prev_filt = 0.0

    # -----------------------------------------------------------------------

    def _protected_ntsm_term(self, e_dot: float,
                              beta: float, gamma: float) -> float:
        """
        Nonsingular terminal term with singularity protection.
        When |ė| < mu_min and (2-gamma) < 0, the term would blow up.
        Clamp |ė| to mu_min before computing.
        """
        e_dot_safe = math.copysign(
            max(abs(e_dot), self.mu_min), e_dot
        ) if abs(e_dot) < self.mu_min else e_dot
        return _ntsm_term(e_dot_safe, beta, gamma)

    # -----------------------------------------------------------------------

    def compute_control(self,
                        state: np.ndarray,
                        reference: np.ndarray,
                        dt: float) -> np.ndarray:
        """
        Parameters
        ----------
        state : np.ndarray (12,)
            [x, y, z, φ, θ, ψ,  ẋ, ẏ, ż,  φ̇, θ̇, ψ̇]
        reference : np.ndarray (12,)
            figure8_trajectory layout:
            [x, y, z, phi_d, theta_d, psi_d,
             x_dot, y_dot, z_dot, p, q, r]
        dt : float
            Control timestep (s)

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

        # ── unpack reference ────────────────────────────────────────────
        xd     = float(reference[0])
        yd     = float(reference[1])
        zd     = float(reference[2])
        psid   = float(reference[5])
        zd_dot = float(reference[8])

        # ── guard cos(φ)cos(θ) ──────────────────────────────────────────
        cphi_ctheta = math.cos(phi) * math.cos(theta)
        if abs(cphi_ctheta) < 1e-4:
            cphi_ctheta = math.copysign(1e-4, cphi_ctheta)

        gamma = self.gamma

        # ══════════════════════════════════════════════════════════════
        # ALTITUDE — U1
        # s_z = ez + (1/beta_z^gamma)*|ėz|^gamma*sign(ėz)
        # ══════════════════════════════════════════════════════════════
        ez     = zd - z
        ez_dot = zd_dot - z_dot

        s_z = _ntsm_surface(ez, ez_dot, self.beta_z, gamma)

        self.I_u1 += math.copysign(1.0, s_z) * dt
        self.I_u1 = max(-50.0, min(50.0, self.I_u1))

        ntsm_z = self._protected_ntsm_term(ez_dot, self.beta_z, gamma)

        U1 = (self.m / cphi_ctheta) * (
            0.0           # z_ddot_des = 0 (constant z in figure8)
            + self.g
            + ntsm_z
            + _st_term(s_z, self.I_u1, self.k1_z, self.k2_z)
        )
        U1 = max(0.0, min(self.U1_MAX, U1))

        # Safe denominator for ux/uy — MATLAB uses max(|u1_cmd|, m*g*0.3)
        U1_safe = max(abs(U1), self.m * self.g * 0.3)

        # ══════════════════════════════════════════════════════════════
        # XY VIRTUAL INPUTS
        # Velocity reference via proportional scaling
        # ══════════════════════════════════════════════════════════════
        xd_dot  = self.kpx * (xd - x_pos)
        yd_dot  = self.kpy * (yd - y_pos)
        xd_ddot = self.kdx * (xd_dot - x_dot)
        yd_ddot = self.kdy * (yd_dot - y_dot)

        # — X —
        ex     = xd - x_pos
        ex_dot = xd_dot - x_dot

        s_x = _ntsm_surface(ex, ex_dot, self.beta_x, gamma)

        self.I_ux += math.copysign(1.0, s_x) * dt
        self.I_ux = max(-50.0, min(50.0, self.I_ux))

        ntsm_x = self._protected_ntsm_term(ex_dot, self.beta_x, gamma)

        ux_raw = self.m * (
            xd_ddot
            + ntsm_x
            + _st_term(s_x, self.I_ux, self.k1_x, self.k2_x)
        )
        ux_raw = max(-1.0, min(1.0, ux_raw))
        ux = ux_raw / U1_safe
        ux = max(-self.UXY_MAX, min(self.UXY_MAX, ux))

        # — Y —
        ey     = yd - y_pos
        ey_dot = yd_dot - y_dot

        s_y = _ntsm_surface(ey, ey_dot, self.beta_y, gamma)

        self.I_uy += math.copysign(1.0, s_y) * dt
        self.I_uy = max(-50.0, min(50.0, self.I_uy))

        ntsm_y = self._protected_ntsm_term(ey_dot, self.beta_y, gamma)

        uy_raw = self.m * (
            yd_ddot
            + ntsm_y
            + _st_term(s_y, self.I_uy, self.k1_y, self.k2_y)
        )
        uy_raw = max(-1.0, min(1.0, uy_raw))
        uy = uy_raw / U1_safe
        uy = max(-self.UXY_MAX, min(self.UXY_MAX, uy))

        # ── attitude commands with low-pass filtered derivatives ─────
        spsi = math.sin(psid)
        cpsi = math.cos(psid)

        phi_cmd = _asin_norm(ux * spsi - uy * cpsi)

        # Filtered phi_cmd_dot
        raw_phi_cmd_dot        = _wrap(phi_cmd - self.phi_cmd_prev) / dt
        self.phi_cmd_dot_filt  = (self.alpha_f_phi * raw_phi_cmd_dot
                                  + (1.0 - self.alpha_f_phi) * self.phi_cmd_dot_filt)
        phi_cmd_dot = self.phi_cmd_dot_filt

        # Filtered phi_cmd_ddot
        raw_phi_cmd_ddot       = (self.phi_cmd_dot_filt
                                  - self.phi_cmd_dot_prev_filt) / dt
        self.phi_cmd_ddot_filt = (self.alpha_f_phi * raw_phi_cmd_ddot
                                  + (1.0 - self.alpha_f_phi) * self.phi_cmd_ddot_filt)
        phi_cmd_ddot = self.phi_cmd_ddot_filt

        cos_phi_cmd = math.cos(phi_cmd)
        if abs(cos_phi_cmd) < 1e-4:
            cos_phi_cmd = math.copysign(1e-4, cos_phi_cmd)

        theta_cmd = _asin_norm((ux * cpsi + uy * spsi) / cos_phi_cmd)

        # Filtered theta_cmd_dot
        raw_theta_cmd_dot         = _wrap(theta_cmd - self.theta_cmd_prev) / dt
        self.theta_cmd_dot_filt   = (self.alpha_f_theta * raw_theta_cmd_dot
                                     + (1.0 - self.alpha_f_theta) * self.theta_cmd_dot_filt)
        theta_cmd_dot = self.theta_cmd_dot_filt

        # Filtered theta_cmd_ddot
        raw_theta_cmd_ddot         = (self.theta_cmd_dot_filt
                                      - self.theta_cmd_dot_prev_filt) / dt
        self.theta_cmd_ddot_filt   = (self.alpha_f_theta * raw_theta_cmd_ddot
                                      + (1.0 - self.alpha_f_theta) * self.theta_cmd_ddot_filt)
        theta_cmd_ddot = self.theta_cmd_ddot_filt

        # ══════════════════════════════════════════════════════════════
        # ROLL — U2
        # s_phi = ephi + (1/beta_phi^gamma)*|ėphi|^gamma*sign(ėphi)
        # ══════════════════════════════════════════════════════════════
        ephi     = _wrap(phi_cmd - phi)
        ephi_dot = _wrap(phi_cmd_dot - phi_dot)

        s_phi = _ntsm_surface(ephi, ephi_dot, self.beta_phi, gamma)

        self.I_u2 += math.copysign(1.0, s_phi) * dt
        self.I_u2 = max(-50.0, min(50.0, self.I_u2))

        ntsm_phi = self._protected_ntsm_term(ephi_dot, self.beta_phi, gamma)

        U2 = self.Ixx * (
            phi_cmd_ddot
            - ((self.Iyy - self.Izz) / self.Ixx) * theta_dot * psi_dot
            + ntsm_phi
            + _st_term(s_phi, self.I_u2, self.k1_att, self.k2_att)
        )

        # ══════════════════════════════════════════════════════════════
        # PITCH — U3
        # ══════════════════════════════════════════════════════════════
        etheta     = _wrap(theta_cmd - theta)
        etheta_dot = _wrap(theta_cmd_dot - theta_dot)

        s_theta = _ntsm_surface(etheta, etheta_dot, self.beta_theta, gamma)

        self.I_u3 += math.copysign(1.0, s_theta) * dt
        self.I_u3 = max(-50.0, min(50.0, self.I_u3))

        ntsm_theta = self._protected_ntsm_term(etheta_dot, self.beta_theta, gamma)

        U3 = self.Iyy * (
            theta_cmd_ddot
            - ((self.Izz - self.Ixx) / self.Iyy) * phi_dot * psi_dot
            + ntsm_theta
            + _st_term(s_theta, self.I_u3, self.k1_att, self.k2_att)
        )

        # ══════════════════════════════════════════════════════════════
        # YAW — U4
        # ══════════════════════════════════════════════════════════════
        epsi     = _wrap(psid - psi)
        epsi_dot = _wrap(0.0 - psi_dot)

        s_psi = _ntsm_surface(epsi, epsi_dot, self.beta_psi, gamma)

        self.I_u4 += math.copysign(1.0, s_psi) * dt
        self.I_u4 = max(-50.0, min(50.0, self.I_u4))

        ntsm_psi = self._protected_ntsm_term(epsi_dot, self.beta_psi, gamma)

        U4 = self.Izz * (
            0.0           # psi_cmd_ddot = 0
            - ((self.Ixx - self.Iyy) / self.Izz) * phi_dot * theta_dot
            + ntsm_psi
            + _st_term(s_psi, self.I_u4, self.k1_psi, self.k2_psi)
        )

        # ── clamp torques ────────────────────────────────────────────
        U2 = max(-self.U2_MAX, min(self.U2_MAX, U2))
        U3 = max(-self.U3_MAX, min(self.U3_MAX, U3))
        U4 = max(-self.U4_MAX, min(self.U4_MAX, U4))

        # ── update memory ────────────────────────────────────────────
        self.phi_cmd_prev            = phi_cmd
        self.theta_cmd_prev          = theta_cmd
        self.phi_cmd_dot_prev_filt   = self.phi_cmd_dot_filt
        self.theta_cmd_dot_prev_filt = self.theta_cmd_dot_filt

        return np.array([U1, U2, U3, U4])