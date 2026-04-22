# Update cf21_parameters.py with corrected limits
"""
cf21_parameters.py
==================
Single source of truth for all Crazyflie 2.1 physical parameters.

To use different vehicle parameters, create a new file (e.g. cf21_heavy_params.py)
with the same structure and import that instead — no other file needs to change.

Physical parameters sourced from:
  - IMRCLab/crazyswarm2: crazyflie2.urdf, crazyflie2.yaml
  - Bitcraze Crazyflie 2.1 datasheet
  - Silano et al., CrazyS, ROBOT 2017 (arXiv:1811.03557)
"""

import math

# =============================================================================
# VEHICLE PHYSICAL PARAMETERS
# =============================================================================

# Mass [kg]
MASS = 0.034

# Gravitational acceleration [m/s^2]
GRAVITY = 9.81

# Inertia tensor diagonal elements [kg.m^2]
# Source: crazyflie2.urdf from IMRCLab/crazyswarm2
IXX = 16.571710e-6
IYY = 16.655602e-6
IZZ = 29.261652e-6

# Arm length (center to motor) [m]
ARM_LENGTH = 0.046

# Thrust-to-torque ratio [dimensionless]
THRUST_TO_TORQUE = 0.006

# Maximum total thrust (all 4 motors) [N]
MAX_THRUST_TOTAL = 1.3

# Maximum thrust per motor [N]
MAX_THRUST_PER_MOTOR = MAX_THRUST_TOTAL / 4.0   # 0.325 N

# Minimum thrust [N]
MIN_THRUST_TOTAL = 0.0

# =============================================================================
# CONTROL OUTPUT LIMITS
# Physics-based derivation:
#   Max roll/pitch torque = F_max_per_motor * arm_length * sqrt(2)
#                         = 0.325 * 0.046 * 1.414 = 0.02116 N.m
#   Max yaw torque = 4 * F_max_per_motor * thrust_to_torque
#                  = 4 * 0.325 * 0.006 = 0.0078 N.m
# =============================================================================

# Thrust limits [N]
U1_MAX = MAX_THRUST_TOTAL    # 1.3 N
U1_MIN = 0.0

# Roll torque limits [N.m] — physics-based
U2_MAX =  0.002
U2_MIN = -0.002

# Pitch torque limits [N.m] — physics-based
U3_MAX =  0.002
U3_MIN = -0.002

# Yaw torque limits [N.m] — physics-based
U4_MAX =  0.001
U4_MIN = -0.001

# =============================================================================
# ANGLE LIMITS [radians]
# =============================================================================
PHI_MAX   = math.radians(30)     # Max roll  — reduced for stability
THETA_MAX = math.radians(30)     # Max pitch — reduced for stability
PSI_MAX   = math.radians(180)    # Max yaw

# =============================================================================
# SIMULATION / CONTROL PARAMETERS
# =============================================================================
CONTROL_FREQUENCY = 100.0        # Hz — reduced from 500 for stability
DT = 1.0 / CONTROL_FREQUENCY
