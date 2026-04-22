
#!/bin/bash
# =============================================================================
# Crazyflie Control Workspace Activation Script
# Project: Sim-to-Real Pipeline for Quadrotor Trajectory Tracking
# Usage: source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
# =============================================================================

WORKSPACE_ROOT="$HOME/crazyflie_control_ws"

# --- Step 1: Source ROS2 Humble ---
source /opt/ros/humble/setup.bash
echo "[1/4] ROS2 Humble sourced"

# --- Step 2: Source the ROS2 workspace (once packages are built) ---
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "[2/4] Crazyflie ROS2 workspace sourced"
else
    echo "[2/4] ROS2 workspace not built yet (skip — expected in early phases)"
fi
# --- Step 3: Activate Python venv ---
source "$WORKSPACE_ROOT/cf_control_venv/bin/activate"
echo "[3/4] Python venv cf_control_venv activated"

# --- Step 4: Set project environment variables ---
export CRAZYFLIE_WS="$WORKSPACE_ROOT"
export CRAZYFLIE_SRC="$WORKSPACE_ROOT/src"
export CRAZYFLIE_CONFIGS="$WORKSPACE_ROOT/configs"
export CRAZYFLIE_LOGS="$WORKSPACE_ROOT/logs"
export GAZEBO_MODEL_PATH="$WORKSPACE_ROOT/src/crazyflie_gazebo_sim/models:$GAZEBO_MODEL_PATH"
export PYTHONPATH="$WORKSPACE_ROOT/src:$PYTHONPATH"
echo "[4/4] Environment variables set"
# --- Summary ---
echo ""
echo "=============================================="
echo "  Crazyflie Control Workspace Ready"
echo "  ROS2 Distro  : $ROS_DISTRO"
echo "  Python        : $(python3 --version)"
echo "  Workspace     : $CRAZYFLIE_WS"
echo "=============================================="

# --- Fix: Force venv site-packages to front of PYTHONPATH ---
# This ensures our pinned libraries (matplotlib 3.8.4, numpy 1.26.4)
# always take priority over system and ROS2 injected paths
VENV_SITE="$WORKSPACE_ROOT/cf_control_venv/lib/python3.10/site-packages"
export PYTHONPATH="$VENV_SITE:$PYTHONPATH"
echo "[fix] venv site-packages prepended to PYTHONPATH"
