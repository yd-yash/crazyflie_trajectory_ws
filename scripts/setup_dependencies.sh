#!/bin/bash
# =============================================================================
# setup_dependencies.sh
# =============================================================================
# Sets up the complete Crazyflie simulation environment on a fresh Ubuntu 22.04
# system with ROS2 Humble already installed.
#
# Usage:
#   cd ~/crazyflie_control_ws
#   chmod +x scripts/setup_dependencies.sh
#   ./scripts/setup_dependencies.sh
#
# What this script does:
#   1. Installs all system apt dependencies
#   2. Creates the Python virtual environment
#   3. Installs all Python control libraries
#   4. Clones crazyswarm2 and motion_capture_tracking
#   5. Runs rosdep for all dependencies
#   6. Builds the full workspace
# =============================================================================

set -e  # Exit on any error

WORKSPACE_ROOT="$HOME/crazyflie_control_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ""
echo "============================================================"
echo "  Crazyflie Control Workspace — Dependency Setup"
echo "  Workspace: $WORKSPACE_ROOT"
echo "============================================================"
echo ""

# ─── Sanity checks ───────────────────────────────────────────────────────────
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble/"
    echo "Please install ROS2 Humble first:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "WARNING: This script is tested on Ubuntu 22.04 (jammy)."
    echo "Current OS: $(lsb_release -ds)"
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
fi

source /opt/ros/humble/setup.bash
echo "[OK] ROS2 Humble sourced"

# ─── Step 1: System dependencies ─────────────────────────────────────────────
echo ""
echo "--- Step 1: Installing system dependencies ---"
sudo apt-get update -q
sudo apt-get install -y \
    git \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-plot \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    libgazebo-dev \
    ros-humble-gazebo-dev
echo "[OK] System dependencies installed"

# ─── Step 2: Python virtual environment ──────────────────────────────────────
echo ""
echo "--- Step 2: Creating Python virtual environment ---"
cd "$WORKSPACE_ROOT"

if [ -d "cf_control_venv" ]; then
    echo "[SKIP] cf_control_venv already exists"
else
    python3 -m venv cf_control_venv --system-site-packages
    echo "[OK] Virtual environment created"
fi

source "$WORKSPACE_ROOT/cf_control_venv/bin/activate"
pip install --upgrade pip --quiet

echo "Installing Python control libraries..."
pip install --quiet \
    numpy==1.26.4 \
    scipy==1.13.0 \
    matplotlib==3.8.4 \
    transforms3d==0.4.1 \
    control==0.10.1 \
    casadi==3.6.5 \
    pandas==2.2.2 \
    pyquaternion==0.9.9 \
    pyyaml==6.0.1 \
    pytest==8.2.0 \
    cflib==0.1.26

echo "[OK] Python libraries installed"

# Fix mpl_toolkits namespace issue
cat > "$WORKSPACE_ROOT/cf_control_venv/lib/python3.10/site-packages/mpl_toolkits/__init__.py" << 'PYEOF'
# Prevents Python from merging with system mpl_toolkits namespace.
PYEOF
echo "[OK] mpl_toolkits namespace fixed"

# Prevent colcon from scanning the venv
touch "$WORKSPACE_ROOT/cf_control_venv/COLCON_IGNORE"

# ─── Step 3: Clone external dependencies ─────────────────────────────────────
echo ""
echo "--- Step 3: Cloning external dependencies ---"
cd "$WORKSPACE_ROOT/src"

if [ -d "crazyswarm2" ]; then
    echo "[SKIP] crazyswarm2 already exists — pulling latest..."
    cd crazyswarm2
    git pull --rebase
    git submodule update --init --recursive
    cd ..
else
    echo "Cloning crazyswarm2..."
    git clone --branch main --recurse-submodules \
        https://github.com/IMRCLab/crazyswarm2.git
    echo "[OK] crazyswarm2 cloned"
fi

if [ -d "motion_capture_tracking" ]; then
    echo "[SKIP] motion_capture_tracking already exists — pulling latest..."
    cd motion_capture_tracking && git pull --rebase && cd ..
else
    echo "Cloning motion_capture_tracking..."
    git clone --branch main \
        https://github.com/IMRCLab/motion_capture_tracking.git
    echo "[OK] motion_capture_tracking cloned"
fi

# Install crazyswarm2 Python deps
pip install --quiet vispy==0.14.3 nicegui==1.4.22 importlib-metadata==7.2.1

# ─── Step 4: rosdep ──────────────────────────────────────────────────────────
echo ""
echo "--- Step 4: Running rosdep ---"
cd "$WORKSPACE_ROOT"

sudo rosdep init 2>/dev/null || echo "[SKIP] rosdep already initialized"
rosdep update --quiet
rosdep install --from-paths src/crazyswarm2 --ignore-src -r -y --quiet
echo "[OK] rosdep complete"

# ─── Step 5: Build workspace ──────────────────────────────────────────────────
echo ""
echo "--- Step 5: Building workspace ---"
cd "$WORKSPACE_ROOT"

colcon build --symlink-install \
    --packages-select \
        crazyflie_interfaces \
        crazyflie \
        crazyflie_py \
        crazyflie_sim \
        crazyflie_examples \
        crazyflie_gazebo_sim \
        cf_trajectory_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tee logs/build_setup.log

echo "[OK] Workspace built"

# ─── Step 6: Final summary ───────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  Setup Complete!"
echo "============================================================"
echo ""
echo "To activate the workspace in any terminal:"
echo "  source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh"
echo ""
echo "To run the simulation:"
echo "  ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py"
echo ""
echo "To run the plotter:"
echo "  ros2 run cf_trajectory_controller cf_trajectory_plotter"
echo "============================================================"
