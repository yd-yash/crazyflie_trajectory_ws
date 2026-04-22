# Sim-to-Real Pipeline for Quadrotor Trajectory Tracking

Simulation environment for testing trajectory tracking controllers on a
**Crazyflie 2.1** nano-quadrotor using **ROS2 Humble** and **Gazebo Classic 11**.

## Platform

| Software | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Gazebo Classic | 11.x |
| Python | 3.10.x |

## Quick Start (Fresh System)

### Prerequisites
- Ubuntu 22.04 with ROS2 Humble installed
- Git configured with your name and email

### 1. Clone this repository

```bash
git clone https://github.com/YOUR_USERNAME/crazyflie_trajectory_ws.git ~/crazyflie_control_ws
cd ~/crazyflie_control_ws
```

### 2. Run the setup script (one time only)

```bash
chmod +x scripts/setup_dependencies.sh
./scripts/setup_dependencies.sh
```

This script:
- Installs all apt and Python dependencies
- Creates the Python virtual environment
- Clones `crazyswarm2` and `motion_capture_tracking`
- Builds the complete workspace

### 3. Activate workspace (every new terminal)

```bash
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
```

### 4. Run the simulation

**Terminal 1:**
```bash
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py
```

**Terminal 2 (after 10 seconds):**
```bash
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
ros2 run cf_trajectory_controller cf_trajectory_plotter
```

## Switch Controller or Trajectory

Edit **one line** in the config file:
```bash
nano src/cf_trajectory_controller/config/controller_params.yaml
```

```yaml
active_controller: "cascaded_pid"   # cascaded_pid | conventional_smc | ...
active_trajectory: "figure8"        # figure8 | circle | helix | hover
```

## Repository Structure
crazyflie_control_ws/
├── src/
│   ├── crazyflie_gazebo_sim/        # Gazebo world, SDF model, C++ plugin
│   └── cf_trajectory_controller/   # Controllers, trajectories, ROS2 nodes
├── scripts/
│   ├── activate_crazyflie_ws.sh     # Source this every session
│   └── setup_dependencies.sh        # Run once on fresh system
├── docs/
│   ├── crazyflie_sim_guide.md       # Complete replication guide
│   └── architecture_doc.md         # Software architecture reference
└── logs/                            # Plots and metrics (gitignored)

## Controllers Implemented

| Controller | Status | RMS 3D Error |
|---|---|---|
| Cascaded PID | ✅ Working | 0.055 m |
| Conventional SMC | 🔄 In progress | — |
| Super Twisting SMC | 🔄 Planned | — |
| NS Terminal SMC | 🔄 Planned | — |

## Documentation

- [Complete Replication Guide](docs/crazyflie_sim_guide.md)
- [Architecture Reference](docs/architecture_doc.md)

## References

- [IMRCLab/crazyswarm2](https://github.com/IMRCLab/crazyswarm2)
- Silano et al., *CrazyS: A Software-in-the-Loop Simulation Platform for the Crazyflie 2.0*, ROBOT 2017 ([arXiv:1811.03557](https://arxiv.org/abs/1811.03557))
