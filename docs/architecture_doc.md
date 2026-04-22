# Sim-to-Real Pipeline for Quadrotor Trajectory Tracking
## Software Architecture Reference
### Project: Crazyflie 2.1 ROS2 + Gazebo Simulation
### Date: April 2026 | Platform: Ubuntu 22.04 + ROS2 Humble

---

## Table of Contents
1. Project Overview
2. Directory Structure
3. ROS2 Package Overview
4. File-by-File Reference
5. ROS2 Node Graph
6. Topic Reference
7. Data Flow (Step by Step)
8. Coordinate Frames
9. Plug-and-Play Controller System
10. Plug-and-Play Trajectory System
11. Adding a New Controller (Checklist)
12. Adding a New Trajectory (Checklist)
13. Configuration Reference

---

## 1. Project Overview

This project implements a simulation environment for testing trajectory tracking controllers on a Crazyflie 2.1 nano-quadrotor. The simulation runs in Gazebo Classic 11, controlled via ROS2 Humble.

The software is split into two ROS2 packages:

| Package | Purpose |
|---|---|
| `crazyflie_gazebo_sim` | Simulation environment: Gazebo world, drone model, physics plugin |
| `cf_trajectory_controller` | Control logic: controller, trajectory, plotter, launch files |

The third component, `crazyswarm2`, is an external package cloned from IMRCLab that provides the Crazyflie ROS2 interface definitions (message types).

---

## 2. Directory Structure

```
~/crazyflie_control_ws/                          ← Main workspace root
│
├── scripts/
│   └── activate_crazyflie_ws.sh                 ← Source this to set up environment
│
├── config/                                       ← Global configs (currently empty)
├── logs/                                         ← All plot outputs + build logs
├── docs/                                         ← Documentation (this file)
│
├── cf_control_venv/                              ← Python virtual environment
│   └── lib/python3.10/site-packages/            ← numpy, scipy, cflib, etc.
│
├── src/                                          ← All ROS2 source packages
│   │
│   ├── crazyflie_gazebo_sim/                    ← PACKAGE 1: Simulation
│   │   ├── CMakeLists.txt                        ← Build config (ament_cmake)
│   │   ├── package.xml                           ← Package metadata + dependencies
│   │   ├── src/
│   │   │   └── cf_motor_plugin.cpp               ← C++ Gazebo physics plugin
│   │   ├── models/
│   │   │   └── crazyflie_2_1/
│   │   │       ├── model.config                  ← Gazebo model metadata
│   │   │       ├── model.sdf                     ← Drone model: links, joints, plugins
│   │   │       └── meshes/
│   │   │           └── cf2_assembly_with_props.dae ← 3D visual mesh
│   │   ├── worlds/
│   │   │   └── crazyflie_trajectory_world.world  ← Gazebo world file
│   │   ├── urdf/
│   │   │   └── crazyflie_2_1_gazebo.urdf.xacro   ← URDF for robot_state_publisher
│   │   ├── config/
│   │   │   └── cf1_sim_params.yaml               ← Sim parameters reference
│   │   └── launch/
│   │       └── gazebo_cf1_sim.launch.py          ← Launches Gazebo + spawns drone
│   │
│   ├── cf_trajectory_controller/                ← PACKAGE 2: Control
│   │   ├── package.xml
│   │   ├── setup.py                              ← Python package entry points
│   │   ├── config/
│   │   │   └── controller_params.yaml            ← ALL tunable gains (edit here)
│   │   ├── launch/
│   │   │   └── cf1_full_simulation.launch.py     ← MAIN launch file (use this)
│   │   └── cf_trajectory_controller/            ← Python package root
│   │       ├── core/
│   │       │   └── cf21_parameters.py            ← Physical constants (one place)
│   │       ├── controllers/
│   │       │   ├── base_controller.py            ← Abstract interface
│   │       │   └── cascaded_pid.py               ← PID implementation
│   │       ├── trajectories/
│   │       │   └── figure8_trajectory.py         ← Figure-8 generator
│   │       ├── nodes/
│   │       │   ├── cf_controller_node.py         ← Main ROS2 control node
│   │       │   └── cf_gazebo_force_node.py       ← (unused, superseded by C++ plugin)
│   │       └── utils/
│   │           └── cf_trajectory_plotter.py      ← Data recording + plot generation
│   │
│   └── crazyswarm2/                             ← PACKAGE 3: External (Bitcraze)
│       ├── crazyflie_interfaces/                 ← Custom ROS2 message definitions
│       └── crazyflie_sim/                        ← (not used in our pipeline)
│
├── build/                                        ← colcon build output (auto-generated)
├── install/                                      ← colcon install output (auto-generated)
└── log/                                          ← colcon build logs (auto-generated)
```

---

## 3. ROS2 Package Overview

### Package 1: `crazyflie_gazebo_sim`
- **Build type:** ament_cmake (has C++ code)
- **Purpose:** Everything related to the Gazebo simulation environment
- **Key output:** Compiled `libcf_motor_plugin.so` — loaded by Gazebo at runtime
- **Does NOT:** Implement any control logic

### Package 2: `cf_trajectory_controller`
- **Build type:** ament_python
- **Purpose:** All control logic — controllers, trajectories, ROS2 nodes, plotting
- **Key output:** Three executable ROS2 nodes registered in setup.py
- **Does NOT:** Know anything about Gazebo internals

### Package 3: `crazyswarm2` (external)
- **Purpose:** Provides `crazyflie_interfaces` — custom ROS2 message types
- **Key message used:** `crazyflie_interfaces/msg/FullState`
- **We use:** Only the interfaces package, not the full simulator

---

## 4. File-by-File Reference

### crazyflie_gazebo_sim package

**`src/cf_motor_plugin.cpp`**
The most critical file in the simulation package. This is a Gazebo Classic model plugin written in C++ that directly applies forces and torques to the drone's physics body inside Gazebo's physics update loop.

Why C++ and not Python: Gazebo Classic plugins must be compiled shared libraries. Python cannot write Gazebo model plugins.

What it does at runtime:
1. Gazebo loads it when the drone model spawns
2. It subscribes to `/cf1/control_debug` (Float64MultiArray)
3. Every physics step (~1000Hz), it reads [u1, u2, u3, u4] from that topic
4. It rotates u1 (thrust) from body frame to world frame using current orientation
5. It calls `link->AddForce()` and `link->AddRelativeTorque()` directly in physics

**`models/crazyflie_2_1/model.sdf`**
The SDF (Simulation Description Format) file that defines the Crazyflie model in Gazebo. Contains:
- Base link with exact physical parameters (mass=0.034kg, inertia from datasheet)
- 4 rotor links connected via revolute joints
- 3 Gazebo plugins embedded: IMU sensor, ground truth odometry (p3d), motor plugin

**`worlds/crazyflie_trajectory_world.world`**
Defines the Gazebo simulation environment: physics engine settings (ODE, 1000Hz step rate), lighting, ground plane, and initial camera position.

**`urdf/crazyflie_2_1_gazebo.urdf.xacro`**
A parameterized URDF used by `robot_state_publisher` to broadcast TF2 coordinate frame transforms. It is NOT used by Gazebo directly — Gazebo uses the SDF. This file only exists to provide the TF tree for visualization tools like RViz.

**`launch/gazebo_cf1_sim.launch.py`**
Launches: Gazebo server, Gazebo client (GUI), robot_state_publisher, and the spawn_entity node that inserts the drone SDF into the running simulation.

---

### cf_trajectory_controller package

**`core/cf21_parameters.py`**
The single source of truth for all Crazyflie 2.1 physical constants. Every other file imports from here. If you change the vehicle (e.g. add a payload), only this file needs updating.

Key constants defined here:
- MASS, GRAVITY, IXX, IYY, IZZ (inertia)
- ARM_LENGTH, THRUST_TO_TORQUE
- U1_MAX/MIN, U2_MAX/MIN, U3_MAX/MIN, U4_MAX/MIN (control limits)
- PHI_MAX, THETA_MAX, PSI_MAX (angle limits)
- CONTROL_FREQUENCY, DT

**`controllers/base_controller.py`**
Abstract base class (ABC) that defines the interface every controller must implement. Contains two abstract methods: `compute_control()` and `reset()`. The ROS2 node only ever calls these two methods — it never knows which specific controller is loaded.

**`controllers/cascaded_pid.py`**
The cascaded PID implementation. Translated line-by-line from MATLAB script `PID_3D_05.m`. Contains the outer loop (position → attitude commands) and inner loop (attitude → torques). Comments reference the exact MATLAB line numbers.

**`trajectories/figure8_trajectory.py`**
Generates figure-8 reference trajectory. Translated from MATLAB anonymous functions. The `get_reference(t)` method returns a 12-element numpy array at any time t — position, euler angles, velocity, angular rates.

**`nodes/cf_controller_node.py`**
The main ROS2 node. This is the orchestrator. It:
1. Reads `active_controller` and `active_trajectory` from ROS2 parameters
2. Looks up the controller class in `CONTROLLER_REGISTRY` dictionary
3. Instantiates the controller with gains loaded from YAML
4. Subscribes to `/cf1/ground_truth/odom` at 100Hz
5. Runs control loop at 100Hz via timer
6. Publishes commands to `/cf1/cmd_full_state`, `/cf1/trajectory_ref`, `/cf1/control_debug`

**`utils/cf_trajectory_plotter.py`**
A ROS2 node that records data from running topics for a configurable duration, then generates 10 performance plots and a CSV metrics file. It is independent of the controller — it just subscribes to topics.

**`config/controller_params.yaml`**
The only file you need to edit to change controller gains or switch controllers. All gains are loaded from here at node startup via ROS2 parameter system.

**`launch/cf1_full_simulation.launch.py`**
The main entry point for running the complete simulation. Includes the Gazebo launch file and starts the controller node with a 5-second delay (to allow Gazebo to fully initialize).

---

## 5. ROS2 Node Graph

```
┌─────────────────────────────────────────────────────────────────┐
│                        GAZEBO PROCESS                           │
│                                                                 │
│  ┌──────────────────┐    ┌─────────────────┐                   │
│  │  /gazebo          │    │ cf_motor_plugin  │  (C++ plugin)    │
│  │  (Gazebo ROS node)│    │ inside Gazebo    │                  │
│  └──────┬───────────┘    └────────┬────────┘                   │
│         │                         │ AddForce()                  │
│         │ /clock                  │ AddRelativeTorque()         │
│         │ /performance_metrics    │                             │
│         │                    ┌────▼──────────────────────┐     │
│         │                    │   Gazebo Physics Engine    │     │
│         │                    │   (ODE, 1000Hz)            │     │
│         │                    └────┬──────────────────────┘     │
│         │                         │                             │
│  ┌──────▼───────────┐    ┌────────▼────────┐                   │
│  │ cf_ground_truth  │    │  cf_imu_plugin  │                   │
│  │ _plugin (p3d)    │    │  (IMU sensor)   │                   │
│  └──────┬───────────┘    └────────┬────────┘                   │
│         │                         │                             │
└─────────┼─────────────────────────┼───────────────────────────┘
          │ /cf1/ground_truth/odom  │ /cf1/imu
          │ (nav_msgs/Odometry)     │ (sensor_msgs/Imu)
          │ 100Hz                   │ 500Hz
          │
┌─────────▼───────────────────────────────────────────────────────┐
│                   /cf_controller_node                           │
│                   (Python ROS2 Node, 100Hz)                     │
│                                                                 │
│  odom_callback() ──► current_state[12]                         │
│                                                                 │
│  control_loop_callback() every 0.01s:                          │
│    1. trajectory.get_reference(sim_time) ──► reference[12]     │
│    2. controller.compute_control(state, ref, dt) ──► u[4]      │
│    3. publish /cf1/cmd_full_state                               │
│    4. publish /cf1/trajectory_ref                              │
│    5. publish /cf1/control_debug                               │
│                                                                 │
│  ┌─────────────────┐    ┌──────────────────────┐               │
│  │ CascadedPID     │    │ Figure8Trajectory    │               │
│  │ (or any other   │    │ (or any other        │               │
│  │  controller)    │    │  trajectory)         │               │
│  └─────────────────┘    └──────────────────────┘               │
└──────────┬──────────────────────────────────────────────────────┘
           │
           ├── /cf1/cmd_full_state  ──► (to real hardware, future)
           │   (crazyflie_interfaces/msg/FullState)
           │
           ├── /cf1/control_debug  ──► cf_motor_plugin (subscribes)
           │   (std_msgs/Float64MultiArray)    and cf_trajectory_plotter
           │   [u1, u2, u3, u4, ex, ey, ez, t]
           │
           └── /cf1/trajectory_ref ──► cf_trajectory_plotter
               (nav_msgs/Odometry)

┌─────────────────────────────────────────────────────────────────┐
│                  /cf1/cf1_state_publisher                       │
│                  (robot_state_publisher)                        │
│                                                                 │
│  Reads: /cf1/robot_description (URDF from xacro)               │
│  Publishes: /tf, /tf_static (TF2 coordinate frames)            │
│  Used by: RViz visualization (not needed for control)          │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                  /cf_trajectory_plotter                         │
│                  (Python ROS2 Node, optional)                   │
│                                                                 │
│  Subscribes to: /cf1/ground_truth/odom                         │
│                 /cf1/trajectory_ref                            │
│                 /cf1/control_debug                             │
│  Saves: 10 PNG plots + 1 CSV to ~/crazyflie_control_ws/logs/   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 6. Topic Reference

| Topic | Type | Publisher | Subscriber(s) | Rate | Content |
|---|---|---|---|---|---|
| `/cf1/ground_truth/odom` | `nav_msgs/Odometry` | cf_ground_truth_plugin (Gazebo) | cf_controller_node, cf_trajectory_plotter | 100Hz | Position, orientation (quaternion), linear+angular velocity |
| `/cf1/imu` | `sensor_msgs/Imu` | cf_imu_plugin (Gazebo) | (unused in our pipeline) | 500Hz | Angular velocity, linear acceleration with noise |
| `/cf1/cmd_full_state` | `crazyflie_interfaces/FullState` | cf_controller_node | (future: real hardware) | 100Hz | Desired position, velocity, acceleration, yaw |
| `/cf1/control_debug` | `std_msgs/Float64MultiArray` | cf_controller_node | cf_motor_plugin, cf_trajectory_plotter | 100Hz | [u1, u2, u3, u4, ex, ey, ez, sim_time] |
| `/cf1/trajectory_ref` | `nav_msgs/Odometry` | cf_controller_node | cf_trajectory_plotter | 100Hz | Reference position for current time step |
| `/cf1/joint_states` | `sensor_msgs/JointState` | robot_state_publisher | (visualization only) | 100Hz | Rotor joint angles |
| `/cf1/robot_description` | `std_msgs/String` | robot_state_publisher | (visualization only) | Latched | Full URDF as string |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | (visualization only) | 100Hz | Coordinate frame transforms |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | All nodes | 1000Hz | Simulation time |

---

## 7. Data Flow (Step by Step)

This section traces one complete control cycle from physics to command and back.

### Step 1 — Physics Simulation (inside Gazebo, 1000Hz)
Gazebo's ODE physics engine simulates the drone's rigid body dynamics every 1ms. It computes position, velocity, orientation, and angular rates based on forces applied to the body.

### Step 2 — Ground Truth Publishing (100Hz)
The `cf_ground_truth_plugin` (libgazebo_ros_p3d.so) reads the drone's current state directly from Gazebo's physics engine and publishes it as a `nav_msgs/Odometry` message on `/cf1/ground_truth/odom`. This gives perfect, noise-free state information — equivalent to an ideal motion capture system.

### Step 3 — State Reception (cf_controller_node, 100Hz)
The `odom_callback()` function in `cf_controller_node.py` receives the Odometry message and extracts the 12-element state vector:
```
state[0:3]  = [x, y, z]             from pose.position
state[3:6]  = [phi, theta, psi]     converted from quaternion using scipy Rotation
state[6:9]  = [x_dot, y_dot, z_dot] from twist.linear
state[9:12] = [phi_dot, theta_dot, psi_dot] from twist.angular
```

### Step 4 — Reference Generation (cf_controller_node, 100Hz)
The control loop timer fires every 10ms. It calls `trajectory.get_reference(sim_time)` which computes the desired 12-element reference state at the current simulation time using the trajectory equations (e.g. figure-8 sinusoids).

### Step 5 — Control Computation (cf_controller_node, 100Hz)
`controller.compute_control(state, reference, dt)` is called. The controller computes errors, runs the PID/SMC equations, and returns `u = [u1, u2, u3, u4]`:
```
u[0] = u1 = total thrust [N]
u[1] = u2 = roll torque  [N.m]
u[2] = u3 = pitch torque [N.m]
u[3] = u4 = yaw torque   [N.m]
```

### Step 6 — Command Publishing (cf_controller_node, 100Hz)
Three topics are published:
- `/cf1/cmd_full_state` — for future real hardware connection
- `/cf1/trajectory_ref` — reference position for the plotter
- `/cf1/control_debug` — control inputs + errors for the motor plugin and plotter

### Step 7 — Force Application (cf_motor_plugin, ~100Hz via ROS callback)
The C++ plugin running inside Gazebo receives the `/cf1/control_debug` message. It:
1. Gets the drone's current orientation quaternion from Gazebo directly
2. Rotates thrust vector `[0, 0, u1]` from body frame to world frame
3. Calls `link->AddForce(thrust_world)` on the base link
4. Calls `link->AddRelativeTorque([u2, u3, u4])` on the base link

### Step 8 — Physics Integration (Gazebo, 1000Hz)
Gazebo integrates the applied forces over the next physics step, producing a new drone state. The cycle repeats from Step 1.

### Timing Summary
```
Gazebo physics:     1000 Hz  (1ms steps)
IMU publishing:      500 Hz  (every 2ms)
Ground truth odom:   100 Hz  (every 10ms)
Control loop:        100 Hz  (every 10ms)
Force application: ~100 Hz  (limited by ROS2 callback rate)
```

---

## 8. Coordinate Frames

### World Frame
- Origin: Gazebo world origin (where drone spawns)
- X axis: forward (East)
- Y axis: left (North)
- Z axis: up
- Used by: ground truth odometry, force application

### Body Frame (cf_base_link)
- Origin: drone center of mass
- X axis: drone forward
- Y axis: drone left
- Z axis: drone up
- Used by: torque application, IMU measurements

### Euler Angle Convention
ZYX (yaw-pitch-roll) convention:
- φ (phi) = roll — rotation around X axis
- θ (theta) = pitch — rotation around Y axis
- ψ (psi) = yaw — rotation around Z axis

All angles in radians internally. Converted to degrees only for plotting.

### Quaternion Convention
ROS2 standard: `[x, y, z, w]` (scalar last)
Converted to euler using: `scipy.spatial.transform.Rotation.from_quat([x,y,z,w]).as_euler('ZYX')`

---

## 9. Plug-and-Play Controller System

### How It Works

The controller system uses a Python dictionary called `CONTROLLER_REGISTRY` in `cf_controller_node.py`:

```python
CONTROLLER_REGISTRY = {
    'cascaded_pid':       CascadedPID,
    'conventional_smc':   ConventionalSMC,    # (add when implemented)
    'super_twisting_smc': SuperTwistingSMC,   # (add when implemented)
    'nst_smc':            NonsingularSTSMC,   # (add when implemented)
}
```

At startup, the node reads the `active_controller` parameter from the YAML file, looks up the corresponding class in the registry, and instantiates it. The node never imports or references a specific controller directly — it only calls the abstract interface methods.

### The Interface Contract

Every controller must inherit from `BaseController` and implement exactly two methods:

```python
def compute_control(self,
                    state: np.ndarray,      # shape (12,)
                    reference: np.ndarray,   # shape (12,)
                    dt: float
                   ) -> np.ndarray:          # shape (4,) = [u1, u2, u3, u4]

def reset(self):
    # Reset all internal states (integrators, filters, etc.)
```

The state vector layout (same for both state and reference):
```
index 0:  x       [m]
index 1:  y       [m]
index 2:  z       [m]
index 3:  phi     [rad]
index 4:  theta   [rad]
index 5:  psi     [rad]
index 6:  x_dot   [m/s]
index 7:  y_dot   [m/s]
index 8:  z_dot   [m/s]
index 9:  phi_dot [rad/s]
index 10: theta_dot [rad/s]
index 11: psi_dot [rad/s]
```

The output vector:
```
index 0: u1 = total thrust     [N]     range: [0, 1.3]
index 1: u2 = roll torque      [N.m]   range: [-0.002, 0.002]
index 2: u3 = pitch torque     [N.m]   range: [-0.002, 0.002]
index 3: u4 = yaw torque       [N.m]   range: [-0.001, 0.001]
```

### Switching Controllers

Only one line in `controller_params.yaml` needs to change:
```yaml
active_controller: "cascaded_pid"   # change to: conventional_smc
```
No code changes needed anywhere.

---

## 10. Plug-and-Play Trajectory System

Identical pattern to controllers. The `TRAJECTORY_REGISTRY` in `cf_controller_node.py`:

```python
TRAJECTORY_REGISTRY = {
    'figure8': Figure8Trajectory,
    'circle':  CircleTrajectory,   # (add when implemented)
    'helix':   HelixTrajectory,    # (add when implemented)
    'hover':   HoverTrajectory,    # (add when implemented)
}
```

Every trajectory must implement:
```python
def get_reference(self, t: float) -> np.ndarray:  # shape (12,)
    # Returns full reference state at time t
```

Switching trajectories — one line in YAML:
```yaml
active_trajectory: "figure8"   # change to: circle
```

---

## 11. Adding a New Controller (Step-by-Step Checklist)

Example: Adding Conventional SMC

### Step 1 — Create the controller file
**Location:** `~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/controllers/conventional_smc.py`

**Template:**
```python
"""
conventional_smc.py
====================
Conventional Sliding Mode Controller.
Translated from MATLAB: [your_matlab_filename.m]
"""
import numpy as np
from cf_trajectory_controller.controllers.base_controller import BaseController
from cf_trajectory_controller.core.cf21_parameters import (
    MASS, GRAVITY, U1_MAX, U1_MIN,
    U2_MAX, U2_MIN, U3_MAX, U3_MIN, U4_MAX, U4_MIN
)

class ConventionalSMC(BaseController):

    def __init__(self, params: dict):
        super().__init__(params)
        # Load your gains from params dict
        # e.g. self.lambda_x = params.get('lambda_x', 2.0)
        self.reset()

    def reset(self):
        # Reset any internal states
        pass

    def compute_control(self, state, reference, dt):
        # Your exact MATLAB equations here
        # Return np.array([u1, u2, u3, u4])
        pass
```

### Step 2 — Add gains to YAML config
**File:** `~/crazyflie_control_ws/src/cf_trajectory_controller/config/controller_params.yaml`

Add a new section at the bottom:
```yaml
    conventional_smc:
      lambda_x: 2.0
      lambda_y: 2.0
      lambda_z: 4.0
      k_x: 5.0
      # ... all your gains
```

### Step 3 — Add gain names to node's loader
**File:** `~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/nodes/cf_controller_node.py`

In `_load_controller_params()`, add your gain names to `gain_names` list:
```python
gain_names = [
    'Kp_x', 'Ki_x', 'Kd_x',   # existing PID gains
    # ... existing gains ...
    'lambda_x', 'lambda_y', 'lambda_z',  # ADD your SMC gains
    'k_x', 'k_y', 'k_z',
]
```

### Step 4 — Register in the node
**File:** Same file, at the top imports section:
```python
from cf_trajectory_controller.controllers.conventional_smc import ConventionalSMC
```

And in the registry:
```python
CONTROLLER_REGISTRY = {
    'cascaded_pid':     CascadedPID,
    'conventional_smc': ConventionalSMC,   # ADD THIS LINE
}
```

### Step 5 — Copy to build directory
```bash
cp ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/controllers/conventional_smc.py \
   ~/crazyflie_control_ws/build/cf_trajectory_controller/cf_trajectory_controller/controllers/conventional_smc.py
```

Also update the node file in build:
```bash
cp ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/nodes/cf_controller_node.py \
   ~/crazyflie_control_ws/build/cf_trajectory_controller/cf_trajectory_controller/nodes/cf_controller_node.py
```

### Step 6 — Switch to new controller
**File:** `controller_params.yaml`
```yaml
active_controller: "conventional_smc"
```

### Step 7 — Test
```bash
ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py
```

That's it. 7 steps, no changes to Gazebo, no changes to physics, no changes to trajectory.

---

## 12. Adding a New Trajectory (Step-by-Step Checklist)

Example: Adding a circular trajectory

### Step 1 — Create trajectory file
**Location:** `~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/trajectories/circle_trajectory.py`

```python
import numpy as np

class CircleTrajectory:
    def __init__(self, params: dict):
        self.radius = params.get('radius', 0.5)
        self.omega  = params.get('omega',  0.4)
        self.z_const = params.get('z_const', 0.5)

    def get_reference(self, t: float) -> np.ndarray:
        x_des     = self.radius * np.cos(self.omega * t)
        y_des     = self.radius * np.sin(self.omega * t)
        x_dot_des = -self.radius * self.omega * np.sin(self.omega * t)
        y_dot_des =  self.radius * self.omega * np.cos(self.omega * t)
        return np.array([
            x_des, y_des, self.z_const,
            0.0, 0.0, 0.0,
            x_dot_des, y_dot_des, 0.0,
            0.0, 0.0, 0.0
        ])
```

### Step 2 — Add params to YAML
```yaml
    circle:
      radius: 0.5
      omega: 0.4
      z_const: 0.5
```

### Step 3 — Add param names to node loader
In `_load_trajectory_params()`:
```python
traj_param_names = [
    'amplitude', 'omega_x', 'omega_y', 'z_const', 'psi_des',  # figure8
    'radius', 'omega',   # ADD for circle
]
```

### Step 4 — Register and import in node
```python
from cf_trajectory_controller.trajectories.circle_trajectory import CircleTrajectory

TRAJECTORY_REGISTRY = {
    'figure8': Figure8Trajectory,
    'circle':  CircleTrajectory,   # ADD
}
```

### Step 5 — Copy to build + switch YAML
Same pattern as controller steps 5 and 6.

---

## 13. Configuration Reference

### controller_params.yaml — Complete Reference

```yaml
cf_controller_node:
  ros__parameters:

    # Which controller to use (must match key in CONTROLLER_REGISTRY)
    active_controller: "cascaded_pid"

    # Which trajectory to use (must match key in TRAJECTORY_REGISTRY)
    active_trajectory: "figure8"

    # Control loop rate in Hz
    control_frequency: 100.0

    # Cascaded PID gains
    cascaded_pid:
      Kp_x: 4.0    # Position proportional gain X
      Ki_x: 0.1    # Position integral gain X
      Kd_x: 4.0    # Position derivative gain X
      Kp_y: 4.0
      Ki_y: 0.1
      Kd_y: 4.0
      Kp_z: 15.0
      Ki_z: 2.0
      Kd_z: 8.0
      Kp_phi:    0.004    # Attitude proportional gain roll
      Ki_phi:    0.0001
      Kd_phi:    0.0006
      Kp_theta:  0.004
      Ki_theta:  0.0001
      Kd_theta:  0.0006
      Kp_psi:    0.002
      Ki_psi:    0.00005
      Kd_psi:    0.0005

    # Figure-8 trajectory parameters
    figure8:
      amplitude: 0.5    # meters (MATLAB original: 2.0m)
      omega_x:   0.4    # rad/s — X frequency
      omega_y:   0.8    # rad/s — Y frequency (= 2 * omega_x)
      z_const:   0.5    # meters constant altitude
      psi_des:   0.0    # radians desired yaw
```

### activate_crazyflie_ws.sh — Environment Variables Set

| Variable | Value | Used by |
|---|---|---|
| `ROS_DISTRO` | humble | ROS2 tools |
| `CRAZYFLIE_WS` | ~/crazyflie_control_ws | Scripts |
| `CRAZYFLIE_SRC` | ~/crazyflie_control_ws/src | Imports |
| `GAZEBO_MODEL_PATH` | .../models | Gazebo model loading |
| `GAZEBO_PLUGIN_PATH` | .../lib | cf_motor_plugin.so loading |
| `PYTHONPATH` | venv/site-packages prepended | Correct library versions |
| `VIRTUAL_ENV` | cf_control_venv | venv active |

