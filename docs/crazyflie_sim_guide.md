# Sim-to-Real Pipeline for Quadrotor Trajectory Tracking
## Complete Replication Guide

**Platform:** Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11
**Vehicle:** Crazyflie 2.1 Nano-Quadrotor
**Date:** April 2026

---

## Table of Contents

1. [Quick Start Guide](#quick-start-guide)
2. [Phase 0: Software Stack](#phase-0-software-stack)
3. [Phase 1: Workspace and Environment Setup](#phase-1-workspace-and-environment-setup)
4. [Phase 2: Crazyswarm2 Installation](#phase-2-crazyswarm2-installation)
5. [Phase 3: Gazebo Simulation Package](#phase-3-gazebo-simulation-package)
6. [Phase 4: Control Package](#phase-4-control-package)
7. [Phase 5: Launch File and Plotter](#phase-5-launch-file-and-plotter)
8. [Architecture Reference](#architecture-reference)
9. [Adding a New Controller](#adding-a-new-controller)
10. [Adding a New Trajectory](#adding-a-new-trajectory)
11. [Troubleshooting](#troubleshooting)
12. [Performance Benchmarks](#performance-benchmarks)

---

## Quick Start Guide

If you have already completed the full installation and just want to run the simulation, use these commands in 3 separate terminals.

### Terminal 1 — Launch Full Simulation

**Active directory: ~**
```bash
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py
```

### Terminal 2 — Run Plotter (after 10 seconds)

**Active directory: ~**
```bash
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
ros2 run cf_trajectory_controller cf_trajectory_plotter
```

### Switch Controller or Trajectory

Edit one line in the config file:
```bash
nano ~/crazyflie_control_ws/src/cf_trajectory_controller/config/controller_params.yaml
```

```yaml
active_controller: "cascaded_pid"   # options: cascaded_pid | conventional_smc
active_trajectory: "figure8"        # options: figure8 | circle | helix | hover
```

> **NOTE:** Plots are saved to `~/crazyflie_control_ws/logs/` after 30 seconds of recording.

---

## Phase 0: Software Stack

The following software versions are required. Do not mix versions — this exact combination has been validated.

| Software | Version | Purpose |
|---|---|---|
| Ubuntu | 22.04 LTS (Jammy) | Base operating system |
| ROS2 | Humble Hawksbill (LTS) | Robotics middleware |
| Python | 3.10.x (system) | Ships with Ubuntu 22.04 |
| Gazebo Classic | 11.10.x | Physics simulation |
| gazebo_ros_pkgs | Humble compatible | ROS2-Gazebo bridge |
| crazyswarm2 | main branch | Crazyflie ROS2 stack |
| numpy | 1.26.4 | Matrix operations |
| scipy | 1.13.0 | Rotation, ODE solvers |
| cflib | 0.1.26 | Crazyflie hardware library |

---

## Phase 1: Workspace and Environment Setup

All commands in this phase are run from the home directory (`~`) unless stated otherwise.

### Step 1.1 — Create Directory Structure

**Active directory: `~`**
```bash
mkdir -p ~/crazyflie_control_ws/{src,docs,scripts,configs,logs}
```

### Step 1.2 — Create Python Virtual Environment

**Active directory: `~`**
```bash
cd ~/crazyflie_control_ws
python3 -m venv cf_control_venv --system-site-packages
```

> **NOTE:** `--system-site-packages` allows the venv to see ROS2 Python packages (like `rclpy`) while keeping your control libraries isolated.

### ✅ Verification Step 1.2

```bash
ls ~/crazyflie_control_ws/
~/crazyflie_control_ws/cf_control_venv/bin/python3 --version
```

Expected output:
```
cf_control_venv  configs  docs  logs  scripts  src
Python 3.10.12
```

### Step 1.3 — Install Control Libraries

**Active directory: `~/crazyflie_control_ws` (venv active)**
```bash
source ~/crazyflie_control_ws/cf_control_venv/bin/activate
pip install --upgrade pip
pip install \
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
```

### Step 1.4 — Fix mpl_toolkits Namespace Issue

**Active directory: `~/crazyflie_control_ws` (venv active)**

ROS2 injects system Python paths before the venv, causing `mpl_toolkits` to load from the wrong location. Fix this by creating an `__init__.py`:

```bash
cat > ~/crazyflie_control_ws/cf_control_venv/lib/python3.10/site-packages/mpl_toolkits/__init__.py << 'EOF'
# This file makes mpl_toolkits a regular package inside the venv,
# preventing Python from merging it with the system mpl_toolkits namespace.
EOF
```

### Step 1.5 — Create Workspace Activation Script

**Active directory: `~/crazyflie_control_ws`**
```bash
cat > ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh << 'EOF'
#!/bin/bash
# =============================================================================
# Crazyflie Control Workspace Activation Script
# Usage: source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
# =============================================================================

WORKSPACE_ROOT="$HOME/crazyflie_control_ws"

# Source ROS2 Humble
source /opt/ros/humble/setup.bash
echo "[1/4] ROS2 Humble sourced"

# Source the ROS2 workspace (once packages are built)
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "[2/4] Crazyflie ROS2 workspace sourced"
else
    echo "[2/4] ROS2 workspace not built yet (skip — expected in early phases)"
fi

# Activate Python venv
source "$WORKSPACE_ROOT/cf_control_venv/bin/activate"
echo "[3/4] Python venv cf_control_venv activated"

# Set project environment variables
export CRAZYFLIE_WS="$WORKSPACE_ROOT"
export CRAZYFLIE_SRC="$WORKSPACE_ROOT/src"
export CRAZYFLIE_CONFIGS="$WORKSPACE_ROOT/configs"
export CRAZYFLIE_LOGS="$WORKSPACE_ROOT/logs"
export GAZEBO_MODEL_PATH="$WORKSPACE_ROOT/src/crazyflie_gazebo_sim/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_ROOT/install/crazyflie_gazebo_sim/lib:$GAZEBO_PLUGIN_PATH"
export PYTHONPATH="$WORKSPACE_ROOT/src:$PYTHONPATH"

# Force venv site-packages to front of PYTHONPATH
VENV_SITE="$WORKSPACE_ROOT/cf_control_venv/lib/python3.10/site-packages"
export PYTHONPATH="$VENV_SITE:$PYTHONPATH"

echo "[4/4] Environment variables set"
echo ""
echo "=============================================="
echo "  Crazyflie Control Workspace Ready"
echo "  ROS2 Distro  : $ROS_DISTRO"
echo "  Python        : $(python3 --version)"
echo "  Workspace     : $CRAZYFLIE_WS"
echo "=============================================="
EOF

chmod +x ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
```

### Step 1.6 — Add COLCON_IGNORE to venv

**Active directory: `~/crazyflie_control_ws`**

This prevents colcon from scanning thousands of numpy files inside the venv:
```bash
touch ~/crazyflie_control_ws/cf_control_venv/COLCON_IGNORE
```

### ✅ Verification Step 1 (Final)

```bash
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
echo $CRAZYFLIE_WS
echo $ROS_DISTRO
echo $VIRTUAL_ENV
```

Expected:
```
/home/yashu/crazyflie_control_ws
humble
/home/yashu/crazyflie_control_ws/cf_control_venv
```

---

## Phase 2: Crazyswarm2 Installation

Crazyswarm2 provides the Crazyflie ROS2 interface — message types, services, and the hardware driver. We only use the interface definitions for simulation.

**Reference:** [IMRCLab/crazyswarm2](https://github.com/IMRCLab/crazyswarm2)

### Step 2.1 — Install System Dependencies

**Active directory: `~/crazyflie_control_ws` (venv active)**
```bash
sudo apt-get update
sudo apt-get install -y \
    git \
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
    ros-humble-plotjuggler-ros \
    libgazebo-dev \
    ros-humble-gazebo-dev
```

### Step 2.2 — Clone Crazyswarm2

**Active directory: `~/crazyflie_control_ws/src`**
```bash
cd ~/crazyflie_control_ws/src
git clone --branch main --recurse-submodules \
    https://github.com/IMRCLab/crazyswarm2.git
git clone --branch main \
    https://github.com/IMRCLab/motion_capture_tracking.git
```

### Step 2.3 — Install Python Dependencies

**Active directory: `~/crazyflie_control_ws` (venv active)**
```bash
pip install cflib==0.1.26 vispy==0.14.3 nicegui==1.4.22 importlib-metadata==7.2.1
```

### Step 2.4 — Initialize rosdep and Install Dependencies

**Active directory: `~/crazyflie_control_ws`**
```bash
cd ~/crazyflie_control_ws
sudo rosdep init 2>/dev/null || echo "rosdep already initialized"
rosdep update
rosdep install --from-paths src/crazyswarm2 --ignore-src -r -y
```

### Step 2.5 — Build Crazyswarm2 Packages

**Active directory: `~/crazyflie_control_ws`**
```bash
colcon build --symlink-install \
    --packages-select \
        crazyflie_interfaces \
        crazyflie \
        crazyflie_py \
        crazyflie_sim \
        crazyflie_examples \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tee ~/crazyflie_control_ws/logs/build_crazyswarm2.log
```

### ✅ Verification Step 2

```bash
source ~/crazyflie_control_ws/install/setup.bash
ros2 pkg list | grep crazyflie
```

Expected:
```
crazyflie
crazyflie_examples
crazyflie_interfaces
crazyflie_py
crazyflie_sim
```

```bash
ros2 interface list | grep crazyflie | head -5
# Expected: crazyflie_interfaces/msg/FullState (and others)
```

---

## Phase 3: Gazebo Simulation Package

This phase creates the `crazyflie_gazebo_sim` ROS2 package — the Gazebo world, drone model, and the C++ physics plugin that applies forces.

### Step 3.1 — Create Package Structure

**Active directory: `~/crazyflie_control_ws/src`**
```bash
cd ~/crazyflie_control_ws/src
ros2 pkg create crazyflie_gazebo_sim \
    --build-type ament_cmake \
    --dependencies rclpy rclcpp gazebo_ros gazebo_plugins \
                   robot_state_publisher xacro

mkdir -p ~/crazyflie_control_ws/src/crazyflie_gazebo_sim/{models/crazyflie_2_1/meshes,worlds,launch,config,urdf,src}

# Copy drone mesh from crazyswarm2
cp ~/crazyflie_control_ws/src/crazyswarm2/crazyflie/urdf/cf2_assembly_with_props.dae \
   ~/crazyflie_control_ws/src/crazyflie_gazebo_sim/models/crazyflie_2_1/meshes/
```

### Step 3.2 — Create model.config

**File:** `src/crazyflie_gazebo_sim/models/crazyflie_2_1/model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>Crazyflie 2.1</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Crazyflie Control WS</name>
  </author>
  <description>
    Bitcraze Crazyflie 2.1 nano-quadrotor. Mass: 34g, Arm: 46mm.
    Reference: Silano et al., CrazyS, ROBOT 2017 (arXiv:1811.03557)
  </description>
</model>
```

### Step 3.3 — Create model.sdf (Drone Physics Model)

**File:** `src/crazyflie_gazebo_sim/models/crazyflie_2_1/model.sdf`

Physical parameters sourced from IMRCLab/crazyswarm2 `crazyflie2.urdf` and `crazyflie2.yaml`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="crazyflie_2_1">

    <static>false</static>
    <pose>0 0 0.015 0 0 0</pose>

    <!-- Base link: mass=0.034kg, inertia from crazyflie2.urdf -->
    <link name="cf_base_link">
      <inertial>
        <mass>0.034</mass>
        <inertia>
          <ixx>16.571710e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>16.655602e-6</iyy><iyz>0</iyz>
          <izz>29.261652e-6</izz>
        </inertia>
      </inertial>

      <visual name="cf_body_visual">
        <geometry>
          <mesh>
            <uri>model://crazyflie_2_1/meshes/cf2_assembly_with_props.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>

      <collision name="cf_body_collision">
        <geometry><box><size>0.092 0.092 0.029</size></box></geometry>
      </collision>

      <sensor name="cf_imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>500</update_rate>
        <plugin name="cf_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <namespace>/cf1</namespace>
            <remapping>~/out:=imu</remapping>
          </ros>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
        </plugin>
      </sensor>
    </link>

    <!-- Motor 1: Front-Left (CCW) — arm_length/sqrt(2) = 0.03254m -->
    <link name="cf_rotor_m1">
      <pose>0.03254 0.03254 0.012 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia><ixx>9.75e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>9.75e-7</iyy><iyz>0</iyz><izz>1.66704e-6</izz></inertia>
      </inertial>
      <visual name="cf_rotor_m1_visual">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </visual>
      <collision name="cf_rotor_m1_collision">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </collision>
    </link>

    <!-- Motor 2: Front-Right (CW) -->
    <link name="cf_rotor_m2">
      <pose>0.03254 -0.03254 0.012 0 0 0</pose>
      <inertial><mass>0.001</mass>
        <inertia><ixx>9.75e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>9.75e-7</iyy><iyz>0</iyz><izz>1.66704e-6</izz></inertia>
      </inertial>
      <visual name="cf_rotor_m2_visual">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </visual>
      <collision name="cf_rotor_m2_collision">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </collision>
    </link>

    <!-- Motor 3: Rear-Right (CCW) -->
    <link name="cf_rotor_m3">
      <pose>-0.03254 -0.03254 0.012 0 0 0</pose>
      <inertial><mass>0.001</mass>
        <inertia><ixx>9.75e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>9.75e-7</iyy><iyz>0</iyz><izz>1.66704e-6</izz></inertia>
      </inertial>
      <visual name="cf_rotor_m3_visual">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </visual>
      <collision name="cf_rotor_m3_collision">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </collision>
    </link>

    <!-- Motor 4: Rear-Left (CW) -->
    <link name="cf_rotor_m4">
      <pose>-0.03254 0.03254 0.012 0 0 0</pose>
      <inertial><mass>0.001</mass>
        <inertia><ixx>9.75e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>9.75e-7</iyy><iyz>0</iyz><izz>1.66704e-6</izz></inertia>
      </inertial>
      <visual name="cf_rotor_m4_visual">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </visual>
      <collision name="cf_rotor_m4_collision">
        <geometry><cylinder><radius>0.023</radius><length>0.001</length></cylinder></geometry>
      </collision>
    </link>

    <joint name="cf_rotor_m1_joint" type="revolute">
      <parent>cf_base_link</parent><child>cf_rotor_m1</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>
    <joint name="cf_rotor_m2_joint" type="revolute">
      <parent>cf_base_link</parent><child>cf_rotor_m2</child>
      <axis><xyz>0 0 -1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>
    <joint name="cf_rotor_m3_joint" type="revolute">
      <parent>cf_base_link</parent><child>cf_rotor_m3</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>
    <joint name="cf_rotor_m4_joint" type="revolute">
      <parent>cf_base_link</parent><child>cf_rotor_m4</child>
      <axis><xyz>0 0 -1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>

    <!-- Ground truth odometry plugin -->
    <plugin name="cf_ground_truth_plugin" filename="libgazebo_ros_p3d.so">
      <ros>
        <namespace>/cf1</namespace>
        <remapping>odom:=ground_truth/odom</remapping>
      </ros>
      <body_name>cf_base_link</body_name>
      <frame_name>world</frame_name>
      <update_rate>100.0</update_rate>
      <xyz_offset>0 0 0</xyz_offset>
      <rpy_offset>0 0 0</rpy_offset>
      <gaussian_noise>0.0</gaussian_noise>
    </plugin>

    <!-- Motor force/torque plugin (our custom C++ plugin) -->
    <plugin name="cf_motor_plugin" filename="libcf_motor_plugin.so">
    </plugin>

  </model>
</sdf>
```

### Step 3.4 — Create the Gazebo World File

**File:** `src/crazyflie_gazebo_sim/worlds/crazyflie_trajectory_world.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="crazyflie_trajectory_world">

    <!-- Physics tuned for nano-quadrotor (NO gravity tag — causes engine error) -->
    <physics name="cf_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver><type>quick</type><iters>100</iters><sor>1.3</sor></solver>
        <constraints>
          <cfm>0.0</cfm><erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <light name="cf_lab_ceiling_light" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>0 0 -1</direction>
    </light>

    <include><uri>model://ground_plane</uri></include>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen="0">
      <camera name="cf_lab_camera">
        <pose>-2.0 -2.0 1.5 0 0.35 0.785</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```

### Step 3.5 — Create the C++ Motor Plugin

**File:** `src/crazyflie_gazebo_sim/src/cf_motor_plugin.cpp`

This is the most critical file. It runs inside Gazebo and applies forces/torques directly to the physics engine every simulation step.

```cpp
/*
 * cf_motor_plugin.cpp
 * ====================
 * Gazebo Classic model plugin for Crazyflie 2.1.
 * Subscribes to /cf1/control_debug [u1,u2,u3,u4,...]
 * Applies thrust u1 along body Z and torques u2,u3,u4 in physics loop.
 *
 * This bypasses the broken apply_link_wrench service in Gazebo Classic 11 + ROS2.
 * Reference: rotors_simulator (ethz-asl/rotors_simulator)
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{

class CrazyflieMotorPlugin : public ModelPlugin
{
public:
  CrazyflieMotorPlugin() : ModelPlugin(), u1_(0), u2_(0), u3_(0), u4_(0) {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();

    link_ = model->GetLink("cf_base_link");
    if (!link_) {
      gzerr << "[CrazyflieMotorPlugin] Link 'cf_base_link' not found!\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);

    ctrl_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cf1/control_debug",
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
          std::lock_guard<std::mutex> lock(mutex_);
          u1_ = msg->data[0];
          u2_ = msg->data[1];
          u3_ = msg->data[2];
          u4_ = msg->data[3];
        }
      }
    );

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrazyflieMotorPlugin::OnUpdate, this));

    gzmsg << "[CrazyflieMotorPlugin] Loaded. Subscribed to /cf1/control_debug\n";
  }

  void OnUpdate()
  {
    double u1, u2, u3, u4;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      u1 = u1_; u2 = u2_; u3 = u3_; u4 = u4_;
    }

    // Get current body orientation
    ignition::math::Quaterniond q = link_->WorldPose().Rot();

    // Rotate thrust from body Z to world frame
    ignition::math::Vector3d thrust_body(0.0, 0.0, u1);
    ignition::math::Vector3d thrust_world = q * thrust_body;

    // Apply force in world frame
    link_->AddForce(thrust_world);

    // Apply torques in body frame
    link_->AddRelativeTorque(ignition::math::Vector3d(u2, u3, u4));
  }

private:
  physics::ModelPtr   model_;
  physics::WorldPtr   world_;
  physics::LinkPtr    link_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ctrl_sub_;
  std::mutex mutex_;
  double u1_, u2_, u3_, u4_;
};

GZ_REGISTER_MODEL_PLUGIN(CrazyflieMotorPlugin)

} // namespace gazebo
```

### Step 3.6 — Create CMakeLists.txt

**File:** `src/crazyflie_gazebo_sim/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(crazyflie_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

# Crazyflie Motor Plugin
add_library(cf_motor_plugin SHARED src/cf_motor_plugin.cpp)
target_include_directories(cf_motor_plugin PUBLIC include ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(cf_motor_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(cf_motor_plugin rclcpp gazebo_ros std_msgs)
install(TARGETS cf_motor_plugin LIBRARY DESTINATION lib)

# Install all directories
install(DIRECTORY launch worlds models urdf config scripts
        DESTINATION share/${PROJECT_NAME})

ament_package()
```

### Step 3.7 — Create package.xml

**File:** `src/crazyflie_gazebo_sim/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>crazyflie_gazebo_sim</name>
  <version>0.1.0</version>
  <description>
    Gazebo Classic 11 simulation environment for Crazyflie 2.1.
    Part of the Sim-to-Real Pipeline for Quadrotor Trajectory Tracking project.
  </description>
  <maintainer email="your@email.com">Yashu</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>gazebo_ros</depend>
  <depend>gazebo_plugins</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>
  <depend>crazyflie_interfaces</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

### Step 3.8 — Create the Gazebo Launch File

**File:** `src/crazyflie_gazebo_sim/launch/gazebo_cf1_sim.launch.py`

```python
"""
gazebo_cf1_sim.launch.py
Launches Gazebo + spawns Crazyflie model.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cf_gazebo_sim_pkg = get_package_share_directory('crazyflie_gazebo_sim')
    gazebo_ros_pkg     = get_package_share_directory('gazebo_ros')

    world_file  = os.path.join(cf_gazebo_sim_pkg, 'worlds',
                               'crazyflie_trajectory_world.world')
    xacro_file  = os.path.join(cf_gazebo_sim_pkg, 'urdf',
                               'crazyflie_2_1_gazebo.urdf.xacro')
    model_path  = os.path.join(cf_gazebo_sim_pkg, 'models')

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='cf1')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.05')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    gui = LaunchConfiguration('gui')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=model_path)

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' robot_name:=', robot_name,
                 ' initial_x:=', x,
                 ' initial_y:=', y,
                 ' initial_z:=', z]),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cf1_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}])

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true',
                          'pause': 'false'}.items())

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    spawn_cf1_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='cf1_spawner',
        output='screen',
        arguments=['-entity', 'cf1',
                   '-file', os.path.join(model_path, 'crazyflie_2_1', 'model.sdf'),
                   '-robot_namespace', 'cf1',
                   '-x', x, '-y', y, '-z', z,
                   '-R', '0', '-P', '0', '-Y', '0'])

    return LaunchDescription([
        set_gazebo_model_path,
        robot_name_arg, x_arg, y_arg, z_arg, gui_arg,
        gazebo_server, gazebo_client,
        robot_state_publisher_node, spawn_cf1_node])
```

### Step 3.9 — Build the Gazebo Package

**Active directory: `~/crazyflie_control_ws`**
```bash
cd ~/crazyflie_control_ws
colcon build --symlink-install \
    --packages-select crazyflie_gazebo_sim \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tee ~/crazyflie_control_ws/logs/build_gazebo_sim.log

source ~/crazyflie_control_ws/install/setup.bash
```

### ✅ Verification Step 3

```bash
# Check package built
ros2 pkg list | grep crazyflie_gazebo_sim

# Check plugin compiled
find ~/crazyflie_control_ws/install -name 'libcf_motor_plugin.so'
# Expected: .../install/crazyflie_gazebo_sim/lib/libcf_motor_plugin.so

# Launch Gazebo
source ~/crazyflie_control_ws/scripts/activate_crazyflie_ws.sh
ros2 launch crazyflie_gazebo_sim gazebo_cf1_sim.launch.py gui:=true
# Expected terminal output includes:
# [Msg] [CrazyflieMotorPlugin] Loaded. Subscribed to /cf1/control_debug
```

---

## Phase 4: Control Package

This phase creates the `cf_trajectory_controller` package — all control logic, trajectories, and the main ROS2 node.

### Step 4.1 — Create Package Structure

**Active directory: `~/crazyflie_control_ws/src`**
```bash
cd ~/crazyflie_control_ws/src
ros2 pkg create cf_trajectory_controller \
    --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs crazyflie_interfaces

mkdir -p ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/{core,controllers,trajectories,nodes,utils}

touch ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/core/__init__.py
touch ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/controllers/__init__.py
touch ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/trajectories/__init__.py
touch ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/nodes/__init__.py
touch ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/utils/__init__.py

mkdir -p ~/crazyflie_control_ws/src/cf_trajectory_controller/{config,launch}
```

### Step 4.2 — File Overview

| File | Purpose |
|---|---|
| `core/cf21_parameters.py` | ALL physical constants — mass, inertia, control limits |
| `controllers/base_controller.py` | Abstract interface — `compute_control()` and `reset()` |
| `controllers/cascaded_pid.py` | Cascaded PID translated from MATLAB `PID_3D_05.m` |
| `trajectories/figure8_trajectory.py` | Figure-8 reference trajectory generator |
| `nodes/cf_controller_node.py` | Main ROS2 node — orchestrates everything |
| `utils/cf_trajectory_plotter.py` | Data recording and 10-plot generation |
| `config/controller_params.yaml` | ALL tunable gains — only file to edit for tuning |
| `launch/cf1_full_simulation.launch.py` | Single-command full simulation launcher |

### Step 4.3 — core/cf21_parameters.py

**File:** `src/cf_trajectory_controller/cf_trajectory_controller/core/cf21_parameters.py`

```python
"""
cf21_parameters.py
==================
Single source of truth for all Crazyflie 2.1 physical parameters.
To use different vehicle: create a new file and change the import.

Physical parameters sourced from:
  - IMRCLab/crazyswarm2: crazyflie2.urdf, crazyflie2.yaml
  - Bitcraze Crazyflie 2.1 datasheet
"""
import math

# Vehicle Physical Parameters
MASS    = 0.034          # kg
GRAVITY = 9.81           # m/s^2
IXX     = 16.571710e-6   # kg.m^2
IYY     = 16.655602e-6   # kg.m^2
IZZ     = 29.261652e-6   # kg.m^2
ARM_LENGTH        = 0.046   # m
THRUST_TO_TORQUE  = 0.006   # dimensionless
MAX_THRUST_TOTAL  = 1.3     # N (all 4 motors)
MAX_THRUST_PER_MOTOR = MAX_THRUST_TOTAL / 4.0

# Control Output Limits (physics-based)
# U2_MAX = F_max * arm_length * sqrt(2) = 0.325 * 0.046 * 1.414 = 0.021 N.m
U1_MAX =  1.3;    U1_MIN = 0.0
U2_MAX =  0.002;  U2_MIN = -0.002   # Roll torque [N.m]
U3_MAX =  0.002;  U3_MIN = -0.002   # Pitch torque [N.m]
U4_MAX =  0.001;  U4_MIN = -0.001   # Yaw torque [N.m]

# Angle Limits [radians]
PHI_MAX   = math.radians(30)
THETA_MAX = math.radians(30)
PSI_MAX   = math.radians(180)

# Control Loop
CONTROL_FREQUENCY = 100.0   # Hz
DT = 1.0 / CONTROL_FREQUENCY
```

### Step 4.4 — controllers/base_controller.py

**File:** `src/cf_trajectory_controller/cf_trajectory_controller/controllers/base_controller.py`

```python
"""
base_controller.py
==================
Abstract base class that every controller must implement.
The ROS2 node only ever calls compute_control() and reset().
"""
from abc import ABC, abstractmethod
import numpy as np


class BaseController(ABC):

    def __init__(self, params: dict):
        self.params = params
        self._is_initialized = False

    @abstractmethod
    def compute_control(self,
                        state: np.ndarray,      # shape (12,)
                        reference: np.ndarray,  # shape (12,)
                        dt: float
                       ) -> np.ndarray:          # shape (4,) = [u1,u2,u3,u4]
        pass

    @abstractmethod
    def reset(self):
        """Reset all internal states (integrators, filters, etc.)"""
        pass

    def get_controller_name(self) -> str:
        return self.__class__.__name__
```

### Step 4.5 — controllers/cascaded_pid.py

**File:** `src/cf_trajectory_controller/cf_trajectory_controller/controllers/cascaded_pid.py`

```python
"""
cascaded_pid.py
================
Cascaded PID controller for Crazyflie 2.1 trajectory tracking.
Translated exactly from MATLAB script: PID_3D_05.m

Outer loop: Position -> Desired attitude
  x_ddot_cmd = Kp_x*ex + Kd_x*ex_dot + Ki_x*int_ex    [MATLAB line 71]
  phi_des    = -(1/g) * y_ddot_cmd                      [MATLAB line 76]
  theta_des  =  (1/g) * x_ddot_cmd                      [MATLAB line 77]
  u1         = m * (g + z_ddot_cmd)                     [MATLAB line 86]

Inner loop: Attitude -> Torques
  u2 = Kp_phi*(phi_des-phi) + Kd_phi*(0-phi_dot) + Ki_phi*int_ephi
"""
import numpy as np
from cf_trajectory_controller.controllers.base_controller import BaseController
from cf_trajectory_controller.core.cf21_parameters import (
    MASS, GRAVITY, PHI_MAX, THETA_MAX, PSI_MAX,
    U1_MAX, U1_MIN, U2_MAX, U2_MIN, U3_MAX, U3_MIN, U4_MAX, U4_MIN
)


class CascadedPID(BaseController):

    def __init__(self, params: dict):
        super().__init__(params)

        # Outer loop gains (position) — tuned for CF2.1 at 100Hz
        self.Kp_x = params.get('Kp_x', 4.0)
        self.Ki_x = params.get('Ki_x', 0.1)
        self.Kd_x = params.get('Kd_x', 4.0)
        self.Kp_y = params.get('Kp_y', 4.0)
        self.Ki_y = params.get('Ki_y', 0.1)
        self.Kd_y = params.get('Kd_y', 4.0)
        self.Kp_z = params.get('Kp_z', 15.0)
        self.Ki_z = params.get('Ki_z', 2.0)
        self.Kd_z = params.get('Kd_z', 8.0)

        # Inner loop gains (attitude) — physics-based, IXX=16.5e-6 kg.m2
        self.Kp_phi   = params.get('Kp_phi',   0.004)
        self.Ki_phi   = params.get('Ki_phi',    0.0001)
        self.Kd_phi   = params.get('Kd_phi',    0.0006)
        self.Kp_theta = params.get('Kp_theta',  0.004)
        self.Ki_theta = params.get('Ki_theta',   0.0001)
        self.Kd_theta = params.get('Kd_theta',   0.0006)
        self.Kp_psi   = params.get('Kp_psi',    0.002)
        self.Ki_psi   = params.get('Ki_psi',    0.00005)
        self.Kd_psi   = params.get('Kd_psi',    0.0005)

        # Anti-windup limits
        self.int_limit_pos = 0.5   # [m]
        self.int_limit_att = 0.1   # [rad]

        self.reset()

    def reset(self):
        self.int_ex = self.int_ey = self.int_ez = 0.0
        self.int_ephi = self.int_etheta = self.int_epsi = 0.0
        self._is_initialized = True

    def compute_control(self, state, reference, dt):
        # Unpack state
        phi=state[3]; theta=state[4]; psi=state[5]
        x_dot=state[6]; y_dot=state[7]; z_dot=state[8]
        phi_dot=state[9]; theta_dot=state[10]; psi_dot=state[11]

        # Unpack reference
        x_des=reference[0]; y_des=reference[1]; z_des=reference[2]
        psi_des=reference[5]
        x_dot_des=reference[6]; y_dot_des=reference[7]; z_dot_des=reference[8]

        # Position errors
        e_x=x_des-state[0]; e_x_dot=x_dot_des-x_dot
        e_y=y_des-state[1]; e_y_dot=y_dot_des-y_dot
        e_z=z_des-state[2]; e_z_dot=z_dot_des-z_dot

        # Integral errors with anti-windup
        self.int_ex = np.clip(self.int_ex+e_x*dt, -self.int_limit_pos, self.int_limit_pos)
        self.int_ey = np.clip(self.int_ey+e_y*dt, -self.int_limit_pos, self.int_limit_pos)
        self.int_ez = np.clip(self.int_ez+e_z*dt, -self.int_limit_pos, self.int_limit_pos)

        # Outer loop: Position PID -> commanded accelerations [MATLAB lines 71-73]
        x_ddot_cmd = self.Kp_x*e_x + self.Kd_x*e_x_dot + self.Ki_x*self.int_ex
        y_ddot_cmd = self.Kp_y*e_y + self.Kd_y*e_y_dot + self.Ki_y*self.int_ey
        z_ddot_cmd = self.Kp_z*e_z + self.Kd_z*e_z_dot + self.Ki_z*self.int_ez

        # Desired roll and pitch [MATLAB lines 76-77]
        phi_des   = -(1.0/GRAVITY)*y_ddot_cmd
        theta_des =  (1.0/GRAVITY)*x_ddot_cmd
        phi_des   = np.clip(phi_des,   -PHI_MAX,   PHI_MAX)
        theta_des = np.clip(theta_des, -THETA_MAX, THETA_MAX)
        psi_des   = np.clip(psi_des,   -PSI_MAX,   PSI_MAX)

        # Attitude errors
        e_phi=phi_des-phi; e_theta=theta_des-theta; e_psi=psi_des-psi

        self.int_ephi   = np.clip(self.int_ephi+e_phi*dt,   -self.int_limit_att, self.int_limit_att)
        self.int_etheta = np.clip(self.int_etheta+e_theta*dt,-self.int_limit_att, self.int_limit_att)
        self.int_epsi   = np.clip(self.int_epsi+e_psi*dt,   -self.int_limit_att, self.int_limit_att)

        # Control inputs
        u1 = MASS*(GRAVITY+z_ddot_cmd)                                       # [MATLAB line 86]
        u2 = self.Kp_phi*e_phi   + self.Kd_phi*(0.0-phi_dot)   + self.Ki_phi*self.int_ephi
        u3 = self.Kp_theta*e_theta + self.Kd_theta*(0.0-theta_dot) + self.Ki_theta*self.int_etheta
        u4 = self.Kp_psi*e_psi   + self.Kd_psi*(0.0-psi_dot)   + self.Ki_psi*self.int_epsi

        # Saturation
        u1=np.clip(u1,U1_MIN,U1_MAX); u2=np.clip(u2,U2_MIN,U2_MAX)
        u3=np.clip(u3,U3_MIN,U3_MAX); u4=np.clip(u4,U4_MIN,U4_MAX)

        return np.array([u1, u2, u3, u4])
```

### Step 4.6 — trajectories/figure8_trajectory.py

**File:** `src/cf_trajectory_controller/cf_trajectory_controller/trajectories/figure8_trajectory.py`

```python
"""
figure8_trajectory.py
======================
Figure-8 trajectory. Translated from MATLAB PID_3D_05.m:
  x_des = A*sin(omega_x*t)    [MATLAB line 13]
  y_des = A*sin(omega_y*t)    [MATLAB line 14]  (omega_y = 2*omega_x)
  z_des = z_const             [MATLAB line 15]

Scaled for CF2.1: amplitude=0.5m (MATLAB: 2.0m), z=0.5m (MATLAB: 2.0m)
"""
import numpy as np


class Figure8Trajectory:

    def __init__(self, params: dict):
        self.A       = params.get('amplitude', 0.5)
        self.omega_x = params.get('omega_x',   0.4)
        self.omega_y = params.get('omega_y',   0.8)
        self.z_const = params.get('z_const',   0.5)
        self.psi_des = params.get('psi_des',   0.0)

    def get_reference(self, t: float) -> np.ndarray:
        x_des     = self.A * np.sin(self.omega_x * t)
        y_des     = self.A * np.sin(self.omega_y * t)
        x_dot_des = self.A * self.omega_x * np.cos(self.omega_x * t)
        y_dot_des = self.A * self.omega_y * np.cos(self.omega_y * t)

        return np.array([
            x_des, y_des, self.z_const,
            0.0, 0.0, self.psi_des,
            x_dot_des, y_dot_des, 0.0,
            0.0, 0.0, 0.0
        ])
```

### Step 4.7 — config/controller_params.yaml

**File:** `src/cf_trajectory_controller/config/controller_params.yaml`

```yaml
# =============================================================================
# Controller Parameters — Crazyflie 2.1 Sim-to-Real Pipeline
# To switch controllers: change active_controller only.
# To retune gains: edit values below — no Python code changes needed.
# =============================================================================

cf_controller_node:
  ros__parameters:

    active_controller: "cascaded_pid"   # OPTIONS: cascaded_pid | conventional_smc | ...
    active_trajectory: "figure8"        # OPTIONS: figure8 | circle | helix | hover
    control_frequency: 100.0            # Hz

    # =========================================================================
    # CASCADED PID GAINS — Crazyflie 2.1 (m=0.034 kg)
    # Tuned from MATLAB PID_3D_05.m (m=0.5kg) using physics-based scaling
    # Inner loop gains: Kp = Ixx * wn^2 = 16.57e-6 * 11.43^2 = 0.002164
    # =========================================================================
    cascaded_pid:
      # Outer loop — Position PID
      Kp_x: 4.0
      Ki_x: 0.1
      Kd_x: 4.0

      Kp_y: 4.0
      Ki_y: 0.1
      Kd_y: 4.0

      Kp_z: 15.0
      Ki_z: 2.0
      Kd_z: 8.0

      # Inner loop — Attitude PID (physics-correct for 34g vehicle)
      Kp_phi:    0.004
      Ki_phi:    0.0001
      Kd_phi:    0.0006

      Kp_theta:  0.004
      Ki_theta:  0.0001
      Kd_theta:  0.0006

      Kp_psi:    0.002
      Ki_psi:    0.00005
      Kd_psi:    0.0005

    # =========================================================================
    # FIGURE-8 TRAJECTORY
    # Scaled for CF2.1 indoor flight (original MATLAB: amplitude=2m, z=2m)
    # =========================================================================
    figure8:
      amplitude: 0.5      # [m]     MATLAB original: 2.0
      omega_x:   0.4      # [rad/s] same as MATLAB
      omega_y:   0.8      # [rad/s] same as MATLAB
      z_const:   0.5      # [m]     MATLAB original: 2.0
      psi_des:   0.0      # [rad]   same as MATLAB
```

### Step 4.8 — launch/cf1_full_simulation.launch.py

**File:** `src/cf_trajectory_controller/launch/cf1_full_simulation.launch.py`

```python
"""
cf1_full_simulation.launch.py
Starts: Gazebo + drone model + controller (5s delayed startup)

Usage:
  ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py
  ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py gui:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cf_gazebo_sim_pkg = get_package_share_directory('crazyflie_gazebo_sim')
    cf_controller_pkg = get_package_share_directory('cf_trajectory_controller')
    controller_config  = os.path.join(cf_controller_pkg, 'config', 'controller_params.yaml')

    controller_arg = DeclareLaunchArgument('controller', default_value='cascaded_pid')
    trajectory_arg = DeclareLaunchArgument('trajectory', default_value='figure8')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.05')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_gazebo_sim_pkg, 'launch', 'gazebo_cf1_sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z')}.items())

    controller_node = Node(
        package='cf_trajectory_controller',
        executable='cf_controller_node',
        name='cf_controller_node',
        output='screen',
        parameters=[
            controller_config,
            {'active_controller': LaunchConfiguration('controller'),
             'active_trajectory':  LaunchConfiguration('trajectory')}])

    delayed_controller = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[cf1_full_simulation] Starting controller node...'),
            controller_node])

    return LaunchDescription([
        controller_arg, trajectory_arg, gui_arg, x_arg, y_arg, z_arg,
        gazebo_launch,
        delayed_controller])
```

### Step 4.9 — Update setup.py

**File:** `src/cf_trajectory_controller/setup.py`

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cf_trajectory_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yashu',
    maintainer_email='your@email.com',
    description='Modular trajectory tracking controllers for Crazyflie 2.1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cf_controller_node = cf_trajectory_controller.nodes.cf_controller_node:main',
            'cf_trajectory_plotter = cf_trajectory_controller.utils.cf_trajectory_plotter:main',
        ],
    },
)
```

### Step 4.10 — Build Control Package

**Active directory: `~/crazyflie_control_ws`**
```bash
cd ~/crazyflie_control_ws
colcon build --symlink-install \
    --packages-select cf_trajectory_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tee ~/crazyflie_control_ws/logs/build_controller.log

source ~/crazyflie_control_ws/install/setup.bash
```

### ✅ Verification Step 4

```bash
ros2 pkg list | grep cf_trajectory
# Expected: cf_trajectory_controller

ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py --show-args
# Expected: shows controller, trajectory, gui, x, y, z arguments
```

---

## Phase 5: Launch File and Plotter

### Running the Full Simulation

```bash
# Default: cascaded_pid + figure8 + GUI
ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py

# Headless (no GUI):
ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py gui:=false
```

### Running the Plotter

Start in a separate terminal after the simulation is running:
```bash
ros2 run cf_trajectory_controller cf_trajectory_plotter
```

Records for 30 seconds, then saves plots to `~/crazyflie_control_ws/logs/`

### Plots Generated

| Plot File | Content |
|---|---|
| `plot_1_trajectory_*.png` | XY top view + XZ side view (actual vs reference) |
| `plot_2_position_tracking_*.png` | x, y, z over time with reference |
| `plot_3_position_errors_*.png` | ex, ey, ez with RMS annotation |
| `plot_4_control_inputs_*.png` | u1, u2, u3, u4 over time |
| `plot_5_attitude_angles_*.png` | Roll, pitch, yaw angles |
| `plot_6_attitude_errors_*.png` | Roll, pitch, yaw errors with RMS |
| `plot_7_velocity_tracking_*.png` | vx, vy, vz actual vs reference |
| `plot_8_rms_convergence_*.png` | Rolling RMS convergence |
| `plot_9_phase_portraits_*.png` | ex vs ėx phase plane |
| `plot_10_summary_*.png` | 4-panel summary |
| `metrics_*.csv` | Numeric metrics for comparison |

### Verified Performance — Cascaded PID + Figure-8

| Metric | Value |
|---|---|
| ex RMS | 0.0141 m |
| ey RMS | 0.0496 m |
| ez RMS | 0.0184 m |
| 3D RMS total | **0.0547 m** |
| ez steady-state | 0.018 m |
| Roll amplitude | ±2 deg |
| Pitch amplitude | ±0.5 deg |
| Yaw amplitude | ±0.003 deg |
| Control saturation | None |

---

## Architecture Reference

### Project Directory Structure

```
~/crazyflie_control_ws/
├── scripts/
│   └── activate_crazyflie_ws.sh         ← Source this every session
├── logs/                                 ← All plots and build logs saved here
├── docs/                                 ← Documentation
├── cf_control_venv/                      ← Python venv (numpy, scipy, etc.)
└── src/
    ├── crazyflie_gazebo_sim/             ← PACKAGE 1: Simulation
    │   ├── src/cf_motor_plugin.cpp       ← C++ force/torque plugin (KEY FILE)
    │   ├── models/crazyflie_2_1/
    │   │   ├── model.config
    │   │   ├── model.sdf                 ← Drone physics model
    │   │   └── meshes/*.dae              ← 3D visual mesh
    │   ├── worlds/*.world                ← Gazebo environment
    │   ├── urdf/*.urdf.xacro             ← TF2 transforms (for RViz)
    │   └── launch/gazebo_cf1_sim.launch.py
    │
    ├── cf_trajectory_controller/         ← PACKAGE 2: Control
    │   ├── config/controller_params.yaml ← ALL gains (edit here only)
    │   ├── launch/cf1_full_simulation.launch.py ← MAIN entry point
    │   └── cf_trajectory_controller/
    │       ├── core/cf21_parameters.py   ← Physical constants
    │       ├── controllers/
    │       │   ├── base_controller.py    ← Abstract interface
    │       │   └── cascaded_pid.py       ← PID implementation
    │       ├── trajectories/
    │       │   └── figure8_trajectory.py
    │       ├── nodes/
    │       │   └── cf_controller_node.py ← Main ROS2 node
    │       └── utils/
    │           └── cf_trajectory_plotter.py
    │
    └── crazyswarm2/                      ← PACKAGE 3: External
        └── crazyflie_interfaces/         ← Custom message types (FullState etc.)
```

### ROS2 Node Graph

```
GAZEBO PROCESS
├── /gazebo                     publishes: /clock, /performance_metrics
├── cf_ground_truth_plugin      publishes: /cf1/ground_truth/odom (100Hz)
├── cf_imu_plugin               publishes: /cf1/imu (500Hz)
└── cf_motor_plugin  ←──────── subscribes: /cf1/control_debug
    └── AddForce() + AddRelativeTorque() → Physics Engine (1000Hz)

/cf_controller_node (100Hz timer)
    subscribes: /cf1/ground_truth/odom
    publishes:  /cf1/cmd_full_state   (for future real hardware)
                /cf1/control_debug    [u1,u2,u3,u4,ex,ey,ez,t]
                /cf1/trajectory_ref   (reference position)
    contains:   CascadedPID + Figure8Trajectory (loaded from YAML)

/cf1/cf1_state_publisher (robot_state_publisher)
    publishes: /tf, /tf_static (coordinate frames for RViz)

/cf_trajectory_plotter (optional)
    subscribes: /cf1/ground_truth/odom
                /cf1/trajectory_ref
                /cf1/control_debug
    outputs:    10 PNG plots + 1 CSV in ~/crazyflie_control_ws/logs/
```

### Topic Reference

| Topic | Type | Publisher | Rate | Content |
|---|---|---|---|---|
| `/cf1/ground_truth/odom` | `nav_msgs/Odometry` | Gazebo p3d plugin | 100Hz | Position, orientation, velocity |
| `/cf1/imu` | `sensor_msgs/Imu` | Gazebo IMU plugin | 500Hz | Accel, gyro (with noise) |
| `/cf1/control_debug` | `std_msgs/Float64MultiArray` | cf_controller_node | 100Hz | [u1,u2,u3,u4,ex,ey,ez,t] |
| `/cf1/cmd_full_state` | `crazyflie_interfaces/FullState` | cf_controller_node | 100Hz | Desired state (for hardware) |
| `/cf1/trajectory_ref` | `nav_msgs/Odometry` | cf_controller_node | 100Hz | Reference position |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | 1000Hz | Simulation time |

### Data Flow (One Control Cycle)

```
Step 1: Gazebo physics (1000Hz) → computes drone state
Step 2: cf_ground_truth_plugin (100Hz) → publishes /cf1/ground_truth/odom
Step 3: cf_controller_node.odom_callback() → extracts 12-state vector
Step 4: trajectory.get_reference(sim_time) → computes reference[12]
Step 5: controller.compute_control(state, reference, dt) → u[4]
Step 6: publish /cf1/control_debug → received by cf_motor_plugin
Step 7: cf_motor_plugin.OnUpdate() → AddForce() + AddRelativeTorque()
Step 8: Back to Step 1
```

### State Vector Convention

```
state[0:3]  = [x, y, z]                    positions [m]
state[3:6]  = [phi, theta, psi]             euler angles [rad] (ZYX convention)
state[6:9]  = [x_dot, y_dot, z_dot]        velocities [m/s]
state[9:12] = [phi_dot, theta_dot, psi_dot] angular rates [rad/s]
```

### Control Output Convention

```
u[0] = u1 = total thrust      [N]    range: [0, 1.3]
u[1] = u2 = roll torque       [N.m]  range: [-0.002, 0.002]
u[2] = u3 = pitch torque      [N.m]  range: [-0.002, 0.002]
u[3] = u4 = yaw torque        [N.m]  range: [-0.001, 0.001]
```

---

## Adding a New Controller

> ⚠️ **Complete ALL steps in order.** Skipping any step will cause the node to crash or ignore your new controller.

### Step 1 — Create the controller file

**Location:** `src/cf_trajectory_controller/cf_trajectory_controller/controllers/conventional_smc.py`

```python
import numpy as np
from cf_trajectory_controller.controllers.base_controller import BaseController
from cf_trajectory_controller.core.cf21_parameters import (
    MASS, GRAVITY, U1_MAX, U1_MIN, U2_MAX, U2_MIN, U3_MAX, U3_MIN, U4_MAX, U4_MIN
)

class ConventionalSMC(BaseController):

    def __init__(self, params: dict):
        super().__init__(params)
        # Load your gains from params dict
        self.lambda_x = params.get('lambda_x', 2.0)
        self.k_x = params.get('k_x', 5.0)
        # ... all other gains
        self.reset()

    def reset(self):
        # Reset any internal states (sliding surfaces, etc.)
        pass

    def compute_control(self, state, reference, dt):
        # YOUR EXACT MATLAB EQUATIONS HERE
        # Must return np.array([u1, u2, u3, u4])
        pass
```

### Step 2 — Add gains to YAML

**File:** `config/controller_params.yaml` — add at the bottom:

```yaml
    conventional_smc:
      lambda_x: 2.0
      lambda_y: 2.0
      lambda_z: 4.0
      k_x: 5.0
      # ... all your gains
```

### Step 3 — Add gain names to node loader

**File:** `nodes/cf_controller_node.py` — in `_load_controller_params()`:

```python
gain_names = [
    'Kp_x', 'Ki_x', 'Kd_x',           # existing PID gains
    # ... existing gains ...
    'lambda_x', 'lambda_y', 'lambda_z',  # ADD your SMC gains
    'k_x', 'k_y', 'k_z',
]
```

### Step 4 — Import and register in the node

**File:** `nodes/cf_controller_node.py`

```python
# At top — add import:
from cf_trajectory_controller.controllers.conventional_smc import ConventionalSMC

# In CONTROLLER_REGISTRY — add entry:
CONTROLLER_REGISTRY = {
    'cascaded_pid':     CascadedPID,
    'conventional_smc': ConventionalSMC,   # ADD THIS LINE
}
```

### Step 5 — Copy files to build directory

```bash
cp ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/controllers/conventional_smc.py \
   ~/crazyflie_control_ws/build/cf_trajectory_controller/cf_trajectory_controller/controllers/conventional_smc.py

cp ~/crazyflie_control_ws/src/cf_trajectory_controller/cf_trajectory_controller/nodes/cf_controller_node.py \
   ~/crazyflie_control_ws/build/cf_trajectory_controller/cf_trajectory_controller/nodes/cf_controller_node.py
```

### Step 6 — Switch to new controller

**File:** `config/controller_params.yaml`

```yaml
active_controller: "conventional_smc"
```

### Step 7 — Launch and verify

```bash
ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py
# Expected: [INFO] Loaded controller: ConventionalSMC
```

---

## Adding a New Trajectory

### Step 1 — Create trajectory file

**Location:** `src/cf_trajectory_controller/cf_trajectory_controller/trajectories/circle_trajectory.py`

```python
import numpy as np

class CircleTrajectory:
    def __init__(self, params: dict):
        self.radius  = params.get('radius',  0.5)
        self.omega   = params.get('omega',   0.4)
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
    'radius', 'omega',                                           # circle
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

Same pattern as controller Steps 5 and 6.

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| Drone doesn't move | `GAZEBO_PLUGIN_PATH` not set | Always run `source activate_crazyflie_ws.sh` first |
| `[CrazyflieMotorPlugin]` NOT in terminal | Plugin `.so` not found | Rebuild `crazyflie_gazebo_sim` package |
| Controller says "Waiting for state" forever | `ROS_DOMAIN_ID` mismatch | All terminals must use same domain ID |
| `Loaded 0 gains` message | YAML not loaded | Check launch file passes `controller_config` to Node |
| Drone oscillates wildly | Control gains too high | CF2.1 inertia = 16.5e-6 kg.m² — use Kp_phi ≈ 0.004 |
| Control inputs show solid blocks in plot | Torque limits too low | Physics limit: U2_MAX = 0.002 N.m for CF2.1 |
| Position tracking plot shows `1e9` on time axis | Clock mismatch in plotter | Use sim time from odom header stamp |
| colcon scans venv and shows errors | colcon finds numpy `setup.py` | `touch ~/crazyflie_control_ws/cf_control_venv/COLCON_IGNORE` |
| `mpl_toolkits` import error | System version loaded before venv | Create `__init__.py` in venv mpl_toolkits (Phase 1.4) |
| Gazebo shows gravity error | `<gravity>` tag in world SDF | Remove `<gravity>` from `<physics>` block in `.world` file |
| `apply_link_wrench` service does nothing | Known Gazebo Classic 11 + ROS2 bug | Use the C++ `cf_motor_plugin` instead (Phase 3.5) |

---

## Performance Benchmarks

Use this table to compare controller performance. Run each for 30 seconds on the same trajectory and record metrics from the plotter.

| Controller | Trajectory | ex RMS (m) | ey RMS (m) | ez RMS (m) | 3D RMS (m) | Notes |
|---|---|---|---|---|---|---|
| Cascaded PID | Figure-8 | 0.0141 | 0.0496 | 0.0184 | **0.0547** | Baseline |
| Conventional SMC | Figure-8 | — | — | — | — | To be filled |
| Super Twisting SMC | Figure-8 | — | — | — | — | To be filled |
| NS Terminal SMC | Figure-8 | — | — | — | — | To be filled |

---

*End of Document*
