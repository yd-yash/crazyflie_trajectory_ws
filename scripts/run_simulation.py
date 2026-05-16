#!/usr/bin/env python3
"""
Orchestrator script for Crazyflie 2.1 simulation.
Prompts for controller and trajectory, patches the YAML,
then launches Gazebo and the controller node in separate Terminator windows.

Usage:
    cd ~/crazyflie_control_ws
    python3 scripts/run_simulation.py
"""

import os
import sys
import time
import subprocess

# ─────────────────────────────────────────────
# Paths
# ─────────────────────────────────────────────
WS_ROOT = os.path.expanduser("~/crazyflie_control_ws")
YAML_PATH = os.path.join(
    WS_ROOT,
    "src/cf_trajectory_controller/config/controller_params.yaml"
)
ACTIVATE_SCRIPT = os.path.join(WS_ROOT, "scripts/activate_crazyflie_ws.sh")

# ─────────────────────────────────────────────
# Available options (extend these lists as needed)
# ─────────────────────────────────────────────
CONTROLLERS = [
    "cascaded_pid",
    "conventional_smc",
    "super_twisting_smc",
    "nstt_smc",
]

TRAJECTORIES = [
    "figure8",
]

# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────

def print_banner():
    print("\n" + "=" * 55)
    print("   Crazyflie 2.1 Simulation Launcher")
    print("   Platform : Ubuntu 22.04 | ROS2 Humble | Gazebo 11")
    print("=" * 55 + "\n")


def prompt_choice(label: str, options: list) -> str:
    """Print a numbered menu and return the chosen string."""
    print(f"Select {label}:")
    for i, opt in enumerate(options, start=1):
        print(f"  [{i}] {opt}")
    while True:
        raw = input(f"Enter number (1–{len(options)}): ").strip()
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(options):
                chosen = options[idx]
                print(f"  ✔  {label} set to: {chosen}\n")
                return chosen
        print(f"  ✘  Invalid input. Please enter a number between 1 and {len(options)}.")


def patch_yaml(yaml_path: str, controller: str) -> None:
    """
    Replace the active_controller value in the YAML file.
    Only the active_controller line is changed — all gains are preserved.
    """
    if not os.path.isfile(yaml_path):
        print(f"  ✘  YAML not found at: {yaml_path}")
        sys.exit(1)

    with open(yaml_path, "r") as f:
        lines = f.readlines()

    patched = False
    new_lines = []
    for line in lines:
        stripped = line.lstrip()
        if stripped.startswith("active_controller:"):
            # Preserve leading indentation
            indent = line[: len(line) - len(stripped)]
            new_lines.append(f'{indent}active_controller: "{controller}"\n')
            patched = True
        else:
            new_lines.append(line)

    if not patched:
        print("  ✘  Could not find 'active_controller:' key in YAML.")
        print("     Please ensure it exists in controller_params.yaml.")
        sys.exit(1)

    with open(yaml_path, "w") as f:
        f.writelines(new_lines)

    print(f"  ✔  YAML patched — active_controller: \"{controller}\"")
    print(f"     File: {yaml_path}\n")


def build_bash_command(ros_command: str) -> str:
    """
    Wrap a ROS2 command with workspace + venv activation.
    The trailing 'exec bash' keeps the Terminator window open after the
    process ends so you can read logs.
    """
    return (
        f"bash -c '"
        f"cd {WS_ROOT} && "
        f"source {ACTIVATE_SCRIPT} && "
        f"{ros_command}; "
        f"echo \"--- Process ended. Press Ctrl+C or close window. ---\"; "
        f"exec bash"
        f"'"
    )

# for terminator
def launch_in_terminator(title: str, ros_command: str) -> subprocess.Popen:
    """
    Open a new Terminator window and run the given ROS2 command inside it.
    Returns the Popen handle for the Terminator process.
    """
    bash_cmd = build_bash_command(ros_command)
    terminator_cmd = [
        "terminator",
        "--title", title,
        "-e", bash_cmd,
    ]
    print(f"  Launching: {title}")
    print(f"  Command  : {ros_command}")
    proc = subprocess.Popen(terminator_cmd)
    return proc

# for terminal
# def launch_in_terminator(title: str, ros_command: str) -> subprocess.Popen:
#     """
#     Open a new GNOME Terminal window and run the given ROS2 command inside it.
#     Returns the Popen handle for the gnome-terminal process.
#     """
#     bash_cmd = build_bash_command(ros_command)
#     gnome_cmd = [
#         "gnome-terminal",
#         "--title", title,
#         "--",
#         "bash", "-c", bash_cmd,
#     ]
#     print(f"  Launching: {title}")
#     print(f"  Command  : {ros_command}")
#     proc = subprocess.Popen(gnome_cmd)
#     return proc


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def main():
    print_banner()

    # ── 1. User selections ───────────────────
    controller = prompt_choice("Controller", CONTROLLERS)
    trajectory = prompt_choice("Trajectory", TRAJECTORIES)

    # ── 2. Patch YAML ────────────────────────
    print("Patching controller_params.yaml ...")
    patch_yaml(YAML_PATH, controller)

    # ── 3. Launch Gazebo ─────────────────────
    gazebo_cmd = (
        "ros2 launch cf_trajectory_controller cf1_full_simulation.launch.py"
    )
    print("Launching Gazebo ...")
    launch_in_terminator("CF Gazebo", gazebo_cmd)

    # ── 4. Wait for Gazebo to initialize ─────
    GAZEBO_WAIT = 8  # seconds — increase if Gazebo is slow to start on your machine
    print(f"\n  Waiting {GAZEBO_WAIT}s for Gazebo to initialize ...\n")
    for i in range(GAZEBO_WAIT, 0, -1):
        print(f"  Starting controller node in {i}s ...", end="\r")
        time.sleep(1)
    print()

    # ── 5. Launch controller node ─────────────
    controller_cmd = (
        "ros2 run cf_trajectory_controller cf_controller_node "
        "--ros-args --params-file "
        "src/cf_trajectory_controller/config/controller_params.yaml"
    )
    print("Launching Controller Node ...")
    launch_in_terminator(f"CF Controller [{controller}]", controller_cmd)

    # ── 6. Summary ───────────────────────────
    print("\n" + "=" * 55)
    print("  ✅  Gazebo launched")
    print(f"  ✅  Controller node launched  →  {controller}")
    print(f"  ✅  Trajectory selected       →  {trajectory}")
    print("=" * 55)
    print("\n  📊  When tracking looks stable, run the plotter manually:")
    print(f"\n      cd {WS_ROOT} && source scripts/activate_crazyflie_ws.sh")
    print("      python3 src/cf_trajectory_controller/cf_trajectory_controller/utils/cf_trajectory_plotter.py")
    print("\n  Launcher exiting. Your simulation is running.\n")


if __name__ == "__main__":
    main()