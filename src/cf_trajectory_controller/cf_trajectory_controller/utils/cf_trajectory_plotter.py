"""
cf_trajectory_plotter.py
=========================
VERSION: v3
DATE: 2026-04-20
STATUS: WORKING — Complete plots with all additions
CHANGES FROM v1:
  - Fixed position tracking time axis (1e9 bug)
  - Added attitude error plot (ephi, etheta, epsi)
  - Added velocity tracking plot
  - Added rolling RMS convergence plot
  - Added phase portrait (ex vs ex_dot)
  - Fixed control inputs plot (y-axis margins)
  - Added CSV export for multi-controller comparison
  - Fixed ref_callback using odom sim time

Subscribes to:
  /cf1/ground_truth/odom  — actual state
  /cf1/trajectory_ref     — desired position
  /cf1/control_debug      — [u1,u2,u3,u4,ex,ey,ez,t]

Produces plots (saved to ~/crazyflie_control_ws/logs/):
  1. Trajectory (XY top view + XZ side view)
  2. Position tracking (x,y,z vs time)
  3. Position errors (ex,ey,ez vs time)
  4. Control inputs (u1,u2,u3,u4 vs time)
  5. Attitude angles (phi,theta,psi vs time)
  6. Attitude errors (ephi,etheta,epsi vs time)
  7. Velocity tracking (vx,vy,vz vs time)
  8. Rolling RMS convergence
  9. Phase portraits (ex vs ex_dot, ey vs ey_dot)
 10. Performance metrics CSV export

Usage:
  ros2 run cf_trajectory_controller cf_trajectory_plotter
"""

import rclpy
from rclpy.node import Node
import numpy as np
import os
import csv

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation


class CrazyflieTrajectoryPlotter(Node):
    """
    Collects trajectory data and generates complete performance plots.
    """

    def __init__(self):
        super().__init__('cf_trajectory_plotter')

        # --- Parameters ---
        self.declare_parameter('record_duration', 30.0)
        self.declare_parameter('save_path',
            os.path.expanduser('~/crazyflie_control_ws/logs/'))
        self.declare_parameter('controller_name', 'cascaded_pid')
        self.declare_parameter('trajectory_name', 'figure8')

        self.record_duration = self.get_parameter('record_duration').value
        self.save_path       = self.get_parameter('save_path').value
        self.controller_name = self.get_parameter('controller_name').value
        self.trajectory_name = self.get_parameter('trajectory_name').value

        os.makedirs(self.save_path, exist_ok=True)

        # --- Data storage: actual state ---
        self.t_actual     = []
        self.x_actual     = []
        self.y_actual     = []
        self.z_actual     = []
        self.vx_actual    = []
        self.vy_actual    = []
        self.vz_actual    = []
        self.phi_actual   = []
        self.theta_actual = []
        self.psi_actual   = []

        # --- Data storage: reference ---
        self.t_ref    = []
        self.x_ref    = []
        self.y_ref    = []
        self.z_ref    = []
        self.vx_ref   = []
        self.vy_ref   = []
        self.vz_ref   = []

        # --- Data storage: control + errors ---
        self.t_ctrl = []
        self.u1     = []
        self.u2     = []
        self.u3     = []
        self.u4     = []
        self.ex     = []
        self.ey     = []
        self.ez     = []

        self.start_time = None
        self.recording  = True

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry, '/cf1/ground_truth/odom',
            self.odom_callback, 10)

        self.ref_sub = self.create_subscription(
            Odometry, '/cf1/trajectory_ref',
            self.ref_callback, 10)

        self.ctrl_sub = self.create_subscription(
            Float64MultiArray, '/cf1/control_debug',
            self.ctrl_callback, 10)

        self.check_timer = self.create_timer(1.0, self.check_duration)

        self.get_logger().info(
            f'CrazyflieTrajectoryPlotter v2 started.\n'
            f'Recording for {self.record_duration}s...\n'
            f'Controller: {self.controller_name} | '
            f'Trajectory: {self.trajectory_name}'
        )

    # =========================================================================
    # TIME MANAGEMENT
    # =========================================================================

    def _get_elapsed(self, stamp):
        """Convert ROS stamp to elapsed time using sim time."""
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.start_time is None:
            self.start_time = t
            self.get_logger().info(
                f'Recording started at sim_time={t:.2f}s')
        return t - self.start_time

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def odom_callback(self, msg: Odometry):
        if not self.recording:
            return
        elapsed = self._get_elapsed(msg.header.stamp)
        if elapsed < 0:
            return

        self.t_actual.append(elapsed)

        # Position
        self.x_actual.append(msg.pose.pose.position.x)
        self.y_actual.append(msg.pose.pose.position.y)
        self.z_actual.append(msg.pose.pose.position.z)

        # Velocity
        self.vx_actual.append(msg.twist.twist.linear.x)
        self.vy_actual.append(msg.twist.twist.linear.y)
        self.vz_actual.append(msg.twist.twist.linear.z)

        # Orientation: quaternion → euler ZYX
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('ZYX')
        self.phi_actual.append(np.degrees(euler[2]))    # roll
        self.theta_actual.append(np.degrees(euler[1]))  # pitch
        self.psi_actual.append(np.degrees(euler[0]))    # yaw

    def ref_callback(self, msg: Odometry):
        if not self.recording:
            return
        if self.start_time is None:
            return
        # Use latest sim time from odom to avoid clock mismatch
        if len(self.t_actual) == 0:
            return
        elapsed = self.t_actual[-1]
        self.t_ref.append(elapsed)
        self.x_ref.append(msg.pose.pose.position.x)
        self.y_ref.append(msg.pose.pose.position.y)
        self.z_ref.append(msg.pose.pose.position.z)
        self.vx_ref.append(msg.twist.twist.linear.x)
        self.vy_ref.append(msg.twist.twist.linear.y)
        self.vz_ref.append(msg.twist.twist.linear.z)

    def ctrl_callback(self, msg: Float64MultiArray):
        if not self.recording or len(msg.data) < 8:
            return
        if self.start_time is None:
            return
        sim_t = float(msg.data[7])
        if sim_t < self.start_time:
            return
        self.t_ctrl.append(sim_t - self.start_time)
        self.u1.append(msg.data[0])
        self.u2.append(msg.data[1])
        self.u3.append(msg.data[2])
        self.u4.append(msg.data[3])
        self.ex.append(msg.data[4])
        self.ey.append(msg.data[5])
        self.ez.append(msg.data[6])

    def check_duration(self):
        if not self.recording or self.start_time is None:
            return
        if len(self.t_actual) > 10:
            current = self.t_actual[-1]
            self.get_logger().info(
                f'Recording: {current:.1f}/{self.record_duration:.1f}s | '
                f'actual={len(self.t_actual)} '
                f'ref={len(self.t_ref)} '
                f'ctrl={len(self.t_ctrl)}'
            )
            if current >= self.record_duration:
                self.recording = False
                self.get_logger().info(
                    'Recording complete. Generating plots...')
                self.generate_all_plots()

    # =========================================================================
    # PLOTTING
    # =========================================================================

    def generate_all_plots(self):
        """Generate all performance plots and export CSV."""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # Convert to numpy arrays
        t_a  = np.array(self.t_actual)
        xa   = np.array(self.x_actual)
        ya   = np.array(self.y_actual)
        za   = np.array(self.z_actual)
        vxa  = np.nan_to_num(np.array(self.vx_actual))
        vya  = np.nan_to_num(np.array(self.vy_actual))
        vza  = np.nan_to_num(np.array(self.vz_actual))
        phi  = np.array(self.phi_actual)
        th   = np.array(self.theta_actual)
        psi  = np.array(self.psi_actual)

        t_r  = np.array(self.t_ref)
        xr   = np.array(self.x_ref)
        yr   = np.array(self.y_ref)
        zr   = np.array(self.z_ref)
        vxr  = np.array(self.vx_ref)
        vyr  = np.array(self.vy_ref)
        vzr  = np.array(self.vz_ref)

        t_c  = np.array(self.t_ctrl)
        u1   = np.array(self.u1)
        u2   = np.array(self.u2)
        u3   = np.array(self.u3)
        u4   = np.array(self.u4)
        ex   = np.array(self.ex)
        ey   = np.array(self.ey)
        ez   = np.array(self.ez)

        tag = f'{self.controller_name}_{self.trajectory_name}'

        # Compute attitude errors (actual - reference=0 for phi,theta; psi_des=0)
        ephi   = phi   - 0.0
        etheta = th    - 0.0
        epsi   = psi   - 0.0

        # Compute velocity errors
        if len(t_r) > 0 and len(vxr) > 0:
            # Interpolate ref velocities to actual timestamps
            vxr_interp = np.interp(t_a, t_r, vxr)
            vyr_interp = np.interp(t_a, t_r, vyr)
            vzr_interp = np.interp(t_a, t_r, vzr)
            evx = vxr_interp - vxa
            evy = vyr_interp - vya
            evz = vzr_interp - vza
        else:
            vxr_interp = np.zeros_like(t_a)
            vyr_interp = np.zeros_like(t_a)
            vzr_interp = np.zeros_like(t_a)
            evx = evy = evz = np.zeros_like(t_a)

        # =====================================================================
        # Plot 1: 3D Trajectory (XY + XZ views)
        # =====================================================================
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle(
            f'Trajectory — {self.controller_name} | {self.trajectory_name}',
            fontsize=13)

        # XY plane
        if len(xr) > 0:
            axes[0].plot(xr, yr, 'r--', linewidth=2, label='Reference')
        axes[0].plot(xa, ya, 'b-', linewidth=1.5, label='Actual')
        axes[0].scatter(xa[0], ya[0], c='g', s=100,
                        marker='o', label='Start', zorder=5)
        axes[0].set_xlabel('X (m)')
        axes[0].set_ylabel('Y (m)')
        axes[0].set_title('XY Plane (Top View)')
        axes[0].legend()
        axes[0].grid(True)
        axes[0].set_aspect('equal')

        # XZ plane
        if len(xr) > 0:
            axes[1].plot(xr, zr, 'r--', linewidth=2, label='Reference')
        axes[1].plot(xa, za, 'b-', linewidth=1.5, label='Actual')
        axes[1].scatter(xa[0], za[0], c='g', s=100,
                        marker='o', label='Start', zorder=5)
        axes[1].set_xlabel('X (m)')
        axes[1].set_ylabel('Z (m)')
        axes[1].set_title('XZ Plane (Side View)')
        axes[1].legend()
        axes[1].grid(True)

        plt.tight_layout()
        self._save_fig(plt, f'plot_1_trajectory_{tag}.png')

        # =====================================================================
        # Plot 2: Position Tracking
        # =====================================================================
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(
            f'Position Tracking — {self.controller_name} | {self.trajectory_name}')

        labels = [('X', xa, xr), ('Y', ya, yr), ('Z', za, zr)]
        colors = ['b', 'b', 'b']
        ylabels = ['x (m)', 'y (m)', 'z (m)']

        for i, (name, actual, ref) in enumerate(labels):
            axes[i].plot(t_a, actual, 'b-', linewidth=1.5,
                         label=f'{name} actual')
            if len(t_r) > 0:
                axes[i].plot(t_r, ref, 'r--', linewidth=1.5,
                             label=f'{name} reference')
            axes[i].set_ylabel(ylabels[i])
            axes[i].legend(loc='upper right')
            axes[i].grid(True)

        axes[2].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_fig(plt, f'plot_2_position_tracking_{tag}.png')

        # =====================================================================
        # Plot 3: Position Errors
        # =====================================================================
        if len(t_c) > 0:
            fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            fig.suptitle(
                f'Position Errors — {self.controller_name} | {self.trajectory_name}')

            for i, (err, label, color) in enumerate([
                    (ex, '$e_x$ (m)', 'r'),
                    (ey, '$e_y$ (m)', 'g'),
                    (ez, '$e_z$ (m)', 'b')]):
                axes[i].plot(t_c, err, color=color, linewidth=1.5)
                axes[i].axhline(0, color='k', linestyle='--', linewidth=0.8)
                axes[i].set_ylabel(label)
                axes[i].grid(True)
                # Add RMS annotation
                rms = np.sqrt(np.mean(err**2))
                axes[i].set_title(f'RMS = {rms:.4f} m', fontsize=9)

            axes[2].set_xlabel('Time (s)')
            plt.tight_layout()
            self._save_fig(plt, f'plot_3_position_errors_{tag}.png')

        # =====================================================================
        # Plot 4: Control Inputs
        # =====================================================================
        if len(t_c) > 0:
            fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
            fig.suptitle(
                f'Control Inputs — {self.controller_name} | {self.trajectory_name}')

            ctrl_data = [
                (u1, '$u_1$ Thrust (N)', 'b'),
                (u2, '$u_2$ Roll Torque (N·m)', 'r'),
                (u3, '$u_3$ Pitch Torque (N·m)', 'g'),
                (u4, '$u_4$ Yaw Torque (N·m)', 'm'),
            ]
            for i, (data, label, color) in enumerate(ctrl_data):
                axes[i].plot(t_c, data, color=color, linewidth=1.5)
                axes[i].set_ylabel(label)
                axes[i].grid(True)
                # Dynamic y-axis with 30% margin to avoid solid blocks
                d_range = max(abs(data.max()), abs(data.min()), 1e-9)
                axes[i].set_ylim(-d_range * 1.3, d_range * 1.3)
                axes[i].axhline(0, color='k', linestyle='--', linewidth=0.5)

            axes[3].set_xlabel('Time (s)')
            plt.tight_layout()
            self._save_fig(plt, f'plot_4_control_inputs_{tag}.png')

        # =====================================================================
        # Plot 5: Attitude Angles
        # =====================================================================
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(
            f'Attitude Angles — {self.controller_name} | {self.trajectory_name}')

        for i, (data, label, color) in enumerate([
                (phi, '$\\phi$ Roll (deg)', 'b'),
                (th,  '$\\theta$ Pitch (deg)', 'r'),
                (psi, '$\\psi$ Yaw (deg)', 'g')]):
            axes[i].plot(t_a, data, color=color, linewidth=1.5)
            axes[i].axhline(0, color='k', linestyle='--', linewidth=0.8)
            axes[i].set_ylabel(label)
            axes[i].grid(True)

        axes[2].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_fig(plt, f'plot_5_attitude_angles_{tag}.png')

        # =====================================================================
        # Plot 6: Attitude Errors
        # =====================================================================
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(
            f'Attitude Errors — {self.controller_name} | {self.trajectory_name}')

        for i, (data, label, color) in enumerate([
                (ephi,   '$e_\\phi$ Roll Error (deg)', 'b'),
                (etheta, '$e_\\theta$ Pitch Error (deg)', 'r'),
                (epsi,   '$e_\\psi$ Yaw Error (deg)', 'g')]):
            axes[i].plot(t_a, data, color=color, linewidth=1.5)
            axes[i].axhline(0, color='k', linestyle='--', linewidth=0.8)
            axes[i].set_ylabel(label)
            axes[i].grid(True)
            rms = np.sqrt(np.mean(data**2))
            axes[i].set_title(f'RMS = {rms:.4f} deg', fontsize=9)

        axes[2].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_fig(plt, f'plot_6_attitude_errors_{tag}.png')

        # =====================================================================
        # Plot 7: Velocity Tracking
        # =====================================================================
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(
            f'Velocity Tracking — {self.controller_name} | {self.trajectory_name}')

        vel_data = [
            (vxa, vxr_interp, '$v_x$ (m/s)'),
            (vya, vyr_interp, '$v_y$ (m/s)'),
            (vza, vzr_interp, '$v_z$ (m/s)'),
        ]
        for i, (actual, ref, label) in enumerate(vel_data):
            axes[i].plot(t_a, actual, 'b-', linewidth=1.5, label='Actual')
            axes[i].plot(t_a, ref, 'r--', linewidth=1.5, label='Reference')
            axes[i].set_ylabel(label)
            axes[i].legend(loc='upper right')
            axes[i].grid(True)

        axes[2].set_xlabel('Time (s)')
        plt.tight_layout()
        self._save_fig(plt, f'plot_7_velocity_tracking_{tag}.png')

        # =====================================================================
        # Plot 8: Rolling RMS Convergence
        # =====================================================================
        if len(t_c) > 50:
            window = min(100, len(t_c) // 5)
            rms_x = self._rolling_rms(ex, window)
            rms_y = self._rolling_rms(ey, window)
            rms_z = self._rolling_rms(ez, window)
            rms_3d = self._rolling_rms(
                np.sqrt(ex**2 + ey**2 + ez**2), window)

            fig, ax = plt.subplots(figsize=(12, 6))
            fig.suptitle(
                f'Rolling RMS Convergence — {self.controller_name} | {self.trajectory_name}')

            ax.plot(t_c, rms_x,  'r-',  linewidth=1.5, label='RMS $e_x$')
            ax.plot(t_c, rms_y,  'g-',  linewidth=1.5, label='RMS $e_y$')
            ax.plot(t_c, rms_z,  'b-',  linewidth=1.5, label='RMS $e_z$')
            ax.plot(t_c, rms_3d, 'k--', linewidth=2.0, label='RMS 3D')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('RMS Error (m)')
            ax.legend()
            ax.grid(True)
            plt.tight_layout()
            self._save_fig(plt, f'plot_8_rms_convergence_{tag}.png')

        # =====================================================================
        # Plot 9: Phase Portraits
        # =====================================================================
        if len(t_c) > 10:
            # Compute error derivatives
            ex_dot = np.gradient(ex, t_c) if len(t_c) > 1 else np.zeros_like(ex)
            ey_dot = np.gradient(ey, t_c) if len(t_c) > 1 else np.zeros_like(ey)
            ez_dot = np.gradient(ez, t_c) if len(t_c) > 1 else np.zeros_like(ez)

            fig, axes = plt.subplots(1, 3, figsize=(15, 5))
            fig.suptitle(
                f'Phase Portraits — {self.controller_name} | {self.trajectory_name}')

            phase_data = [
                (ex, ex_dot, '$e_x$', '$\\dot{e}_x$', 'r'),
                (ey, ey_dot, '$e_y$', '$\\dot{e}_y$', 'g'),
                (ez, ez_dot, '$e_z$', '$\\dot{e}_z$', 'b'),
            ]
            for i, (err, err_dot, xlabel, ylabel, color) in \
                    enumerate(phase_data):
                axes[i].plot(err, err_dot, color=color,
                             linewidth=0.8, alpha=0.7)
                axes[i].scatter(err[0], err_dot[0], c='g',
                                s=50, zorder=5, label='Start')
                axes[i].scatter(err[-1], err_dot[-1], c='r',
                                s=50, zorder=5, label='End')
                axes[i].axhline(0, color='k', linestyle='--', linewidth=0.5)
                axes[i].axvline(0, color='k', linestyle='--', linewidth=0.5)
                axes[i].set_xlabel(xlabel + ' (m)')
                axes[i].set_ylabel(ylabel + ' (m/s)')
                axes[i].set_title(f'Phase: {xlabel}')
                axes[i].legend(fontsize=8)
                axes[i].grid(True)

            plt.tight_layout()
            self._save_fig(plt, f'plot_9_phase_portraits_{tag}.png')

        # =====================================================================
        # Plot 10: Performance Summary (single figure with key metrics)
        # =====================================================================
        if len(t_c) > 0:
            fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            fig.suptitle(
                f'Performance Summary — {self.controller_name} | {self.trajectory_name}',
                fontsize=13)

            # Top-left: XY trajectory
            if len(xr) > 0:
                axes[0,0].plot(xr, yr, 'r--', linewidth=2, label='Reference')
            axes[0,0].plot(xa, ya, 'b-', linewidth=1.5, label='Actual')
            axes[0,0].set_xlabel('X (m)')
            axes[0,0].set_ylabel('Y (m)')
            axes[0,0].set_title('Trajectory (Top View)')
            axes[0,0].legend()
            axes[0,0].grid(True)
            axes[0,0].set_aspect('equal')

            # Top-right: Position errors
            axes[0,1].plot(t_c, ex, 'r-', linewidth=1.2, label='$e_x$')
            axes[0,1].plot(t_c, ey, 'g-', linewidth=1.2, label='$e_y$')
            axes[0,1].plot(t_c, ez, 'b-', linewidth=1.2, label='$e_z$')
            axes[0,1].axhline(0, color='k', linestyle='--', linewidth=0.8)
            axes[0,1].set_xlabel('Time (s)')
            axes[0,1].set_ylabel('Error (m)')
            axes[0,1].set_title('Position Errors')
            axes[0,1].legend()
            axes[0,1].grid(True)

            # Bottom-left: Attitude
            axes[1,0].plot(t_a, phi, 'b-', linewidth=1.2, label='$\\phi$ Roll')
            axes[1,0].plot(t_a, th,  'r-', linewidth=1.2, label='$\\theta$ Pitch')
            axes[1,0].plot(t_a, psi, 'g-', linewidth=1.2, label='$\\psi$ Yaw')
            axes[1,0].set_xlabel('Time (s)')
            axes[1,0].set_ylabel('Angle (deg)')
            axes[1,0].set_title('Attitude Angles')
            axes[1,0].legend()
            axes[1,0].grid(True)

            # Bottom-right: Control inputs
            axes[1,1].plot(t_c, u1, 'b-', linewidth=1.2, label='$u_1$ Thrust')
            axes[1,1].set_xlabel('Time (s)')
            axes[1,1].set_ylabel('Thrust (N)')
            axes[1,1].set_title('Thrust Input')
            axes[1,1].legend()
            axes[1,1].grid(True)

            plt.tight_layout()
            self._save_fig(plt, f'plot_10_summary_{tag}.png')

        # =====================================================================
        # CSV Export
        # =====================================================================
        self._export_csv(t_c, ex, ey, ez, u1, u2, u3, u4, tag)

        # =====================================================================
        # Performance Metrics
        # =====================================================================
        self._print_metrics(t_a, ex, ey, ez, ephi, etheta, epsi)

        self.get_logger().info('All plots saved. Press Ctrl+C to exit.')

    # =========================================================================
    # HELPER METHODS
    # =========================================================================

    def _save_fig(self, plt, filename):
        """Save figure and log."""
        path = os.path.join(self.save_path, filename)
        plt.savefig(path, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Saved: {path}')

    def _rolling_rms(self, data: np.ndarray, window: int) -> np.ndarray:
        """Compute rolling RMS with given window size."""
        result = np.zeros_like(data)
        for i in range(len(data)):
            start = max(0, i - window + 1)
            result[i] = np.sqrt(np.mean(data[start:i+1]**2))
        return result

    def _export_csv(self, t_c, ex, ey, ez, u1, u2, u3, u4, tag):
        """Export performance data to CSV for multi-controller comparison."""
        if len(t_c) == 0:
            return
        path = os.path.join(self.save_path, f'metrics_{tag}.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'controller', 'trajectory',
                'rms_ex', 'rms_ey', 'rms_ez', 'rms_3d',
                'ess_ex', 'ess_ey', 'ess_ez',
                'mean_u1', 'max_u2', 'max_u3', 'max_u4',
                'duration'
            ])
            n_ss = min(200, len(ex))
            writer.writerow([
                self.controller_name, self.trajectory_name,
                float(np.sqrt(np.mean(ex**2))),
                float(np.sqrt(np.mean(ey**2))),
                float(np.sqrt(np.mean(ez**2))),
                float(np.sqrt(np.mean(ex**2 + ey**2 + ez**2))),
                float(np.mean(ex[-n_ss:])),
                float(np.mean(ey[-n_ss:])),
                float(np.mean(ez[-n_ss:])),
                float(np.mean(u1)),
                float(np.max(np.abs(u2))),
                float(np.max(np.abs(u3))),
                float(np.max(np.abs(u4))),
                float(t_c[-1]) if len(t_c) > 0 else 0.0
            ])
        self.get_logger().info(f'CSV exported: {path}')

    def _print_metrics(self, t_a, ex, ey, ez, ephi, etheta, epsi):
        """Print complete performance metrics to terminal."""
        if len(ex) == 0:
            return
        n_ss = min(200, len(ex))
        rms_ex    = np.sqrt(np.mean(ex**2))
        rms_ey    = np.sqrt(np.mean(ey**2))
        rms_ez    = np.sqrt(np.mean(ez**2))
        rms_3d    = np.sqrt(np.mean(ex**2 + ey**2 + ez**2))
        rms_ephi  = np.sqrt(np.mean(ephi**2))
        rms_etheta= np.sqrt(np.mean(etheta**2))
        rms_epsi  = np.sqrt(np.mean(epsi**2))

        self.get_logger().info(
            f'\n{"="*55}\n'
            f'  TRACKING PERFORMANCE METRICS\n'
            f'  Controller : {self.controller_name}\n'
            f'  Trajectory : {self.trajectory_name}\n'
            f'  Duration   : {t_a[-1]:.1f}s\n'
            f'{"="*55}\n'
            f'  RMS Position Errors:\n'
            f'    ex    = {rms_ex:.4f} m\n'
            f'    ey    = {rms_ey:.4f} m\n'
            f'    ez    = {rms_ez:.4f} m\n'
            f'    3D    = {rms_3d:.4f} m\n'
            f'  RMS Attitude Errors:\n'
            f'    ephi  = {rms_ephi:.4f} deg\n'
            f'    etheta= {rms_etheta:.4f} deg\n'
            f'    epsi  = {rms_epsi:.4f} deg\n'
            f'  Steady-State Errors (last 2s):\n'
            f'    ex_ss = {np.mean(ex[-n_ss:]):.4f} m\n'
            f'    ey_ss = {np.mean(ey[-n_ss:]):.4f} m\n'
            f'    ez_ss = {np.mean(ez[-n_ss:]):.4f} m\n'
            f'{"="*55}\n'
            f'  Plots saved to: {self.save_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieTrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Plotter stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
