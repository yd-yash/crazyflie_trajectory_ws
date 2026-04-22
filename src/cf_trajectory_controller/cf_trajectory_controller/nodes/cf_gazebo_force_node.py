"""
cf_gazebo_force_node.py
========================
Applies control inputs [u1, u2, u3, u4] as physical forces and torques
to the Crazyflie model in Gazebo Classic 11.

Correct service: /apply_link_wrench (gazebo_msgs/srv/ApplyLinkWrench)
Link name format: model_name::link_name → cf1::cf_base_link

Subscribes:
  /cf1/control_debug     (std_msgs/Float64MultiArray) — [u1,u2,u3,u4,...]
  /cf1/ground_truth/odom (nav_msgs/Odometry)          — for orientation

Applies:
  Thrust u1 along body Z axis, rotated to world frame
  Torques u2,u3,u4 as body-frame torques
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import ApplyLinkWrench
from geometry_msgs.msg import Wrench, Point
from builtin_interfaces.msg import Duration


class CrazyflieGazeboForceNode(Node):

    def __init__(self):
        super().__init__('cf_gazebo_force_node')

        self.q = np.array([0.0, 0.0, 0.0, 1.0])
        self.last_u = np.zeros(4)
        self.state_received = False

        # Subscriber: control outputs
        self.ctrl_sub = self.create_subscription(
            Float64MultiArray,
            '/cf1/control_debug',
            self.control_callback,
            10
        )

        # Subscriber: odometry for orientation
        self.odom_sub = self.create_subscription(
            Odometry,
            '/cf1/ground_truth/odom',
            self.odom_callback,
            10
        )

        # Gazebo ApplyLinkWrench service client
        self.wrench_client = self.create_client(
            ApplyLinkWrench,
            '/apply_link_wrench'
        )

        self.get_logger().info('Waiting for /apply_link_wrench service...')
        while not self.wrench_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Still waiting for /apply_link_wrench...')

        self.get_logger().info('CrazyflieGazeboForceNode ready.')

        # Apply forces at 100 Hz
        self.timer = self.create_timer(0.01, self.apply_forces)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.q = np.array([q.x, q.y, q.z, q.w])
        self.state_received = True

    def control_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.last_u = np.array(msg.data[:4])

    def apply_forces(self):
        if not self.state_received:
            return

        u1 = float(self.last_u[0])   # thrust    [N]
        u2 = float(self.last_u[1])   # roll tau  [N.m]
        u3 = float(self.last_u[2])   # pitch tau [N.m]
        u4 = float(self.last_u[3])   # yaw tau   [N.m]

        # Rotate thrust from body Z to world frame
        rot = Rotation.from_quat(self.q)
        thrust_world = rot.apply(np.array([0.0, 0.0, u1]))

        req = ApplyLinkWrench.Request()
        req.link_name = 'cf1::cf_base_link'
        req.reference_frame = 'world'
        req.reference_point = Point(x=0.0, y=0.0, z=0.0)

        req.wrench = Wrench()
        req.wrench.force.x  = float(thrust_world[0])
        req.wrench.force.y  = float(thrust_world[1])
        req.wrench.force.z  = float(thrust_world[2])
        req.wrench.torque.x = u2
        req.wrench.torque.y = u3
        req.wrench.torque.z = u4

        # Apply for one control step (10ms), continuously renewed
        req.duration = Duration(sec=0, nanosec=10_000_000)

        self.wrench_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieGazeboForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Force node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
