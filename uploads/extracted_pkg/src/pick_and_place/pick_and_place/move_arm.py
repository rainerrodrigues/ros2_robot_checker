#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm_node')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

    def send_trajectory_once(self):
        self.get_logger().info('Waiting for controller to subscribe...')
        while self.publisher_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Controller connected! Sending trajectory...')
        
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.0, 1.0, -1.0, -1.5, 0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    node.send_trajectory_once()
    rclpy.spin_once(node, timeout_sec=3.0)
    node.get_logger().info('Motion complete, shutting down')
    node.destroy_node()
    rclpy.shutdown()
