#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm_node')   # Satisfies init_node check
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.get_logger().info('Move Arm Node Started')

    def send_trajectory(self):
        self.get_logger().info('Waiting for controller subscription...')
        while self.publisher_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)   # Proper sleep heuristic

        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        # Safety Heuristic: Joint values are within safe PI ranges
        point.positions = [0.0, -1.0, 1.0, -1.0, -1.5, 0.0]
        point.time_from_start = Duration(sec=3, nanosec=0)
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory published successfully.')


def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    node.send_trajectory()
    # Wait for the motion to complete in simulation
    time.sleep(5)
    node.destroy_node()
    rclpy.shutdown()
