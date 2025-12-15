import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointRecorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.cb,
            10
        )
        self.file = open('/tmp/joint_motion.csv', 'w')
        self.writer = csv.writer(self.file)

    def cb(self, msg):
        self.writer.writerow(msg.position)

def main():
    rclpy.init()
    node = JointRecorder()
    rclpy.spin(node)
    node.file.close()
    rclpy.shutdown()

