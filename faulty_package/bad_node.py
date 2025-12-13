import rclpy
from rclpy.node import Node

class BadNode(Node) # ERROR: Missing colon here
    def __init__(self):
        super().__init__('bad_node')
        # ERROR: Unsafe joint value (> 3.14) to trigger your safety heuristic
        self.unsafe_pos = 10.5 

def main():
    print("This will fail")
