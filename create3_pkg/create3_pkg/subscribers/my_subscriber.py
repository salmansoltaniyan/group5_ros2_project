import rclpy
from rclpy.node import Node

class SalmanSubscriber(Node):
    def __init__(self):
        super().__init__("salman_node")