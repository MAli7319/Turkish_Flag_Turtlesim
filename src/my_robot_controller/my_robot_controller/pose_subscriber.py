#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import signal
import sys


class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)


    def pose_callback(self, msg: Pose):
        self.get_logger().info(
            f"The turtle's coordinates: ({str(msg.x)}, {str(msg.y)})")


def signal_handler(signal, frame):
    print("\nThe subscriber is killed!")
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()