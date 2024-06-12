#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import signal
import sys


class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)


    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter_}")
        self.counter_ += 1
        

def signal_handler(signal, frame):
    print("\nThe process is cancelled") 
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    signal.signal(signal.SIGINT, signal_handler)


    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    
    
    main()
    
