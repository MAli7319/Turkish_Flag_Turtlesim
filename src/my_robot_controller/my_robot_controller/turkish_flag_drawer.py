#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, Kill
from functools import partial
import signal
import sys
from math import radians, pi


BOLD_RED_LOG = '\033[1;31m'
BOLD_GREEN_LOG = '\033[1;32m'
BOLD_CYAN_LOG = '\033[1;36m'
RESET_COLOR = '\033[0;0m'


class TurtleControllerNode(Node):


    def __init__(self):

        super().__init__("turkish_flag_drawer")

        self.call_spawn_service(6.00, 4.15, 0.00, "turtle2")
        
        self.turtle1_cmd_vel_pubisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.turtle1_pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.turtle1_pose_callback, 10)
        
        self.turtle2_cmd_vel_pubisher_ = self.create_publisher(
            Twist, "/turtle2/cmd_vel", 10)
        self.turtle2_pose_subscriber_ = self.create_subscription(
            Pose, "/turtle2/pose", self.turtle2_pose_callback, 10)
        
        self.call_set_pen_service(255, 255, 255, 5, 0, "turtle1")
        self.call_set_pen_service(255, 255, 255, 5, 0, "turtle2")

        self.number_timer_ = self.create_timer(1.0, self.publish_msg)

        self.step_for_t1 = 0
        self.step_for_t2 = 0
        self.step_for_t3 = 0
        self.pattern = 0

    
    def publish_msg(self):
        print(BOLD_RED_LOG + "SELAM DUR!" + RESET_COLOR)
        print(BOLD_GREEN_LOG + "FREE PALESTINE" + RESET_COLOR)
        print(BOLD_CYAN_LOG + "FREE EAST TURKESTAN" + RESET_COLOR)
        print("\n"*(self.pattern % 17))
        self.pattern += 16



    def turtle1_pose_callback(self, pose: Pose):

        cmd = Twist()

        if self.step_for_t1 == 0:
            cmd.angular.z = 0.3
            if round(abs(pose.theta - radians(125)), 2) <= 0.02:
                cmd.angular.z = 0.0
                self.step_for_t1 += 1

        elif self.step_for_t1 == 1:
            cmd.angular.z = 0.4
            cmd.linear.x = 0.4 + (0.4*0.7)
            if abs(round(pose.theta - radians(55), 2)) <= 0.02:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.step_for_t1 += 1

        elif self.step_for_t1 == 2:
            cmd.angular.z = 0.3
            if abs(round(pose.theta - (-pi + radians(45)), 2)) <= 0.02:
                cmd.angular.z = 0.0
                self.step_for_t1 += 1

        elif self.step_for_t1 == 3:
            cmd.angular.z = -0.3
            cmd.linear.x = 0.3 + (0.3*0.31)
            if abs(round(pose.theta - (-pi + radians(135)), 2)) <= 0.02:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.step_for_t1 += 1

        else:
            if self.step_for_t1 == 4 and self.step_for_t2 == 10:
                self.call_kill_service("turtle1")
                self.call_kill_service("turtle2")
        
        self.turtle1_cmd_vel_pubisher_.publish(cmd)


    def turtle2_pose_callback(self, pose: Pose):

        cmd = Twist()

        if self.step_for_t2 == 0:
            cmd.angular.z = 0.2
            if round(abs(pose.theta - radians(72)), 3) <= 0.001:
                cmd.angular.z = 0.0
                self.step_for_t2 += 1

        elif self.step_for_t2 == 1:
            cmd.linear.x = 0.2
            if pose.y >= 5.15:
                cmd.linear.x = 0.0
                self.step_for_t2 += 1
        
        elif self.step_for_t2 == 2:
            cmd.angular.z = 0.2
            if round(abs(pose.theta - radians(108)), 3) <= 0.001:
                cmd.angular.z = 0.0
                self.step_for_t2 += 1
        
        elif self.step_for_t2 == 3:
            cmd.linear.x = -0.2
            if pose.x >= 6.65:
                cmd.linear.x = 0.0
                self.step_for_t2 += 1

        elif self.step_for_t2 == 4:
            cmd.angular.z = 0.2
            if round(abs(pose.theta - radians(144)), 3) <= 0.001:
                cmd.angular.z = 0.0
                self.step_for_t2 += 1

        elif self.step_for_t2 == 5:
            cmd.linear.x = 0.2
            if pose.x <= 5.8:
                cmd.linear.x = 0.0
                self.step_for_t2 += 1
            
        elif self.step_for_t2 == 6:
            cmd.angular.z = 0.2
            if round(abs(pi - pose.theta), 3) <= 0.002:
                cmd.angular.z = 0.0
                self.step_for_t2 += 1

        elif self.step_for_t2 == 7:
            cmd.linear.x = -0.2
            if pose.x >= 6.85:
                cmd.linear.x = 0.0
                self.step_for_t2 += 1

        elif self.step_for_t2 == 8:
            cmd.angular.z = 0.2
            if round(abs(pose.theta - (-pi + radians(36))), 3) <= 0.001:
                cmd.angular.z = 0.0
                self.step_for_t2 += 1

        elif self.step_for_t2 == 9:
            cmd.linear.x = 0.2
            if pose.y <= 4.15:
                cmd.linear.x = 0.0
                self.step_for_t2 += 1
        
        self.turtle2_cmd_vel_pubisher_.publish(cmd)


    def call_spawn_service(self, x, y, theta, name):

        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback))


    def call_set_pen_service(self, r, g, b, width, off, name):

        client = self.create_client(SetPen, f"/{name}/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off        

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback))


    def call_kill_service(self, name):

        client = self.create_client(Kill, "/kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = Kill.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback))


    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e,}")



def signal_handler(signal, frame):
    print("\nThe controller node is killed!")
    sys.exit(0)


def main(args=None):

    rclpy.init(args=args)
    node = TurtleControllerNode()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()


    