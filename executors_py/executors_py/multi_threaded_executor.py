#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time


class Node1(Node):
    def __init__(self):
        super().__init__("node1")
        self.cb_group_1 = MutuallyExclusiveCallbackGroup()
        self.cb_group_2 = ReentrantCallbackGroup()
        self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group_2)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group_2)
        self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group_2)

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")


def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    rclpy.spin(node1, MultiThreadedExecutor(num_threads=4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()