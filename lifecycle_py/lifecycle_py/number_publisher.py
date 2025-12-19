#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle.node import TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher = None
        self.number_timer = None

    # Create ROS2 communications, connect to hardware
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("On configure")
        self.number_publisher = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        return TransitionCallbackReturn.SUCCESS

    # Clean up ROS2 communications, disconnect from hardware
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("On cleanup")
        self._destroy_stuff()
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable hardware
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("On activate")
        return super().on_activate(previous_state)

    # Deactivate/Disable hardware
    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("On deactivate")
        return super().on_deactivate(previous_state)

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self._destroy_stuff()
        return TransitionCallbackReturn.SUCCESS

    # Process errors, deactivate + cleanup
    def on_error(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error("On error")
        self._destroy_stuff()
        return TransitionCallbackReturn.SUCCESS

    def _destroy_stuff(self):
        self.destroy_lifecycle_publisher(self.number_publisher)
        self.destroy_timer(self.number_timer)

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
