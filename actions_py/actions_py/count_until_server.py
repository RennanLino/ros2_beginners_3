import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil


class CountUntilServerNode(Node):
    def __init__(self):
        self.name = "count_until_server"
        super().__init__(self.name)
        self.goal_handle: ServerGoalHandle | None = None
        self.goal_lock = threading.Lock()
        self.goal_queue = []
        self.count_until_server = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"{self.name} has started.")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info(f"{self.name} received a goal request.")

        # # Policy: refuse new goal if current is still active
        # with self.goal_lock:
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().warn(f"{self.name} refusing new goal request because current goal is still active.")
        #         return GoalResponse.REJECT

        # Validate goal request here
        if goal_request.target_number <= 0:
            self.get_logger().error(f"{self.name} received an invalid goal request.")
            return GoalResponse.REJECT

        # # Policy: preempt an existing goal when receiving a new goal
        # with self.goal_lock:
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().warn(f"{self.name} aborting the current goal and aceept new goal.")
        #         self.goal_handle.abort()

        self.get_logger().info(f"{self.name} accepted the goal request.")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock:
            if self.goal_handle is not None:
                self.goal_queue.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"{self.name} received a cancel request for goal {goal_handle.goal_id}.")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle):
        with self.goal_lock:
            self.goal_handle = goal_handle

        # Get request from the goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info(f"{self.name} is counting until {target_number} in {period} seconds.")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()

        counter = 0
        for i in range(target_number):

            if not self.goal_handle.is_active:
                result.reached_number = counter
                break

            # Check if cancel request came in during execution
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f"{self.name} is canceling the goal.")
                goal_handle.canceled()
                result.reached_number = counter
                break

            counter += 1
            self.get_logger().info(f"{self.name} is counting: {counter}.")
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # Once done, set goal final state
        if counter == target_number:
            goal_handle.succeed()

        # And send the result
        result.reached_number = counter

        self.process_next_goal_in_queue()
        return result

    def process_next_goal_in_queue(self):
        with self.goal_lock:
            if len(self.goal_queue) > 0:
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor(num_threads=4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
