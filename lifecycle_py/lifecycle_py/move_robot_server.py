import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from my_robot_interfaces.action import MoveRobot


class MoveRobotServerNode(LifecycleNode):
    def __init__(self):
        self.name = "my_robot"
        super().__init__(self.name)
        self.current_position =0
        self.min_position = 0
        self.max_position = 100
        self.is_activated = False
        self.goal_handle: ServerGoalHandle | None = None
        self.goal_lock = threading.Lock()
        self.move_robot_server = None
        self.get_logger().info(f"Robot position: {self.current_position}")

    def on_configure(self, state):
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.name = self.get_parameter("robot_name").value
        self.move_robot_server = ActionServer(
            self,
            MoveRobot,
            f"{self.name}/move_robot",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"{self.name} has started.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.undeclare_parameter("robot_name")
        self.move_robot_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.move_robot_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"{self.name} is activated.")
        self.is_activated = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"{self.name} is deactivated.")
        self.is_activated = False
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.goal_handle.abort()
        return super().on_deactivate(state)

    def goal_callback(self, goal_request: MoveRobot.Goal):
        response = GoalResponse.ACCEPT
        target_position = goal_request.position

        if not self.is_activated:
            self.get_logger().warn(f"{self.name} is not activated. Rejecting goal request.")
            response = GoalResponse.REJECT

        # Reject goal if the target position is out of range
        if target_position not in range(self.min_position, self.max_position) or goal_request.velocity <= 0:
            self.get_logger().warn(f"{self.name} received an invalid goal request.")
            response = GoalResponse.REJECT

        # To replace goal if new goal is valid, abort current one
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.goal_handle.abort()

        return response

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"{self.name} received a cancel request for goal {goal_handle.goal_id}.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock:
            self.goal_handle = goal_handle

        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()

        target_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        steps = (abs(target_position - self.current_position) // velocity) + 1

        direction = 1 if target_position > self.current_position else -1
        velocity = velocity * direction

        for _ in range(steps):
            if not goal_handle.is_active:
                result.message = "Preempted by another goal."
                break

            if goal_handle.is_cancel_requested:
                self.get_logger().info(f"{self.name} is canceling the goal.")
                goal_handle.canceled()
                break

            next_position = self.current_position + velocity

            if target_position * direction > next_position * direction:
                self.current_position = next_position
            else:
                self.current_position = target_position

            result.position = self.current_position
            feedback.current_position = self.current_position
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        if self.current_position == target_position:
            result.message = "Reached the target position"
            goal_handle.succeed()

        return result
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode()
    rclpy.spin(node, MultiThreadedExecutor(num_threads=4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
