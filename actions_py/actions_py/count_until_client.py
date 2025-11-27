import rclpy
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client = ActionClient(
            self,
            CountUntil,
            "count_until")
        # self.timer = self.create_timer(2.0, self.cancel_goal) # set timer to cancel goal after 2 seconds
        self.get_logger().info("Action client has been started")

    def send_goal(self, target_number, period):
        # Wait for the server to become available
        self.count_until_client.wait_for_server()

        # Create a goal to send to the action server
        goal = CountUntil.Goal(target_number=target_number, period=period)

        # Send the goal
        future = self.count_until_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        self.get_logger().info("Sending a cancel request")
        self.goal_handle.cancel_goal_async()
        self.timer.cancel()

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("Goal got accepted")
            future = self.goal_handle.get_result_async()
            future.add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled")
        self.get_logger().info(f"Result: {result.reached_number}")

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f"Feedback: {number}")

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()