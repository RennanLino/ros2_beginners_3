import rclpy
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import MoveRobot
from std_msgs.msg import Empty


class MoveRobotClientNode(Node):
    def __init__(self):
        self.name = "move_robot_client"
        super().__init__(self.name)
        self.move_robot_client = ActionClient(
            self,
            MoveRobot,
            "move_robot")
        self.cancel_subscriber = self.create_subscription(Empty, "cancel_move", self.cancel_move_callback, 10)
        self.get_logger().info(f"{self.name} has been started")

    def send_goal(self, position, velocity):
        # Wait for the server to become available
        self.move_robot_client.wait_for_server()

        # Create a goal to send to the action server
        goal = MoveRobot.Goal(position=position, velocity=velocity)

        # Send the goal
        future = self.move_robot_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def cancel_move_callback(self, msg: Empty):
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle is None:
            self.get_logger().info("Sending a cancel request")
            self.goal_handle.cancel_goal_async()

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
        self.get_logger().info(f"Result: {result.position}")

    def goal_feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_position
        self.get_logger().info(f"Feedback: {position}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode()
    node.send_goal(76, 7)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()