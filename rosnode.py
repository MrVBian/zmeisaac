import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from printer import Printer
from dual_arm_interfaces.action import ArmTask


class RosNode(Node):
    def __init__(self):
        try:
            super().__init__("ZmeIsaacRosNode")
            Printer.print_normal("ZmeIsaacRosNode was already initialized")
        except RuntimeError as e:
            Printer.print_error("rclpy was already initialized:", e)

        self._action_client = ActionClient(self, ArmTask, 'action_topic')

    def sendGoal(self, goal_value:int):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            Printer.print_error('Action server not available!')
            return
        goal_msg = ArmTask.Goal()
        goal_msg = goal_value

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedbackCallback)
        future.add_done_callback(self.goalResponseCallback)

    def goalResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            Printer.print_normal("Goal rejected")
            return

        Printer.print_normal("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.resultCallback)

    def feedbackCallback(self, feedback_msg):
        Printer.print_normal(f"Feedback: {feedback_msg.feedback.feedback}")

    def resultCallback(self, future):
        result = future.result().result
        Printer.print_normal(f"Result: {result.result}")


    # try:
    #     rclpy.init()
    #     rosNode = RosNode()
    #     Printer.print_normal('ros init')
    # except RuntimeError as e:
    #     Printer.print_error("rclpy was already initialized:", e)



# class RosNode(Node):
#     def __init__(self):
#         try:
#             # rclpy.init()
#             super().__init__("ZmeIsaacRosNode")
#         except RuntimeError as e:
#             print("rclpy was already initialized:", e)

#         # self._action_client = ActionClient(self, ArmTask, 'action_topic')



