import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from printer import Printer
from dual_arm_interfaces.action import ArmTask
from dual_arm_interfaces.msg import DualArmStatus


class RosNode(Node):
    def __init__(self):
        try:
            super().__init__("ZmeIsaacRosNode")
            Printer.print_normal("ZmeIsaacRosNode was already initialized")
        except RuntimeError as e:
            Printer.print_error("rclpy was already initialized:", e)

        self.syncJointData = None
        self.syncJointClock = 0 # 0: no data; 1: data; 2: use data
        qosProfile = QoSProfile(depth=10)
        self._syncJointSub = self.create_subscription(
            DualArmStatus,  # 请替换为实际的消息类型
            '/dual_arm_status',
            self.SyncJointSubCallback,
            qosProfile
        )

        self._armTaskClient = ActionClient(self, ArmTask, 'action_topic')

    def SyncJointSubCallback(self, msg):
        """订阅回调函数，处理 Joint 状态更新"""
        # Printer.print_normal(f"Received Joint State: {msg}")
        if self.syncJointClock != 2:
            self.syncJointData = msg
            self.syncJointClock = 1
            for i in range(7):
                Printer.print_normal(f"{msg.left_arm_joints[i].joint_name.data} : {msg.left_arm_joints[i].joint_position}")
        
    def SendGoal(self, goalValue:int):
        if not self._armTaskClient.wait_for_server(timeout_sec=5.0):
            Printer.print_error('Action server not available!')
            return
        goalMsg = ArmTask.Goal()
        goalMsg = goalValue

        self._armTaskClient.wait_for_server()
        future = self._armTaskClient.send_goal_async(goalMsg, feedback_callback=self.FeedbackCallback)
        future.add_done_callback(self.GoalResponseCallback)

    def GoalResponseCallback(self, future):
        goalHandle = future.result()
        if not goalHandle.accepted:
            Printer.print_normal("Goal rejected")
            return

        Printer.print_normal("Goal accepted")
        result_future = goalHandle.get_result_async()
        result_future.add_done_callback(self.ResultCallback)

    def FeedbackCallback(self, feedback_msg):
        Printer.print_normal(f"Feedback: {feedback_msg.feedback.feedback}")

    def ResultCallback(self, future):
        result = future.result().result
        Printer.print_normal(f"Result: {result.result}")