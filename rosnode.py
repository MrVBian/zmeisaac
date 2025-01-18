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
        
        # # 获取 Dynamic Control 接口
        # dc = _dynamic_control.acquire_dynamic_control_interface()
        # # 获取机械臂 Articulation 句柄
        # art = dc.get_articulation("/World/envs/env_0/zmebot_description/world")
        # if art == _dynamic_control.INVALID_HANDLE:
        #     Printer.print_error("无法获取 zmebot_description articulation 句柄")
        #     return
        # # 机械臂的 DOF 数量
        # num_dofs = dc.get_articulation_dof_count(art)
        # if num_dofs == 0:
        #     print("未检测到关节 DOF")
        #     return
        
        # # 创建目标关节角度的字典
        # jointTargets = {}
        # # 读取 ROS 2 消息中的目标关节值
        # for joint in msg.left_arm_joints:
        #     jointTargets[joint.joint_name.data] = joint.joint_position
        # for joint in msg.right_arm_joints:
        #     jointTargets[joint.joint_name.data] = joint.joint_position
        
        # # 读取仿真机械臂的所有 DOF，并匹配目标角度
        # targetPositions = []

        # for i in range(num_dofs):
        #     joint_handle = dc.get_articulation_dof(art, i)
        #     joint_name = dc.get_dof_name(joint_handle)
        #     print(f"Sim Joint {i} name: {joint_name}")

        #     # 如果在目标关节角度字典中，则使用目标角度，否则使用当前角度
        #     if joint_name in jointTargets:
        #         targetPositions.append(jointTargets[joint_name])
        #     else:
        #         targetPositions.append(dc.get_dof_position(joint_handle))  # 维持当前角度

        # # 设置关节目标角度
        # # dc.set_articulation_dof_position_targets(art, targetPositions)
        # self.targetPositions = copy.deepcopy(targetPositions)


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

#         # self._armTaskClient = ActionClient(self, ArmTask, 'action_topic')



