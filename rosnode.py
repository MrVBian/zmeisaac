import rclpy
from rclpy.node import Node

from printer import Printer



class RosNode(Node):
    def __init__(self):
        try:
            # rclpy.init()
            super().__init__("ZmeIsaacRosNode")
        except RuntimeError as e:
            print("rclpy was already initialized:", e)

        # self._action_client = ActionClient(self, ArmTask, 'action_topic')

        # add_reference_to_stage(USD_CUP, USD_CUP_TAR)
        # add_reference_to_stage(USD_WINE, USD_WINE_TAR)

        # cup_prim = XFormPrim(USD_CUP_TAR)
        # cup_prim.set_scale([0.1, 0.1, 0.1])
        # cup_prim.set_world_pose([0.8, 0.15, 0.6])

        # wine_prim = XFormPrim(USD_WINE_TAR)
        # wine_prim.set_scale([0.1, 0.1, 0.1])
        # wine_prim.set_world_pose([0.8, -0.3, 0.6])

        # self.create_point()



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



