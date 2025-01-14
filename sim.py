import argparse
from omni.isaac.lab.app import AppLauncher
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")    # create argparser
AppLauncher.add_app_launcher_args(parser)   # append AppLauncher cli args
args_cli = parser.parse_args()              # parse the arguments
app_launcher = AppLauncher(args_cli)        # launch omniverse app
simulation_app = app_launcher.app
"""Rest everything follows."""
from omni.isaac.lab.sim import SimulationCfg, SimulationContext


###### region bzw
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
print("project_root: ", current_dir)
from global_variables import *
from printer import Printer
from rosnode import *


from threading import Thread
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdShade
import omni.isaac.core.utils.prims as prims_utils # tranform
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
######

class Builder:
    def __init__(self):
        self._stage = open_stage(USD_SCENE)

        # 加载资源
        add_reference_to_stage(USD_CUP, USD_CUP_TAR)
        add_reference_to_stage(USD_WINE, USD_WINE_TAR)
        cup_prim = prims_utils.get_prim_at_path(USD_CUP_TAR)
        wine_prim = prims_utils.get_prim_at_path(USD_WINE_TAR)

        if cup_prim.IsValid():
            xform = UsdGeom.Xformable(cup_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set((0.8, 0.15, 0.6))
            xform.AddScaleOp().Set((0.1, 0.1, 0.1))
        if wine_prim.IsValid():
            xform = UsdGeom.Xformable(wine_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set((0.8, -0.3, 0.6))
            xform.AddScaleOp().Set((0.1, 0.1, 0.1))

        # self.create_point()

    def create_point(self):
        x_range = [-0.1, 0.3]
        y_range = [-0.5, 0.5]
        z_range = [0.3, 0.7]
        step = 0.1
        # 计算每个维度上的长度范围
        x_range = np.arange(x_range[0], x_range[1], step)
        y_range = np.arange(y_range[0], y_range[1], step)
        z_range = np.arange(z_range[0], z_range[1], step)

        yellow_mat_prim = prims_utils.get_prim_at_path("/Looks/YellowMat")
        material = UsdShade.Material(yellow_mat_prim)
        if not material:
            Printer.print_custom(f"Prim at {yellow_mat_prim} is not a valid UsdShade.Material.")

        num = 0
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    _prim_path = f"/World/envs/env_1/point/Sphere{num}"
                    if prims_utils.is_prim_path_valid(_prim_path):
                        # Printer.print_custom(prims_utils.is_prim_path_valid(_prim_path))
                        continue
                    else:
                        prim = prims_utils.create_prim(
                            prim_path=_prim_path,
                            prim_type="Sphere",
                            position=np.array([x, y, z]),
                            scale=np.array([0.01, 0.01, 0.01])
                        )
                        # UsdShade.MaterialBindingAPI.Apply(prim).Bind(red_material)
                        UsdShade.MaterialBindingAPI(prim).Bind(material)
                        # Printer.print_custom(prims_utils.is_prim_path_valid(_prim_path))
                    num += 1


def rosSpinThread(node):
    rclpy.spin(node)


def main():
    # 场景
    assets_root = get_assets_root_path()    # 确保 Omniverse Nucleus 连接成功
    if assets_root is None:
        Printer.print_normal("[ERROR]: 无法连接到 Omniverse Nucleus，请检查 Nucleus 服务器。")
        return
    else:
        Printer.print_normal("assets_root: " + assets_root)
    builder = Builder()

    # 仿真器
    sim_cfg = SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    world = World(stage_units_in_meters=1.0)
    world.reset()
    Printer.print_normal("[INFO]: Setup complete...")

    # ROS
    rclpy.init()
    rosNode = RosNode()
    spin_thread = Thread(target=rosSpinThread, args=(rosNode,))
    spin_thread.start()
    # 主线程中发送目标
    rosNode.sendGoal(5)

    # 物理仿真
    while simulation_app.is_running():
        sim.step()  # step

    # ROS
    rosNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    # close sim app
    simulation_app.close()
