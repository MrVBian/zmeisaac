from omni.isaac.lab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser(description="Pour wine stage.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# args_cli.headless = True
# app_launcher = AppLauncher(args_cli, settings={"rendering/preset": "Medium"})
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
from math import atan2, asin
from pxr import Usd, UsdGeom, UsdShade, Sdf
from pxr import UsdPhysics, PhysxSchema, Gf
import omni
import omni.isaac.core.utils.prims as prims_utils # tranform
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, get_current_stage


import torch
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

import omni.physx as _physx
######

class Scene:
    def __init__(self):
        self._primDic = {}
        self._artDicart = {}
        # 获取 Dynamic Control 接口
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        open_stage(USD_SCENE)
        # 获取当前打开的 Stage
        self.stage = get_current_stage()





        # 加载资源
        Printer.print_normal("loading assets")
        self._primDic["Cup"]    = add_reference_to_stage(USD_CUP, USD_CUP_TAR)
        self._primDic["Winep"]  = add_reference_to_stage(USD_WINE, USD_WINE_TAR)
        # cup_prim = prims_utils.get_prim_at_path(USD_CUP_TAR)
        # wine_prim = prims_utils.get_prim_at_path(USD_WINE_TAR)
        cup_prim    = self._primDic["Cup"]
        wine_prim   = self._primDic["Winep"]
        if cup_prim.IsValid():
            xform = UsdGeom.Xformable(cup_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set((0.8, 0.15, 0.62))
            xform.AddScaleOp().Set((0.1, 0.1, 0.1))
        if wine_prim.IsValid():
            xform = UsdGeom.Xformable(wine_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set((0.8, -0.3, 0.6))
            xform.AddScaleOp().Set((0.1, 0.1, 0.1))

        self.AddRigidBody(cup_prim)
        self.AddRigidBody(wine_prim)
        Printer.print_normal(f"[Success] Added rigidbody and collision")

        self.CreatePoints()
        Printer.print_normal("loaded assets")

    def GetPrim(self, name):
        if name in self._primDic:
            return self._primDic[name]
        else:
            Printer.print_warning(f"{name}的prim不存在")
    
    def GetArt(self, robotName="zmebot_description"):
        if robotName in self._artDicart:
            return self._artDicart[robotName]
                    
        # 检查 prim 是否存在
        prim_path = ROBOT+"/world"
        # 获取机械臂 Articulation 句柄
        art = self.dc.get_articulation(prim_path)
        if art == _dynamic_control.INVALID_HANDLE:
            Printer.print_error(f" {robotName}无法获取 articulation 句柄")
            # return
        # 机械臂的 DOF 数量
        dofsNum = self.dc.get_articulation_dof_count(art)
        if dofsNum == 0:
            Printer.print_error(f"{robotName}未检测到DOF")
            # return
        # art = Articulation(prim_path="/World/envs/env_0/zmebot_description/world", name="zmebot_description")
        self._artDicart["zmebot_description"] = art

        Printer.print_normal(f"{robotName} Dof nums: {dofsNum}")
        # return art


        # 创建 Articulation 对象
        articulation = Articulation(prim_path=prim_path)
        articulation.initialize()

        # 检查关节名称是否加载成功
        if articulation.dof_names is None:
            Printer.print_error("Failed to initialize articulation. Check the model and scene.")
        else:
            Printer.print_normal(f"Articulation DOF Names: {articulation.dof_names}")
        return art, articulation


    def AddRigidBody(self, prim):
        # 确保 Prim 具有 `PhysicsMassAPI`
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(1.0)  # 设定质量
        mass_api.CreateDensityAttr().Set(1)  # 设定密度（可选）

        # 添加刚体属性
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)

    def CreatePoints(self):
        x_range = [-0.1, 0.3]
        y_range = [-0.5, 0.5]
        z_range = [0.3, 0.7]
        step = 0.1
        # 计算每个维度上的长度范围
        x_range = np.arange(x_range[0], x_range[1], step)
        y_range = np.arange(y_range[0], y_range[1], step)
        z_range = np.arange(z_range[0], z_range[1], step)

        yellow_mat_prim = prims_utils.get_prim_at_path("/Looks/YellowMat")
        self.yelMat = UsdShade.Material(yellow_mat_prim)
        self.redMat = UsdShade.Material(prims_utils.get_prim_at_path("/Looks/RedMat"))
        self.greMat = UsdShade.Material(prims_utils.get_prim_at_path("/Looks/GreenMat"))
        if not self.yelMat:
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
                        UsdShade.MaterialBindingAPI(prim).Bind(self.yelMat)
                        # Printer.print_custom(prims_utils.is_prim_path_valid(_prim_path))
                    num += 1

    def ChangePointColor(self, primPath, color):
        UsdShade.MaterialBindingAPI(prims_utils.get_prim_at_path(primPath)).Bind(color)
        # ChangePointColor("", self.redMat)

    def PointsToCoor(self):
        # Printer.print_normal(f"base coor: " + str(self.GetWorldTransform(BASE_LINK).ExtractTranslation()))
        for x in range(0, 5):
            pointPrimPath = f"/World/envs/env_1/point/Sphere{x}"
            # Printer.print_normal(f"{x}coor: " + str(self.GetWorldTransform(pointPrimPath).ExtractTranslation()))

            relativeTranslation, relativeRotation, relativeEulerRotation = self.ComputeRelativeTransform(BASE_LINK, pointPrimPath)
            # 输出相对位置和旋转
            Printer.print_normal(f"Sphere{x}相对位置: {relativeTranslation * 1000}")
            # Printer.print_normal(f"相对旋转 (四元数): {relativeRotation}")
            # Printer.print_normal(f"相对旋转 (欧拉角XYZ): {relativeEulerRotation}")

    ### region utils
    def GetWorldTransform(self, primPath):
        """ 获取 prim 的世界变换矩阵 """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(primPath)
        xform = UsdGeom.Xformable(prim)
        
        if not xform:
            raise ValueError(f"Prim {primPath} 无效或不可变换")
        
        worldTransform = xform.ComputeLocalToWorldTransform(0)
        return worldTransform
    
    def MatrixToEuler(self, matrix):
        """从 Gf.Matrix4d 提取欧拉角 (roll, pitch, yaw)，假定旋转顺序为 XYZ"""
        # 将旋转矩阵转换为欧拉角
        sy = (matrix[0][0]**2 + matrix[1][0]**2)**0.5

        singular = sy < 1e-6  # 判断是否接近奇异解

        if not singular:
            x = atan2(matrix[2][1], matrix[2][2])   # Roll
            y = asin(-matrix[2][0])                 # Pitch
            z = atan2(matrix[1][0], matrix[0][0])   # Yaw
        else:
            x = atan2(-matrix[1][2], matrix[1][1])  # Roll
            y = asin(-matrix[2][0])                 # Pitch
            z = 0                                   # Yaw

        return x, y, z  # 返回欧拉角 (roll, pitch, yaw)

    def ComputeRelativeTransformWithScale(self, basePath, targetPath):
        """ 计算 targetPath 在 basePath 坐标系下的相对位置和旋转 """
        baseTransform = self.GetWorldTransform(basePath)
        targetTransform = self.GetWorldTransform(targetPath)

        # Printer.print_normal("旋转：" + str(self.MatrixToEuler(baseTransform)))
        # Printer.print_normal("旋转：" + str(self.MatrixToEuler(targetTransform)))

        # 计算 baseTransform 的逆矩阵（世界到 BASE_LINK 坐标变换）
        baseInvTransform = baseTransform.GetInverse()
        # 计算 target 在 BASE_LINK 坐标系下的变换
        relativeTransform = baseInvTransform * targetTransform

        # 提取相对位置（平移）
        relativeTranslation = relativeTransform.ExtractTranslation()
        # 提取相对旋转（四元数）
        relativeRotation = relativeTransform.ExtractRotation().GetQuaternion()
        # 提取相对旋转（欧拉角）
        relativeEulerRotation = self.MatrixToEuler(relativeTransform.ExtractRotationMatrix())

        return relativeTranslation, relativeRotation, relativeEulerRotation
    
    def ComputeRelativeTransform(self, basePath, targetPath):
        """计算 targetPath 在 basePath 坐标系下的相对位置和旋转（忽略缩放）"""
        baseTransform = self.GetWorldTransform(basePath)
        targetTransform = self.GetWorldTransform(targetPath)

        # 提取平移部分（去除缩放）
        basePosition = baseTransform.ExtractTranslation()
        targetPosition = targetTransform.ExtractTranslation()
        # 提取旋转矩阵（去除缩放影响）
        baseRotationMatrix = baseTransform.ExtractRotationMatrix()
        targetRotationMatrix = targetTransform.ExtractRotationMatrix()

        # 计算相对旋转矩阵（target 在 base 坐标系下的旋转）
        relativeTransform = baseRotationMatrix.GetInverse() * targetRotationMatrix

        # 计算相对位置（转换到 base 坐标系）
        relativeTranslation = baseRotationMatrix.GetInverse() * (targetPosition - basePosition)
        # 旋转矩阵转换为四元数
        relativeRotation = relativeTransform.ExtractRotation().GetQuaternion()
        # 提取相对旋转（欧拉角）
        relativeEulerRotation = self.MatrixToEuler(relativeTransform)

        return relativeTranslation, relativeRotation, relativeEulerRotation


def RosSpinThread(node):
    rclpy.spin(node)


def RunSimulator(sim: sim_utils.SimulationContext, scene: Scene, rosNode: RosNode):
    """运行模拟循环。"""
    # 提取场景实体
    # art = scene.GetArt()
    art, articulation = scene.GetArt()
    dc = _dynamic_control.acquire_dynamic_control_interface()
    dofsCot = dc.get_articulation_dof_count(art)
    jointHandle = []
    jointName = []
    initJointPositions = []
    # ctlJointName = {}
    # for i in range(1, 8):
    #     ctlJointName[f"left_joint{i}"] = True
    #     ctlJointName[f"right_joint{i}"] = True
    # ctlJointName["rotate_base_joint"] = True
    # ctlJointName["lifting_base_joint"] = True
    # for i in range(1, 3):
    #     ctlJointName[f"gimbal_joint{i}"] = True
    #     ctlJointName[f"left_finger_joint{i}"] = True
    #     ctlJointName[f"right_finger_joint{i}"] = True

    for i in range(dofsCot):
        handle = dc.get_articulation_dof(art, i)
        name = dc.get_dof_name(handle)
        # pos = dc.get_dof_position(handle)
        jointName.append(name)
        jointHandle.append(handle)
        # initJointPositions.append(pos)
        # jointHandle[name] = handle

        print(f"Sim Joint {i} name: {name}")

    sim_dt = sim.get_physics_dt()           # 定义模拟步长
    Printer.print_normal(f"模拟步长{sim_dt}")

    # 模拟循环
    while simulation_app.is_running():
        # 执行步骤
        sim.step(render=True)

        if rosNode.syncJointClock == 1:
            rosNode.syncJointClock = 2
            # 创建目标关节角度的字典
            jointTargets = {}
            # 读取 ROS 2 消息中的目标关节值
            for joint in rosNode.syncJointData.left_arm_joints:
                jointTargets[joint.joint_name.data] = joint.joint_position
            for joint in rosNode.syncJointData.right_arm_joints:
                jointTargets[joint.joint_name.data] = joint.joint_position

            targetPositions = []

            # 如果在目标关节角度字典中，则使用目标角度，否则使用当前角度
            for i in range(len(jointName)):
                if jointName[i] in jointTargets:
                    targetPositions.append(jointTargets[jointName[i]])
                else:
                    targetPositions.append(0)  # 维持当前角度


            prim_path = ROBOT+"/world"
            omni.kit.commands.execute(
                "ChangeProperty",
                prop_path=f"{prim_path}.joint_positions",
                value=targetPositions,
                prev=None,  # 如果需要撤销，可以设置 prev 的值
            )

            action = ArticulationAction(joint_positions=np.array(targetPositions))
            articulation.apply_action(action)
            
            # Printer.print_normal(f"{type(articulation)}")
            # Printer.print_normal(f"dof_names: {len(articulation.dof_names)}")

            rosNode.syncJointClock = 0


def main():
    # 场景
    assets_root = get_assets_root_path()    # 确保 Omniverse Nucleus 连接成功
    if assets_root is None:
        Printer.print_normal("[ERROR]: 无法连接到 Omniverse Nucleus，请检查 Nucleus 服务器。")
        return
    else:
        Printer.print_normal("assets_root: " + assets_root)
    scene = Scene()

    # 仿真器
    sim_cfg = SimulationCfg(dt=0.01, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    world = World(stage_units_in_meters=1.0)
    world.reset()
    Printer.print_normal("[INFO]: Setup complete...")
    

    # ROS
    rclpy.init()
    rosNode = RosNode()
    spinThread = Thread(target=RosSpinThread, args=(rosNode,))
    spinThread.start()
    # 主线程中发送目标
    # rosNode.SendGoal(0)
    # builder.PointsToCoor(rosNode)

    # 物理仿真
    # while simulation_app.is_running():
    #     sim.step()  # step
    RunSimulator(sim, scene, rosNode)

    # ROS
    Printer.print_normal("Shutting down...")
    rosNode.destroy_node()
    rclpy.shutdown()
    spinThread.join()


if __name__ == "__main__":
    main()
    # close sim app
    simulation_app.close()