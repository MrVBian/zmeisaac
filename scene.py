import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
print("project_root: ", current_dir)
from global_variables import *
from printer import Printer

import numpy as np
from math import atan2, asin
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.prims as prims_utils # tranform
from omni.isaac.core.utils.stage import get_current_stage


class Scene:
    def __init__(self):
        self._primDic = {}

        # self._primDic["table"]  = add_reference_to_stage(USD_TABLE, USD_TABLE_TAR)
        # self._primDic["Cup"]    = add_reference_to_stage(USD_CUP, USD_CUP_TAR)
        # self._primDic["Winep"]  = add_reference_to_stage(USD_WINE, USD_WINE_TAR)
        # table_prim  = self._primDic["table"]
        # cup_prim    = self._primDic["Cup"]
        # wine_prim   = self._primDic["Winep"]
        # if table_prim.IsValid():
        #     xform = UsdGeom.Xformable(table_prim)
        #     xform.ClearXformOpOrder()
        #     xform.AddTranslateOp().Set((0.8, 0, 0))
        #     xform.AddRotateXYZOp().Set((0, 0, 90))
        #     xform.AddScaleOp().Set((0.01, 0.01, 0.01))
        # if cup_prim.IsValid():
        #     xform = UsdGeom.Xformable(cup_prim)
        #     xform.ClearXformOpOrder()
        #     xform.AddTranslateOp().Set((0.8, 0.15, 0.62))
        #     xform.AddScaleOp().Set((0.1, 0.1, 0.1))
        # if wine_prim.IsValid():
        #     xform = UsdGeom.Xformable(wine_prim)
        #     xform.ClearXformOpOrder()
        #     xform.AddTranslateOp().Set((0.8, -0.3, 0.6))
        #     xform.AddScaleOp().Set((0.1, 0.1, 0.1))

        self.stage = get_current_stage()
        self.InitMats()
        self.CreatePoints()
        Printer.print_normal("loaded")
        
    def design_scene(self) -> tuple[dict, list[list[float]]]:
        """Designs the scene."""
        # Ground-plane
        cfg = sim_utils.GroundPlaneCfg()
        cfg.func("/World/defaultGroundPlane", cfg)
        # Lights
        cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
        cfg.func("/World/Light", cfg)

        # Create separate groups called "Origin1", "Origin2"
        # Each group will have a robot in it
        origins = [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]
        # Origin 1
        prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
        # Origin 2
        prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])


        Printer.print_normal("loading")
        prim_path = "/World/envs/env_0/zmebot_description"
        ZMECFG = ArticulationCfg(
            spawn=sim_utils.UsdFileCfg(
                usd_path=ROBOT_USD,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    rigid_body_enabled=True,
                    angular_damping=0.05,
                    max_linear_velocity=1e9,
                    max_angular_velocity=5729,
                    max_depenetration_velocity=3.0,
                    enable_gyroscopic_forces=True,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    articulation_enabled=True,
                    enabled_self_collisions=False,
                    solver_position_iteration_count=32,
                    solver_velocity_iteration_count=1,
                    sleep_threshold=0.005,
                    stabilization_threshold=0.001,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.0), joint_pos={item: 0.0 for item in [
                    'rotate_base_joint', 'lifting_base_joint', 'left_font_steering_joint1',
                    'left_rear_steering_joint3', 'left_whee_joint', 'right_font_steering_joint2',
                    'right_rear_steering_joint4', 'right_whee_joint', 'gimbal_joint1',
                    'left_joint1', 'right_joint1', 'left_font_drive_joint1',
                    'left_rear_drive_joint3', 'right_font_drive_joint2', 'right_rear_drive_joint4',
                    'gimbal_joint2', 'left_joint2', 'right_joint2', 'left_joint3',
                    'right_joint3', 'left_joint4', 'right_joint4', 'left_joint5',
                    'right_joint5', 'left_joint6', 'right_joint6', 'left_joint7',
                    'right_joint7', 'left_finger_joint1', 'left_finger_joint2',
                    'll_tool_joint3', 'lr_tool_joint3', 'right_finger_joint1',
                    'right_finger_joint2', 'rl_tool_joint3', 'rr_tool_joint3',
                    'lr_tool_joint2', 'll_tool_joint2', 'rr_tool_joint2', 'rl_tool_joint2'
                ]}
            ),
            actuators={
                "right_joint": ImplicitActuatorCfg(
                    joint_names_expr=["right_joint."],
                    effort_limit=1000.0,
                    velocity_limit=90.0,
                    damping=17453293764608,
                    stiffness=17453293764608,
                    # effort_limit=10000.0,
                    # velocity_limit=1.0,
                    # stiffness=100000.0,
                    # damping=100000.0,
                ),
                "left_joint": ImplicitActuatorCfg(
                    joint_names_expr=["left_joint."],
                    effort_limit=1000.0,
                    velocity_limit=90.0,
                    damping=17453293764608,
                    stiffness=17453293764608,
                ),
                "gimbal_joint": ImplicitActuatorCfg(
                    joint_names_expr=["gimbal_joint."],
                    effort_limit=1000.0,
                    velocity_limit=90.0,
                    damping=17453293764608,
                    stiffness=17453293764608,
                ),
                "base": ImplicitActuatorCfg(
                    joint_names_expr=[".*base.*"], 
                    effort_limit=1000.0,
                    velocity_limit=90.0,
                    damping=17453293764608,
                    stiffness=17453293764608,
                ),
                "left_gripper": ImplicitActuatorCfg(
                    joint_names_expr=["left_finger_joint."], 
                    effort_limit=1000.0,
                    velocity_limit=90.0,
                    damping=17453293764608,
                    stiffness=17453293764608,
                ),
                "right_gripper": ImplicitActuatorCfg(
                    joint_names_expr=["right_finger_joint."], 
                    effort_limit=1000.0,
                    velocity_limit=90.0,
                    damping=17453293764608,
                    stiffness=17453293764608,
                ),
                # "wheel_joint": ImplicitActuatorCfg(
                #     joint_names_expr=[".*_whee_joint"], 
                #     effort_limit=10000000,
                #     stiffness=1000000.0,
                #     damping=0.0,
                # ),
            },
        )
        ZMECFG.prim_path = prim_path
        zmebot = Articulation(cfg=ZMECFG)
        # cfg = sim_utils.UsdFileCfg(usd_path=ROBOT_USD)
        # prim_path = "/World/zmebot_description"
        # # use the `func` reference in the config class
        # cfg.func(prim_path, cfg)


        # return the scene information
        scene_entities = {"zmebot": zmebot}
        return scene_entities, origins

    def GetPrim(self, name):
        if name in self._primDic:
            return self._primDic[name]
        else:
            Printer.print_warning(f"{name}的prim不存在")

    def InitMats(self):
        self.redMat = UsdShade.Material.Define(self.stage, "/Materials/Red")
        redShader = UsdShade.Shader.Define(self.stage, "/Shaders/RedShader")
        redShader.CreateIdAttr("UsdPreviewSurface")
        redShader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.0, 0.0))  # 红色
        red_output = redShader.CreateOutput("surface", Sdf.ValueTypeNames.Token)  # 获取着色器的表面输出
        self.redMat.CreateSurfaceOutput().ConnectToSource(red_output)  # 连接到材质的输出

        # 创建黄色材质
        self.yellowMat = UsdShade.Material.Define(self.stage, "/Materials/Yellow")
        yellowShader = UsdShade.Shader.Define(self.stage, "/Shaders/YellowShader")
        yellowShader.CreateIdAttr("UsdPreviewSurface")
        yellowShader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 0.0))  # 黄色
        yellow_output = yellowShader.CreateOutput("surface", Sdf.ValueTypeNames.Token)  # 获取着色器的表面输出
        self.yellowMat.CreateSurfaceOutput().ConnectToSource(yellow_output)  # 连接到材质的输出

        # 创建绿色材质
        self.greenMat = UsdShade.Material.Define(self.stage, "/Materials/Green")
        greenShader = UsdShade.Shader.Define(self.stage, "/Shaders/GreenShader")
        greenShader.CreateIdAttr("UsdPreviewSurface")
        greenShader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 1.0, 0.0))  # 绿色
        green_output = greenShader.CreateOutput("surface", Sdf.ValueTypeNames.Token)  # 获取着色器的表面输出
        self.greenMat.CreateSurfaceOutput().ConnectToSource(green_output)  # 连接到材质的输出

    def CreatePoints(self):
        self.points = []
        x_range = [-0.1+0.5, 0.3+1.0]
        y_range = [-0.5-0.1, 0.5+0.1]
        z_range = [0.3, 0.7]
        step = 0.05
        # 计算每个维度上的长度范围
        x_range = np.arange(x_range[0], x_range[1], step)
        y_range = np.arange(y_range[0], y_range[1], step)
        z_range = np.arange(z_range[0], z_range[1], step)

        num = 0
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    _prim_path = f"/World/envs/env_1/point/Sphere{num}"
                    if prims_utils.is_prim_path_valid(_prim_path):
                        continue
                    else:
                        prim = prims_utils.create_prim(
                            prim_path=_prim_path,
                            prim_type="Sphere",
                            position=np.array([x, y, z]),
                            scale=np.array([0.01, 0.01, 0.01])
                        )
                        self.points.append(prim)
                        UsdShade.MaterialBindingAPI(prim).Bind(self.yellowMat)
                    num += 1

    def PointsToCoor(self):
        # Printer.print_normal(f"base coor: " + str(self.GetWorldTransform(BASE_LINK).ExtractTranslation()))
        for x in range(0, 5):
            pointPrimPath = f"/World/envs/env_1/point/Sphere{x}"
            # Printer.print_normal(f"{x}coor: " + str(self.GetWorldTransform(pointPrimPath).ExtractTranslation()))

            relativeTranslation, relativeRotation, relativeEulerRotation = self.ComputeRelativeTransform(BASE_LINK, pointPrimPath)
            # 输出相对位置和旋转
            Printer.print_normal(f"Sphere{x}相对位置: {tuple(round(coord, 1) for coord in (relativeTranslation * 1000))}")
            # Printer.print_normal(f"相对旋转 (四元数): {relativeRotation}")
            # Printer.print_normal(f"相对旋转 (欧拉角XYZ): {relativeEulerRotation}")

    def GetWorldTransform(self, primPath):
        """ 获取 prim 的世界变换矩阵 """
        stage = self.stage
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