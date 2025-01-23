import argparse
from omni.isaac.lab.app import AppLauncher
# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with an articulation.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
"""Rest everything follows."""

import torch
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext

##
# Pre-defined configs
##
from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip

###### region bzw
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
print("project_root: ", current_dir)
from global_variables import *
from printer import Printer
from rosnode import *
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg
######


def design_scene() -> tuple[dict, list[list[float]]]:
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

    # Articulation
    cartpole_cfg = CARTPOLE_CFG.copy()
    cartpole_cfg.prim_path = "/World/Origin.*/Robot"
    cartpole = Articulation(cfg=cartpole_cfg)
    
    Printer.print_normal("loading")
    prim_path = "/World/zmebot_description"
    ZMECFG = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=ROBOT_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
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
            "right_link": ImplicitActuatorCfg(
                joint_names_expr=["right_joint."],
                effort_limit=400.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=0.0,
            ),
            "left_link": ImplicitActuatorCfg(
                joint_names_expr=["left_joint."],
                effort_limit=400.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=0.0,
            ),
            "base": ImplicitActuatorCfg(
                joint_names_expr=[".*base.*"], effort_limit=400.0, velocity_limit=100.0, stiffness=0.0, damping=0.0
            ),
        },
    )
    ZMECFG.prim_path = prim_path
    zmebot = Articulation(cfg=ZMECFG)
    # cfg = sim_utils.UsdFileCfg(usd_path=ROBOT_USD)
    # prim_path = "/World/zmebot_description"
    # # use the `func` reference in the config class
    # cfg.func(prim_path, cfg)
    Printer.print_normal("loaded")


    # return the scene information
    scene_entities = {"cartpole": cartpole, "zmebot": zmebot}
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["cartpole"]
    zmebot = entities["zmebot"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += origins
            robot.write_root_state_to_sim(root_state)
            # set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos += torch.rand_like(joint_pos) * 0.1
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            robot.reset()

            root_state = zmebot.data.default_root_state.clone()
            zmebot.write_root_state_to_sim(root_state)
            joint_pos, joint_vel = zmebot.data.default_joint_pos.clone(), zmebot.data.default_joint_vel.clone()
            zmebot.write_joint_state_to_sim(joint_pos, joint_vel)
            zmebot.reset()
            print("[INFO]: Resetting robot state...")
        # Apply random action
        # -- generate random joint efforts
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        # -- apply action to the robot
        robot.set_joint_effort_target(efforts)
        # -- write data to sim
        robot.write_data_to_sim()

        # -- generate random joint efforts
        # efforts = torch.randn_like(zmebot.data.joint_pos) * 1
        # Printer.print_normal(f"zme joint_names: {zmebot.data.joint_names}  len:{len(zmebot.data.joint_names)}")
        # Printer.print_normal(f"zme efforts: {efforts}  shade:{efforts.shape}")
        # zmebot.set_joint_effort_target(efforts)

        position = torch.randn_like(zmebot.data.joint_pos_target)
        Printer.print_normal(f"shape: {position.shape}")
        for i, name in enumerate(zmebot.data.joint_names):
            name_str = str(name)
            if name_str.startswith(("left_joint", "right_joint")):
                pass
            else:
                position[0, i] = 0.0
        zmebot.set_joint_position_target(position)
        zmebot.write_data_to_sim()


        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        robot.update(sim_dt)
        zmebot.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
