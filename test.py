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
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext

###### region bzw
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
print("project_root: ", current_dir)
from global_variables import *
from printer import Printer
from rosnode import *
from scene import *

import numpy as np
import random
######


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor, scene: Scene):
    """Runs the simulation loop."""
    zmebot = entities["zmebot"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    taskNum = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # reset counter
            count = 0

            randomNum = random.randint(0, 1)
            if randomNum == 0:
                UsdShade.MaterialBindingAPI(scene.points[taskNum]).Bind(scene.greenMat)
            else:
                UsdShade.MaterialBindingAPI(scene.points[taskNum]).Bind(scene.redMat)
            taskNum += 1

            root_state = zmebot.data.default_root_state.clone()
            zmebot.write_root_state_to_sim(root_state)
            joint_pos, joint_vel = zmebot.data.default_joint_pos.clone(), zmebot.data.default_joint_vel.clone()
            zmebot.write_joint_state_to_sim(joint_pos, joint_vel)
            zmebot.reset()
            print("[INFO]: Resetting robot state...")
        # # Apply random action
        # # -- generate random joint efforts
        # efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        # # -- apply action to the robot
        # robot.set_joint_effort_target(efforts)
        # # -- write data to sim
        # robot.write_data_to_sim()

        # -- generate random joint efforts
        # efforts = torch.randn_like(zmebot.data.joint_pos) * 1
        # Printer.print_normal(f"zme joint_names: {zmebot.data.joint_names}  len:{len(zmebot.data.joint_names)}")
        # Printer.print_normal(f"zme efforts: {efforts}  shade:{efforts.shape}")
        # zmebot.set_joint_effort_target(efforts)

        position = torch.randn_like(zmebot.data.joint_pos_target)
        # Printer.print_normal(f"shape: {position.shape}")
        for i, name in enumerate(zmebot.data.joint_names):
            name_str = str(name)
            if name_str.startswith(("left_joint", "right_joint")):
                if name_str[-1] == "1":
                    position[0, i] = -0.965
                elif name_str[-1] == "2":
                    position[0, i] = -0.583
                elif name_str[-1] == "3":
                    position[0, i] = 1.222
                elif name_str[-1] == "4":
                    position[0, i] = -2.062
                elif name_str[-1] == "5":
                    position[0, i] = -0.962
                elif name_str[-1] == "6":
                    position[0, i] = -1.612
                elif name_str[-1] == "7":
                    position[0, i] = 0.000
            elif name_str.startswith(("left_finger_joint", "right_finger_joint")):
                position[0, i] = 1.156
            elif name_str.endswith("_whee_joint"):
                position[0, i] = 5
            else:
                position[0, i] = 0.0
        zmebot.set_joint_position_target(position)
        zmebot.write_data_to_sim()


        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        zmebot.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    # sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene = Scene()
    scene_entities, scene_origins = scene.design_scene()
    scene.PointsToCoor()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
