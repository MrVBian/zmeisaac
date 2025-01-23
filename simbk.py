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
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.assets import Articulation
import omni.usd

# Constants for your scene and robot
USD_SCENE = "omniverse://localhost/Projects/Scene/bzw/pourwine3.usd"
ROBOT_PRIM_PATH = "/World/envs/env_0/zmebot_description"  # Path to your robot in the scene


def open_stage(usd_path: str):
    """Loads a USD scene."""
    omni.usd.get_context().open_stage(usd_path)
    print(f"[INFO]: Opened USD stage: {usd_path}")


def setup_robot_articulation(prim_path: str) -> Articulation:
    """Instantiates an articulation for a robot based on an existing USD prim."""
    # Create an Articulation instance using the prim path of the robot
    robot_cfg = Articulation.Config(prim_path=prim_path)
    robot = Articulation(cfg=robot_cfg)
    print(f"[INFO]: Articulation created for robot at {prim_path}")
    return robot


def run_simulator(sim: SimulationContext, robot: Articulation):
    """Runs the simulation loop."""
    sim_dt = sim.get_physics_dt()
    count = 0

    # Simulation loop
    while simulation_app.is_running():
        # Reset simulation state
        if count % 500 == 0:
            count = 0
            # Reset robot state
            root_state = robot.data.default_root_state.clone()
            robot.write_root_state_to_sim(root_state)
            # Set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos += torch.rand_like(joint_pos) * 0.1
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            print("[INFO]: Resetting robot state...")

        # Apply random joint efforts for demonstration
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        robot.set_joint_effort_target(efforts)
        robot.write_data_to_sim()

        # Perform a simulation step
        sim.step()
        count += 1

        # Update internal buffers
        robot.update(sim_dt)


def main():
    """Main function."""
    # Load the USD scene
    open_stage(USD_SCENE)

    # Load simulation context
    sim_cfg = SimulationContext.Config(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    # Set main camera position
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Instantiate the robot articulation
    robot = setup_robot_articulation(ROBOT_PRIM_PATH)

    # Initialize simulation
    sim.reset()
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, robot)


if __name__ == "__main__":
    # Run the main function
    main()
    # Close simulation app
    simulation_app.close()