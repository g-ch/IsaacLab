# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different legged robots.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/demos/quadrupeds.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--control_mode", type=str, default="velocity", help="Control mode for the robot. Choice, 'effort' or 'velocity' or 'position'.")
parser.add_argument("--robot_num", type=int, default=2, help="Number of robots to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import math
import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation

from dinova_my import DINOVA_CONFIG, DINOVA_VELOCITY_CONFIG


def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_cols = np.floor(np.sqrt(num_origins))
    num_rows = np.ceil(num_origins / num_cols)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    return env_origins.tolist()


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=args_cli.robot_num, spacing=1.25)

    scene_entities = {}
    for i in range(args_cli.robot_num):
        # Origin with Anymal B
        prim_utils.create_prim(f"/World/Origin{i}", "Xform", translation=origins[i])
        # -- Robot
        name = f"dinova_{i}"
        if args_cli.control_mode == "velocity":
            dinova = Articulation(DINOVA_VELOCITY_CONFIG.replace(prim_path=f"/World/Origin{i}/Robot"))
        else:
            dinova = Articulation(DINOVA_CONFIG.replace(prim_path=f"/World/Origin{i}/Robot"))
        
        scene_entities[name] = dinova
        
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # Simulate physics
    while simulation_app.is_running():
        #### reset
        # if count % 200 == 0:
        #     # reset counters
        #     sim_time = 0.0
        #     count = 0
        #     # reset robots
        #     for index, robot in enumerate(entities.values()):
        #         # root state
        #         root_state = robot.data.default_root_state.clone()
        #         root_state[:, :3] += origins[index]
        #         robot.write_root_state_to_sim(root_state)
        #         # joint state
        #         joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
        #         robot.write_joint_state_to_sim(joint_pos, joint_vel)
        #         # reset the internal state
        #         robot.reset()
        #     print("[INFO]: Resetting robots state...")

        #### apply default actions to the quadrupedal robots
        for index, robot in enumerate(entities.values()):

            if args_cli.control_mode == "position":
                # generate random joint positions            
                if index % 2 == 0:
                    joint_pos_target = robot.data.default_joint_pos
                else:
                    joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
                # apply action to the robot
                robot.set_joint_position_target(joint_pos_target)
            elif args_cli.control_mode == "velocity":
                joint_velocities = np.zeros(11)
                joint_velocities[0] = math.sin(count / 300.0)
                joint_velocities[1] = math.cos(count / 300.0)
                joint_velocities[3] = math.sin(count / 300.0)

                joint_velocities = torch.tensor([joint_velocities], device='cuda:0')
                # apply action to the robot
                robot.set_joint_velocity_target(joint_velocities)
            else: # "effort"
                joint_efforts = torch.randn_like(robot.data.default_joint_pos)
                # apply action to the robot
                robot.set_joint_effort_target(joint_efforts)

            # Get the joint positions and velocities
            joint_pos, joint_vel = robot.read_joint_state_from_sim()
            

            # write data to sim
            robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""

    # Initialize the simulation context
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01))
    # Set main camera
    sim.set_camera_view(eye=[2.5, 2.5, 2.5], target=[0.0, 0.0, 0.0])
    # design scene
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
