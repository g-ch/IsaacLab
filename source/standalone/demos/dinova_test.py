# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to create a simple environment with a cartpole. It combines the concepts of
scene, action, observation and event managers to create an environment.
"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--control_mode", type=str, default="position", help="Control mode for the robot. Choice, 'effort' or 'velocity' or 'position'.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import torch

import omni.isaac.lab.envs.mdp as mdp
import omni.isaac.lab.sim as sim_utils

from omni.isaac.lab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveSceneCfg


#from omni.isaac.lab_tasks.manager_based.classic.cartpole.cartpole_env_cfg import CartpoleSceneCfg
from dinova_my import DINOVA_CONFIG, DINOVA_VELOCITY_CONFIG

import numpy as np

@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    if args_cli.control_mode == "effort":
        joint_efforts = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["omni_joint_.*", "arm_joint_[1-6]",  "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"], scale=1.0)
    elif args_cli.control_mode == "velocity":
        joint_velocities = mdp.JointVelocityActionCfg(asset_name="robot", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"], scale=1.0)
    else:
        joint_positions = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"])


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""
    # on reset
    reset_joint_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"]),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class DinovaSceneCfg(InteractiveSceneCfg):
    """Configuration for a Dinova scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # Dinova
    if args_cli.control_mode != "velocity":
        robot: ArticulationCfg = DINOVA_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    else:
        robot: ArticulationCfg = DINOVA_VELOCITY_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

@configclass
class DinovaEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the Dinova environment."""

    # Scene settings
    scene = DinovaSceneCfg(num_envs=1024, env_spacing=2.5)
    # Basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        # simulation settings
        self.sim.dt = 0.01  # sim step every 5ms: 100Hz


def main():
    """Main function."""
    # parse the arguments
    env_cfg = DinovaEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # setup base environment
    env = ManagerBasedEnv(cfg=env_cfg)

    
    # simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            # if count % 300 == 0:
            #     count = 0
            #     env.reset()
            #     print("-" * 80)
            #     print("[INFO]: Resetting environment...")

            ## sample random actions
                
            if args_cli.control_mode == "effort":
                joint_efforts = torch.randn_like(env.action_manager.action)
                print("joint_efforts: ", joint_efforts)
                ## step the environment
                obs, _ = env.step(joint_efforts)
            elif args_cli.control_mode == "velocity":
                # joint_velocities = torch.randn_like(env.action_manager.action)
                # print("joint_velocities: ", joint_velocities)

                joint_velocities = np.zeros(11)
                joint_velocities[0] = math.sin(count / 300.0)
                joint_velocities[1] = math.cos(count / 300.0)
                joint_velocities[3] = math.sin(count / 300.0)

                joint_velocities = torch.tensor([joint_velocities], device='cuda:0')
                print("joint_velocities: ", joint_velocities)

                ## step the environment
                obs, _ = env.step(joint_velocities)
            else:
                joint_init_pose = np.zeros(11)
                # random_pose_offset = np.random.uniform(-0.1, 0.1, 11)
                joint_positions_values = joint_init_pose # + random_pose_offset
                joint_positions_values[0] = math.sin(count / 100.0) * 5
                joint_positions_values[1] = math.cos(count / 100.0) * 5
                joint_positions_values[2] = math.sin(count / 200.0)
                joint_positions_values[3] = math.sin(count / 200.0)
                joint_positions_values[4] = math.sin(count / 200.0)

                joint_positions = torch.tensor([joint_positions_values], device='cuda:0')
                print("joint_positions: ", joint_positions)
                obs, _ = env.step(joint_positions)

            # print current orientation of all joints
            #print("[Env 0]: joint[1]: ", obs["policy"][0][1].item())
            print(obs["policy"][0])

            # update counter
            count += 1

    # close the environment
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
