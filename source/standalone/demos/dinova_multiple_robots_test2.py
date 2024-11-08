# @Author: Clarence Chen
# @Date: 2024-11-07

"""
This script spawns two robots in the environment and controls them using different control modes.
Each robot is mounted with a camera and the images from the camera are accessed.
"""

import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--control_mode", type=str, default="velocity", help="Control mode for the robot. Choice, 'effort' or 'velocity' or 'position'.")

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
import cv2
import numpy as np

import omni.isaac.lab.envs.mdp as mdp
import omni.isaac.lab.sim as sim_utils

from omni.isaac.lab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns

from dinova_my import DINOVA_CONFIG, DINOVA_VELOCITY_CONFIG


@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    if args_cli.control_mode == "effort":
        joint_efforts1 = mdp.JointEffortActionCfg(asset_name="robot1", joint_names=["omni_joint_.*", "arm_joint_[1-6]",  "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"], scale=1.0)
        joint_efforts2 = mdp.JointEffortActionCfg(asset_name="robot2", joint_names=["omni_joint_.*", "arm_joint_[1-6]",  "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"], scale=1.0)
    elif args_cli.control_mode == "velocity":
        joint_velocities1 = mdp.JointVelocityActionCfg(asset_name="robot1", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"], scale=1.0)
        joint_velocities2 = mdp.JointVelocityActionCfg(asset_name="robot2", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"], scale=1.0)    
    else:
        joint_positions1 = mdp.JointPositionActionCfg(asset_name="robot1", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"])
        joint_positions2 = mdp.JointPositionActionCfg(asset_name="robot2", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"])


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos_rel1 = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot1")})
        joint_vel_rel1 = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot1")})
        
        joint_pos_rel2 = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot2")})
        joint_vel_rel2 = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot2")})

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""
    # on reset
    reset_joint_position1 = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot1", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"]),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_joint_position2 = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot2", joint_names=["omni_joint_.*", "arm_joint_[1-6]", "arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"]),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class DinovaSceneCfg(InteractiveSceneCfg):
    """Configuration for a Dinova scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)))

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

    # Dinova
    if args_cli.control_mode != "velocity": # position control or effort control
        robot1: ArticulationCfg = DINOVA_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot1")
        robot2: ArticulationCfg = DINOVA_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot2")
    else: # velocity control
        robot1: ArticulationCfg = DINOVA_VELOCITY_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot1")
        robot2: ArticulationCfg = DINOVA_VELOCITY_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot2")

    robot1.init_state.pos = (0.0, -1.0, 0.0)
    robot2.init_state.pos = (0.0, 1.0, 0.0)
    
    camera1 = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot1/chassis_link/front_cam",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.510, 0.0, 0.015), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    )

    camera2 = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot2/chassis_link/front_cam",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.510, 0.0, 0.015), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    )


@configclass
class DinovaEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the Dinova environment."""

    # Scene settings
    scene = DinovaSceneCfg(num_envs=args_cli.num_envs, env_spacing=5)
    
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
        self.decimation = 2  # env step every 2 sim steps: 100Hz / 2 = 50Hz
        # simulation settings
        self.sim.dt = 0.01  # sim step every 5ms: 100Hz


def main():
    """Main function."""
    # parse the arguments
    env_cfg = DinovaEnvCfg()

    # setup base environment
    env = ManagerBasedEnv(cfg=env_cfg)
    
    # simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")

            ## control the robot                
            if args_cli.control_mode == "effort":
                joint_efforts = torch.randn_like(env.action_manager.action)
                print("joint_efforts: ", joint_efforts)
                ## step the environment
                obs, _ = env.step(joint_efforts)
            elif args_cli.control_mode == "velocity":
                joint_velocities = np.zeros(22)  # Two robots with 11 joints each
                joint_velocities[0] = math.sin(count / 300.0)
                joint_velocities[1] = math.cos(count / 300.0)
                joint_velocities[3] = math.sin(count / 300.0)

                joint_velocities = torch.tensor([joint_velocities], device='cuda:0')
                print("joint_velocities: ", joint_velocities)

                ## step the environment
                obs, _ = env.step(joint_velocities)
            else:
                joint_init_pose = np.zeros(22) # Two robots with 11 joints each
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

            print(env.scene["camera1"])
            print("Received shape of rgb   image: ", env.scene["camera1"].data.output["rgb"].shape)
            print("Received shape of depth image: ", env.scene["camera1"].data.output["distance_to_image_plane"].shape)

            print(env.scene["camera2"])
            print("Received shape of rgb   image: ", env.scene["camera2"].data.output["rgb"].shape)
            print("Received shape of depth image: ", env.scene["camera2"].data.output["distance_to_image_plane"].shape)

            # # render the camera image
            rgb_image = env.scene["camera1"].data.output["rgb"]
            rgb_image = rgb_image.squeeze(0)
            rgb_image = rgb_image.cpu().numpy()
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            depth_image = env.scene["camera1"].data.output["distance_to_image_plane"]
            depth_image = depth_image.squeeze(0)
            depth_image = depth_image.cpu().numpy()
            max_val = np.max(depth_image)
            if max_val > 0 and max_val < 1000:  # Ensure max is not zero to avoid division by zero
                depth_image = (depth_image / max_val) * 255
            else:
                depth_image = np.zeros_like(depth_image)
            depth_image = depth_image.astype(np.uint8)

            print("Show image with cv2.imshow is not working in this environment")

            # update counter
            count += 1

    # close the environment
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
