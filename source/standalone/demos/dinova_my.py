'''
@author: Clarence
@description: This is a custom extension for the Dinova My robot. It is a subclass of the Robot class.
@Date: 2024.11.7
'''

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg


#Available strings: ['omni_joint_x', 'omni_joint_y', 'omni_joint_theta', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'arm_left_finger_bottom_joint', 'arm_right_finger_bottom_joint']

DINOVA_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/cc/chg_ws/isaac_lab/scenes/dinova_usd/dinova.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "omni_joint_x": 0.0,
            "omni_joint_y": 0.0,
            "omni_joint_theta": 0.0,
            "arm_joint_1": 0.0,
            "arm_joint_2": 0.0,
            "arm_joint_3": 0.0,
            "arm_joint_4": 0.0,
            "arm_joint_5": 0.0,
            "arm_joint_6": 0.0,
            "arm_left_finger_bottom_joint": 0.0,
            "arm_right_finger_bottom_joint": 0.0,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["omni_joint_x", "omni_joint_y", "omni_joint_theta"],
            velocity_limit=100.0,
            effort_limit=1000.0,
            # effort_limit={
            #     "omni_joint_x": 10,
            #     "omni_joint_y": 10,
            #     "omni_joint_theta": 10,
            # },
            stiffness=1e15,
            damping=1e5,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["arm_joint_[1-6]"],
            velocity_limit=10.0,
            effort_limit=20.0,
            stiffness=1e15,
            damping=1e5,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"],
            velocity_limit=10.0,
            effort_limit=20.0,
            stiffness=1e15,
            damping=1e5,
        ),
    },

)



DINOVA_VELOCITY_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/cc/chg_ws/isaac_lab/scenes/dinova_usd/dinova.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "omni_joint_x": 0.0,
            "omni_joint_y": 0.0,
            "omni_joint_theta": 0.0,
            "arm_joint_1": 0.0,
            "arm_joint_2": 0.0,
            "arm_joint_3": 0.0,
            "arm_joint_4": 0.0,
            "arm_joint_5": 0.0,
            "arm_joint_6": 0.0,
            "arm_left_finger_bottom_joint": 0.0,
            "arm_right_finger_bottom_joint": 0.0,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["omni_joint_x", "omni_joint_y", "omni_joint_theta"],
            velocity_limit=5.0,
            effort_limit=200.0,
            # effort_limit={
            #     "omni_joint_x": 10,
            #     "omni_joint_y": 10,
            #     "omni_joint_theta": 10,
            # },
            stiffness=0,
            damping=1e5,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["arm_joint_[1-6]"],
            velocity_limit=10.0,
            effort_limit=20.0,
            stiffness=0,
            damping=1e5,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["arm_left_finger_bottom_joint", "arm_right_finger_bottom_joint"],
            velocity_limit=10.0,
            effort_limit=20.0,
            stiffness=0,
            damping=1e5,
        ),
    },

)