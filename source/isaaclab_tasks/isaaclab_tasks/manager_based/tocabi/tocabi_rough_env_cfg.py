# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.managers import RewardTermCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##

# from isaaclab_assets.robots.cartpole import CARTPOLE_CFG  # isort:skip
from isaaclab_assets import Tocabi_CFG  # isort: skip



##
# Scene definition
##


@configclass
class TocabiSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = Tocabi_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##


# @configclass
# class ActionsCfg:
#     """Action specifications for the MDP."""

#     joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)

JOINT_NAMES = [
            ".*_HipYaw_Joint",
            ".*_HipRoll_Joint",
            ".*_HipPitch_Joint",
            ".*_Knee_Joint",
            ".*_AnklePitch_Joint",
            ".*_AnkleRoll_Joint",
            "Waist1_Joint",
            "Waist2_Joint",
            "Upperbody_Joint",
            ".*_Shoulder1_Joint",
            ".*_Shoulder2_Joint",
            ".*_Shoulder3_Joint",
            ".*_Armlink_Joint",
            ".*_Elbow_Joint",
            ".*_Forearm_Joint",
            ".*_Wrist1_Joint",
            ".*_Wrist2_Joint",
            "Neck_Joint",
            "Head_Joint"
        ]


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=JOINT_NAMES,
        scale=0.5,
        use_default_offset=True,
    )


# @configclass
# class TocabiActionsCfg:
#     joint_pos = mdp.TocabiActionCfg(
#         asset_name="robot", 
#         clip = {".*": (-1.0, 1.0)},
#         lower_joint_names=["L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint", "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint",
#                            "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint", "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint"],
#         upper_joint_names=["Waist1_Joint", "Waist2_Joint", "Upperbody_Joint",
#                            "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint", "L_Elbow_Joint", "L_Armlink_Joint", "L_Forearm_Joint", "L_Wrist1_Joint", "L_Wrist2_Joint",
#                            "Neck_Joint", "Head_Joint",
#                            "R_Shoulder1_Joint", "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Elbow_Joint", "R_Armlink_Joint", "R_Forearm_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint"],
#         pd_control=True,

#         p_gains = [2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0, 
#                    2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0, 
#                    6000.0, 10000.0, 10000.0, 
#                    400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0, 
#                    100.0, 100.0, 
#                    400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0],

#         d_gains = [15.0, 50.0, 20.0, 25.0, 24.0, 24.0, 
#                    15.0, 50.0, 20.0, 25.0, 24.0, 24.0, 
#                    200.0, 100.0, 100.0, 
#                    10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0, 
#                    3.0, 3.0, 
#                    10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0],

#         torque_limits= [333, 232, 263, 289, 222, 166,
#                         333, 232, 263, 289, 222, 166,
#                         303, 303, 303,
#                         64, 64, 64, 64, 23, 23, 10, 10,
#                         10, 10,
#                         64, 64, 64, 64, 23, 23, 10, 10],

#         joint_pos_limits = [(-0.3, 0.3), (-0.5, 0.5), (-1.0, 0.5), (-0.3, 1.2), (-0.8, 0.5), (-0.6, 0.6), 
#                             (-0.3, 0.3), (-0.5, 0.5), (-1.0, 0.5), (-0.3, 1.2), (-0.8, 0.5), (-0.6, 0.6)],

#         rand_torque_inj_range = (-0.0, 0.0),
#         rand_motor_scale_range = (0.8, 1.2)
#     )

# @configclass
# class ObservationsCfg:
#     """Observation specifications for the MDP."""

#     @configclass
#     class PolicyCfg(ObsGroup):
#         """Observations for policy group."""

#         # observation terms (order preserved)
#         joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
#         joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

#         def __post_init__(self) -> None:
#             self.enable_corruption = False
#             self.concatenate_terms = True

#     # observation groups
#     policy: PolicyCfg = PolicyCfg()

@configclass
class TocabiObservations:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        base_lin_vel = ObservationTermCfg(
            func=mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        base_ang_vel = ObservationTermCfg(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )
        projected_gravity = ObservationTermCfg(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObservationTermCfg(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        joint_pos = ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES)},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel = ObservationTermCfg(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES)},
            noise=Unoise(n_min=-1.5, n_max=1.5),
        )
        actions = ObservationTermCfg(func=mdp.last_action)
        height_scan = ObservationTermCfg(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # Observation groups:
    policy: PolicyCfg = PolicyCfg()



# @configclass
# class EventCfg:
#     """Configuration for events."""

#     # reset
#     reset_cart_position = EventTerm(
#         func=mdp.reset_joints_by_offset,
#         mode="reset",
#         params={
#             "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
#             "position_range": (-1.0, 1.0),
#             "velocity_range": (-0.5, 0.5),
#         },
#     )

#     reset_pole_position = EventTerm(
#         func=mdp.reset_joints_by_offset,
#         mode="reset",
#         params={
#             "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
#             "position_range": (-0.25 * math.pi, 0.25 * math.pi),
#             "velocity_range": (-0.25 * math.pi, 0.25 * math.pi),
#         },
#     )


# @configclass
# class RewardsCfg:
#     """Reward terms for the MDP."""

#     # (1) Constant running reward
#     alive = RewTerm(func=mdp.is_alive, weight=1.0)
#     # (2) Failure penalty
#     terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
#     # (3) Primary task: keep pole upright
#     pole_pos = RewTerm(
#         func=mdp.joint_pos_target_l2,
#         weight=-1.0,
#         params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]), "target": 0.0},
#     )
#     # (4) Shaping tasks: lower cart velocity
#     cart_vel = RewTerm(
#         func=mdp.joint_vel_l1,
#         weight=-0.01,
#         params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"])},
#     )
#     # (5) Shaping tasks: lower pole angular velocity
#     pole_vel = RewTerm(
#         func=mdp.joint_vel_l1,
#         weight=-0.005,
#         params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"])},
#     )

@configclass
class TocabiRewards:
    termination_penalty = RewardTermCfg(
        func=mdp.is_terminated,
        weight=-100.0,
    )
    track_lin_vel_xy_exp = RewardTermCfg(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewardTermCfg(
        func=mdp.track_ang_vel_z_world_exp,
        weight=1.0,
        params={
            "command_name": "base_velocity",
            "std": math.sqrt(0.25),
        },
    )
    feet_air_time = RewardTermCfg(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_AnkleRoll_Link"),
            "threshold": 0.8,
            "command_name": "base_velocity",
        },
    )
    feet_slide = RewardTermCfg(
        func=mdp.feet_slide,
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_AnkleRoll_Link"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_AnkleRoll_Link"),
        },
    )
    dof_torques_l2 = RewardTermCfg(
        func=mdp.joint_torques_l2,
        weight=-1.0e-6,
    )
    dof_acc_l2 = RewardTermCfg(
        func=mdp.joint_acc_l2,
        weight=-2.0e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES)},
    )
    action_rate_l2 = RewardTermCfg(
        func=mdp.action_rate_l2,
        weight=-0.008,
    )
    flat_orientation_l2 = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-2.5,
    )
    # stand_still = RewardTermCfg(
    #     func=mdp.stand_still_joint_deviation_l1,
    #     weight=-0.4,
    #     params={
    #         "command_name": "base_velocity",
    #         "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES),
    #     },
    # )
    # lin_vel_z_l2 = RewardTermCfg(
    #     func=mdp.lin_vel_z_l2,
    #     weight=-2.0,
    # )
    # ang_vel_xy_l2 = RewardTermCfg(
    #     func=mdp.ang_vel_xy_l2,
    #     weight=-0.1,
    # )
    # no_jumps = RewardTermCfg(
    #     func=mdp.desired_contacts,
    #     weight=-0.5,
    #     params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*_leg_toe_roll"])},
    # )
    # dof_pos_limits = RewardTermCfg(
    #     func=mdp.joint_pos_limits,
    #     weight=-1.0,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_leg_toe_roll", ".*_leg_toe_pitch", ".*_tarsus"])},
    # )
    # joint_deviation_hip_roll = RewardTermCfg(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*_leg_hip_roll")},
    # )
    # joint_deviation_hip_yaw = RewardTermCfg(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*_leg_hip_yaw")},
    # )
    # joint_deviation_knee = RewardTermCfg(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*_tarsus")},
    # )
    # joint_deviation_feet = RewardTermCfg(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_toe_a", ".*_toe_b"])},
    # )
    # joint_deviation_arms = RewardTermCfg(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.2,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", joint_names=".*_arm_.*"),
    #     },
    # )
    # undesired_contacts = RewardTermCfg(
    #     func=mdp.undesired_contacts,
    #     weight=-0.1,
    #     params={
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*_rod", ".*_tarsus"]),
    #         "threshold": 1.0,
    #     },
    # )


# @configclass
# class TerminationsCfg:
#     """Termination terms for the MDP."""

#     # (1) Time out
#     time_out = DoneTerm(func=mdp.time_out, time_out=True)
#     # (2) Cart out of bounds
#     cart_out_of_bounds = DoneTerm(
#         func=mdp.joint_pos_out_of_manual_limit,
#         params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]), "bounds": (-3.0, 3.0)},
#     )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)
    base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Pelvis_Link"]),
            "threshold": 1.0,
        },
    )
    base_orientation = TerminationTermCfg(
        func=mdp.bad_orientation,
        params={"limit_angle": 0.7},
    )


##
# Environment configuration
##


# @configclass
# class TocabiEnvCfg(ManagerBasedRLEnvCfg):
#     # Scene settings
#     scene: TocabiSceneCfg = TocabiSceneCfg(num_envs=4096, env_spacing=4.0)
#     # Basic settings
#     observations: ObservationsCfg = ObservationsCfg()
#     actions: ActionsCfg = ActionsCfg()
#     events: EventCfg = EventCfg()
#     # MDP settings
#     rewards: RewardsCfg = RewardsCfg()
#     terminations: TerminationsCfg = TerminationsCfg()

#     # Post initialization
#     def __post_init__(self) -> None:
#         """Post initialization."""
#         # general settings
#         self.decimation = 2
#         self.episode_length_s = 5
#         # viewer settings
#         self.viewer.eye = (8.0, 0.0, 5.0)
#         # simulation settings
#         self.sim.dt = 1 / 120
#         self.sim.render_interval = self.decimation

@configclass
class TocabiRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: TocabiRewards = TocabiRewards()
    observations: TocabiObservations = TocabiObservations()
    terminations: TerminationsCfg = TerminationsCfg()
    actions: ActionsCfg = ActionsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.decimation = 4
        self.sim.dt = 0.005

        # Scene
        self.scene.robot = Tocabi_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/Pelvis_Link"
        self.scene.contact_forces.history_length = self.decimation
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        # Events:
        self.events.add_base_mass.params["asset_cfg"] = SceneEntityCfg("robot", body_names="Pelvis_Link")
        self.events.base_external_force_torque.params["asset_cfg"] = SceneEntityCfg("robot", body_names="Pelvis_Link")
        # Don't randomize the initial joint positions because we have closed loops.
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        # remove COM randomization
        self.events.base_com = None

        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (-0.8, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.rel_standing_envs = 0.1
        self.commands.base_velocity.resampling_time_range = (3.0, 8.0)


@configclass
class TocabiRoughEnvCfg_PLAY(TocabiRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Make a smaller scene for play.
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # Spawn the robot randomly in the grid (instead of their terrain levels).
        self.scene.terrain.max_init_terrain_level = None
        # Reduce the number of terrains to save memory.
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # Disable randomization for play.
        self.observations.policy.enable_corruption = False
        # Remove random pushing.
        self.events.base_external_force_torque = None
        self.events.push_robot = None
