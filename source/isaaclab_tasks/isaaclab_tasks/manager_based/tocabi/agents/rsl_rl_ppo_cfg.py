# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


# @configclass
# class PPORunnerCfg(RslRlOnPolicyRunnerCfg):
#     num_steps_per_env = 16
#     max_iterations = 150
#     save_interval = 50
#     experiment_name = "cartpole_direct"
#     policy = RslRlPpoActorCriticCfg(
#         init_noise_std=1.0,
#         actor_obs_normalization=False,
#         critic_obs_normalization=False,
#         actor_hidden_dims=[32, 32],
#         critic_hidden_dims=[32, 32],
#         activation="elu",
#     )
#     algorithm = RslRlPpoAlgorithmCfg(
#         value_loss_coef=1.0,
#         use_clipped_value_loss=True,
#         clip_param=0.2,
#         entropy_coef=0.005,
#         num_learning_epochs=5,
#         num_mini_batches=4,
#         learning_rate=1.0e-3,
#         schedule="adaptive",
#         gamma=0.99,
#         lam=0.95,
#         desired_kl=0.01,
#         max_grad_norm=1.0,
#     )

@configclass
class TocabiRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 50
    experiment_name = "tocabi_rough"
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        # learning_rate_min = 1.0e-5,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
)

@configclass
class TocabiFlatPPORunnerCfg(TocabiRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 3000
        self.experiment_name = "tocabi_flat"

        # self.policy.actor_hidden_dims = [256, 256, 128]
        # self.policy.critic_hidden_dims = [256, 256, 128]
        # self.policy.actor_hidden_dims = [256, 256]
        # self.policy.critic_hidden_dims = [256, 256]
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        # self.algorithm.learning_rate = 1.0e-3
        self.algorithm.schedule= "adaptive"


@configclass
class TocabiRoughPPORunnerCfg_PLAY(TocabiRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        # Make a smaller scene for play.
        self.scene.num_envs = 64
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

@configclass
class TocabiFlatEnvCfg_PLAY(TocabiFlatPPORunnerCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        # Make a smaller scene for play.
        self.scene.num_envs = 64
        self.scene.env_spacing = 2.5
        # Disable randomization for play.
        self.observations.policy.enable_corruption = False
        # Remove random pushing.
        self.events.base_external_force_torque = None
        self.events.push_robot = None
