# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import unittest
import omni.kit

import omni.isaac.gym.tests.utils as utils


class TestOmniIsaacGymEnvsLaunchCC(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Cartpole", "cpu", "cpu", 5)

    async def test_ant_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Ant", "cpu", "cpu", 5)

    async def test_humanoid_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Humanoid", "cpu", "cpu", 5)

    async def test_anymal_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Anymal", "cpu", "cpu", 5)

    async def test_anymal_terrain_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "AnymalTerrain", "cpu", "cpu", 5)

    async def test_ball_balance_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "BallBalance", "cpu", "cpu", 5)

    async def test_franka_cabinet_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "FrankaCabinet", "cpu", "cpu", 5)

    async def test_ingenuity_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Ingenuity", "cpu", "cpu", 5)

    async def test_quadcopter_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Quadcopter", "cpu", "cpu", 5)

    async def test_crazyflie_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Crazyflie", "cpu", "cpu", 5)

    async def test_allegro_hand_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "AllegroHand", "cpu", "cpu", 5)

    async def test_shadow_hand_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHand", "cpu", "cpu", 5)

    async def test_shadow_hand_dr_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHand", "cpu", "cpu", 5, True)

    async def test_shadow_hand_openai_ff_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_FF", "cpu", "cpu", 5)

    async def test_shadow_hand_openai_lstm_train_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_LSTM", "cpu", "cpu", 5)


class TestOmniIsaacGymEnvsLaunchGC(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Cartpole", "gpu", "cpu", 5)

    async def test_ant_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Ant", "gpu", "cpu", 5)

    async def test_humanoid_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Humanoid", "gpu", "cpu", 5)

    async def test_anymal_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Anymal", "gpu", "cpu", 5)

    async def test_anymal_terrain_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "AnymalTerrain", "gpu", "cpu", 5)

    async def test_ball_balance_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "BallBalance", "gpu", "cpu", 5)

    async def test_franka_cabinet_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "FrankaCabinet", "gpu", "cpu", 5)

    async def test_ingenuity_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Ingenuity", "gpu", "cpu", 5)

    async def test_quadcopter_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Quadcopter", "gpu", "cpu", 5)

    async def test_crazyflie_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Crazyflie", "gpu", "cpu", 5)

    async def test_allegro_hand_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "AllegroHand", "gpu", "cpu", 5)

    async def test_shadow_hand_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "cpu", 5)

    async def test_shadow_hand_dr_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "cpu", 5, True)

    async def test_shadow_hand_openai_ff_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_FF", "gpu", "cpu", 5)

    async def test_shadow_hand_openai_lstm_train_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_LSTM", "gpu", "cpu", 5)


class TestOmniIsaacGymEnvsLaunchGG(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Cartpole", "gpu", "gpu", 5)

    async def test_ant_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Ant", "gpu", "gpu", 5)

    async def test_humanoid_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Humanoid", "gpu", "gpu", 5)

    async def test_anymal_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Anymal", "gpu", "gpu", 5)

    async def test_anymal_terrain_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "AnymalTerrain", "gpu", "gpu", 5)

    async def test_ball_balance_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "BallBalance", "gpu", "gpu", 5)

    async def test_franka_cabinet_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "FrankaCabinet", "gpu", "gpu", 5)

    async def test_ingenuity_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Ingenuity", "gpu", "gpu", 5)

    async def test_quadcopter_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Quadcopter", "gpu", "gpu", 5)

    async def test_crazyflie_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "Crazyflie", "gpu", "gpu", 5)

    async def test_allegro_hand_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "AllegroHand", "gpu", "gpu", 5)

    async def test_shadow_hand_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "gpu", 5)

    async def test_shadow_hand_dr_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "gpu", 5, True)

    async def test_shadow_hand_openai_ff_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_FF", "gpu", "gpu", 5)

    async def test_shadow_hand_openai_lstm_train_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_LSTM", "gpu", "gpu", 5)


class TestOmniIsaacGymEnvsLaunchCCMT(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Cartpole", "cpu", "cpu", 5)

    async def test_ant_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Ant", "cpu", "cpu", 5)

    async def test_humanoid_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Humanoid", "cpu", "cpu", 5)

    async def test_anymal_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Anymal", "cpu", "cpu", 5)

    async def test_anymal_terrain_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "AnymalTerrain", "cpu", "cpu", 5)

    async def test_ball_balance_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "BallBalance", "cpu", "cpu", 5)

    async def test_franka_cabinet_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "FrankaCabinet", "cpu", "cpu", 5)

    async def test_ingenuity_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Ingenuity", "cpu", "cpu", 5)

    async def test_quadcopter_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Quadcopter", "cpu", "cpu", 5)

    async def test_crazyflie_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Crazyflie", "cpu", "cpu", 5)

    async def test_allegro_hand_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "AllegroHand", "cpu", "cpu", 5)

    async def test_shadow_hand_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "cpu", "cpu", 5)

    async def test_shadow_hand_dr_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "cpu", "cpu", 5, True)

    async def test_shadow_hand_openai_ff_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_FF", "cpu", "cpu", 5)

    async def test_shadow_hand_openai_lstm_train_mt_cc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_LSTM", "cpu", "cpu", 5)


class TestOmniIsaacGymEnvsLaunchGCMT(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Cartpole", "gpu", "cpu", 5)

    async def test_ant_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Ant", "gpu", "cpu", 5)

    async def test_humanoid_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Humanoid", "gpu", "cpu", 5)

    async def test_anymal_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Anymal", "gpu", "cpu", 5)

    async def test_anymal_terrain_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "AnymalTerrain", "gpu", "cpu", 5)

    async def test_ball_balance_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "BallBalance", "gpu", "cpu", 5)

    async def test_franka_cabinet_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "FrankaCabinet", "gpu", "cpu", 5)

    async def test_ingenuity_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Ingenuity", "gpu", "cpu", 5)

    async def test_quadcopter_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Quadcopter", "gpu", "cpu", 5)

    async def test_crazyflie_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Crazyflie", "gpu", "cpu", 5)

    async def test_allegro_hand_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "AllegroHand", "gpu", "cpu", 5)

    async def test_shadow_hand_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "cpu", 5)

    async def test_shadow_hand_dr_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "cpu", 5, True)

    async def test_shadow_hand_openai_ff_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_FF", "gpu", "cpu", 5)

    async def test_shadow_hand_openai_lstm_train_mt_gc(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_LSTM", "gpu", "cpu", 5)


class TestOmniIsaacGymEnvsLaunchGGMT(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Cartpole", "gpu", "gpu", 5)

    async def test_ant_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Ant", "gpu", "gpu", 5)

    async def test_humanoid_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Humanoid", "gpu", "gpu", 5)

    async def test_anymal_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Anymal", "gpu", "gpu", 5)

    async def test_anymal_terrain_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "AnymalTerrain", "gpu", "gpu", 5)

    async def test_ball_balance_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "BallBalance", "gpu", "gpu", 5)

    async def test_franka_cabinet_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "FrankaCabinet", "gpu", "gpu", 5)

    async def test_ingenuity_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Ingenuity", "gpu", "gpu", 5)

    async def test_quadcopter_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Quadcopter", "gpu", "gpu", 5)

    async def test_crazyflie_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "Crazyflie", "gpu", "gpu", 5)

    async def test_allegro_hand_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "AllegroHand", "gpu", "gpu", 5)

    async def test_shadow_hand_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "gpu", 5)

    async def test_shadow_hand_dr_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "gpu", 5, True)

    async def test_shadow_hand_openai_ff_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_FF", "gpu", "gpu", 5)

    async def test_shadow_hand_openai_lstm_train_mt_gg(self):
        experiment_name = utils._run_rlgames_train(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_LSTM", "gpu", "gpu", 5)
