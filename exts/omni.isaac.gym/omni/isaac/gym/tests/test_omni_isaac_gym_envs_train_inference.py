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


class TestOmniIsaacGymEnvsTestGG(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Cartpole", "gpu", "gpu")

    async def test_ant_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Ant", "gpu", "gpu")

    async def test_humanoid_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Humanoid", "gpu", "gpu")

    async def test_anymal_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Anymal", "gpu", "gpu")

    async def test_anymal_terrain_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "AnymalTerrain", "gpu", "gpu")

    async def test_ball_balance_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "BallBalance", "gpu", "gpu")

    async def test_franka_cabinet_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "FrankaCabinet", "gpu", "gpu")

    async def test_ingenuity_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Ingenuity", "gpu", "gpu")

    async def test_quadcopter_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Quadcopter", "gpu", "gpu")

    async def test_crazyflie_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Crazyflie", "gpu", "gpu")

    async def test_allegro_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "AllegroHand", "gpu", "gpu")

    async def test_shadow_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "gpu")

    async def test_shadow_hand_dr_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "gpu", dr=True)

    async def test_shadow_hand_openai_ff_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_FF", "gpu", "gpu")

    async def test_shadow_hand_openai_lstm_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "ShadowHandOpenAI_LSTM", "gpu", "gpu")


class TestOmniIsaacGymEnvsTestGGMT(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Cartpole", "gpu", "gpu")

    async def test_ant_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Ant", "gpu", "gpu")

    async def test_humanoid_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Humanoid", "gpu", "gpu")

    async def test_anymal_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Anymal", "gpu", "gpu")

    async def test_anymal_terrain_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "AnymalTerrain", "gpu", "gpu")

    async def test_ball_balance_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "BallBalance", "gpu", "gpu")

    async def test_franka_cabinet_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "FrankaCabinet", "gpu", "gpu")

    async def test_ingenuity_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Ingenuity", "gpu", "gpu")

    async def test_quadcopter_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Quadcopter", "gpu", "gpu")

    async def test_crazyflie_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Crazyflie", "gpu", "gpu")

    async def test_allegro_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "AllegroHand", "gpu", "gpu")

    async def test_shadow_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "gpu")

    async def test_shadow_hand_dr_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "gpu", dr=True)

    async def test_shadow_hand_openai_ff_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_FF", "gpu", "gpu")

    async def test_shadow_hand_openai_lstm_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "ShadowHandOpenAI_LSTM", "gpu", "gpu")


class TestOmniIsaacGymEnvsTestPreTrainedGG(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Cartpole", "gpu", "gpu", pretrained=True)

    async def test_ant_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Ant", "gpu", "gpu", pretrained=True)

    async def test_humanoid_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Humanoid", "gpu", "gpu", pretrained=True)

    async def test_anymal_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Anymal", "gpu", "gpu", pretrained=True)

    async def test_anymal_terrain_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "AnymalTerrain", "gpu", "gpu", pretrained=True)

    async def test_ball_balance_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "BallBalance", "gpu", "gpu", pretrained=True)

    async def test_franka_cabinet_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "FrankaCabinet", "gpu", "gpu", pretrained=True)

    async def test_ingenuity_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Ingenuity", "gpu", "gpu", pretrained=True)

    async def test_quadcopter_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Quadcopter", "gpu", "gpu", pretrained=True)

    async def test_crazyflie_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "Crazyflie", "gpu", "gpu", pretrained=True)

    async def test_allegro_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "AllegroHand", "gpu", "gpu", pretrained=True)

    async def test_shadow_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_SCRIPT, "ShadowHand", "gpu", "gpu", pretrained=True)


class TestOmniIsaacGymEnvsTestPreTrainedGGMT(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Cartpole", "gpu", "gpu", pretrained=True)

    async def test_ant_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Ant", "gpu", "gpu", pretrained=True)

    async def test_humanoid_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Humanoid", "gpu", "gpu", pretrained=True)

    async def test_anymal_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Anymal", "gpu", "gpu", pretrained=True)

    async def test_anymal_terrain_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "AnymalTerrain", "gpu", "gpu", pretrained=True)

    async def test_ball_balance_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "BallBalance", "gpu", "gpu", pretrained=True)

    async def test_franka_cabinet_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "FrankaCabinet", "gpu", "gpu", pretrained=True)

    async def test_ingenuity_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Ingenuity", "gpu", "gpu", pretrained=True)

    async def test_quadcopter_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Quadcopter", "gpu", "gpu", pretrained=True)

    async def test_crazyflie_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "Crazyflie", "gpu", "gpu", pretrained=True)

    async def test_allegro_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "AllegroHand", "gpu", "gpu", pretrained=True)

    async def test_shadow_hand_test_gg(self):
        utils._run_rlgames_test(utils.RLGAMES_MT_SCRIPT, "ShadowHand", "gpu", "gpu", pretrained=True)
