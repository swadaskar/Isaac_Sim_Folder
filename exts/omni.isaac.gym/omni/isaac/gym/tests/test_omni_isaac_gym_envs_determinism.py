# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import unittest
import omni.kit

import omni.isaac.gym.tests.utils as utils

CHECK_ITERATIONS = 100
NUM_RUNS = 3


def _check_determinism(task, sim_device, pipeline, dr=False):
    prev_reward = -1
    for i in range(NUM_RUNS):
        experiment_name = utils._run_rlgames_train(
            utils.RLGAMES_SCRIPT, task, sim_device, pipeline, CHECK_ITERATIONS, dr
        )
        log_data = utils._retrieve_logs(experiment_name)
        reward = log_data["rewards/iter"][-1][1]
        if i > 0:
            if reward != prev_reward:
                return False
        else:
            prev_reward = reward

    return True


class TestOmniIsaacGymEnvsDeterminismCC(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_determinism_cc(self):
        self.assertTrue(_check_determinism("Cartpole", "cpu", "cpu"))

    async def test_ant_determinism_cc(self):
        self.assertTrue(_check_determinism("Ant", "cpu", "cpu"))

    async def test_humanoid_determinism_cc(self):
        self.assertTrue(_check_determinism("Humanoid", "cpu", "cpu"))

    async def test_anymal_determinism_cc(self):
        self.assertTrue(_check_determinism("Anymal", "cpu", "cpu"))

    async def test_anymal_terrain_determinism_cc(self):
        self.assertTrue(_check_determinism("AnymalTerrain", "cpu", "cpu"))

    async def test_ball_balance_determinism_cc(self):
        self.assertTrue(_check_determinism("BallBalance", "cpu", "cpu"))

    async def test_franka_cabinet_determinism_cc(self):
        self.assertTrue(_check_determinism("FrankaCabinet", "cpu", "cpu"))

    async def test_ingenuity_determinism_cc(self):
        self.assertTrue(_check_determinism("Ingenuity", "cpu", "cpu"))

    async def test_quadcopter_determinism_cc(self):
        self.assertTrue(_check_determinism("Quadcopter", "cpu", "cpu"))

    async def test_crazyflie_determinism_cc(self):
        self.assertTrue(_check_determinism("Crazyflie", "cpu", "cpu"))

    async def test_allegro_hand_determinism_cc(self):
        self.assertTrue(_check_determinism("AllegroHand", "cpu", "cpu"))

    async def test_shadow_hand_determinism_cc(self):
        self.assertTrue(_check_determinism("ShadowHand", "cpu", "cpu"))

    async def test_shadow_hand_dr_determinism_cc(self):
        self.assertTrue(_check_determinism("ShadowHand", "cpu", "cpu", True))

    async def test_shadow_hand_openai_ff_determinism_cc(self):
        self.assertTrue(_check_determinism("ShadowHandOpenAI_FF", "cpu", "cpu"))

    async def test_shadow_hand_openai_lstm_determinism_cc(self):
        self.assertTrue(_check_determinism("ShadowHandOpenAI_LSTM", "cpu", "cpu"))


class TestOmniIsaacGymEnvsDeterminismGC(utils.OmniIsaacGymEnvsTestCase):
    async def test_cartpole_determinism_gc(self):
        self.assertTrue(_check_determinism("Cartpole", "gpu", "cpu"))

    async def test_ant_determinism_gc(self):
        self.assertTrue(_check_determinism("Ant", "gpu", "cpu"))

    async def test_humanoid_determinism_gc(self):
        self.assertTrue(_check_determinism("Humanoid", "gpu", "cpu"))

    async def test_anymal_determinism_gc(self):
        self.assertTrue(_check_determinism("Anymal", "gpu", "cpu"))

    async def test_anymal_terrain_determinism_gc(self):
        self.assertTrue(_check_determinism("AnymalTerrain", "gpu", "cpu"))

    async def test_ball_balance_determinism_gc(self):
        self.assertTrue(_check_determinism("BallBalance", "gpu", "cpu"))

    async def test_franka_cabinet_determinism_gc(self):
        self.assertTrue(_check_determinism("FrankaCabinet", "gpu", "cpu"))

    async def test_ingenuity_determinism_gc(self):
        self.assertTrue(_check_determinism("Ingenuity", "gpu", "cpu"))

    async def test_quadcopter_determinism_gc(self):
        self.assertTrue(_check_determinism("Quadcopter", "gpu", "cpu"))

    async def test_crazyflie_determinism_gc(self):
        self.assertTrue(_check_determinism("Crazyflie", "gpu", "cpu"))

    async def test_allegro_hand_determinism_gc(self):
        self.assertTrue(_check_determinism("AllegroHand", "gpu", "cpu"))

    async def test_shadow_hand_determinism_gc(self):
        self.assertTrue(_check_determinism("ShadowHand", "gpu", "cpu"))

    async def test_shadow_hand_dr_determinism_gc(self):
        self.assertTrue(_check_determinism("ShadowHand", "gpu", "cpu", True))

    async def test_shadow_hand_openai_ff_determinism_gc(self):
        self.assertTrue(_check_determinism("ShadowHandOpenAI_FF", "gpu", "cpu"))

    async def test_shadow_hand_openai_lstm_determinism_gc(self):
        self.assertTrue(_check_determinism("ShadowHandOpenAI_LSTM", "gpu", "cpu"))


class TestOmniIsaacGymEnvsDeterminismGG(utils.OmniIsaacGymEnvsTestCase):
    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_cartpole_determinism_gg(self):
        self.assertTrue(_check_determinism("Cartpole", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_ant_determinism_gg(self):
        self.assertTrue(_check_determinism("Ant", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_humanoid_determinism_gg(self):
        self.assertTrue(_check_determinism("Humanoid", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_anymal_determinism_gg(self):
        self.assertTrue(_check_determinism("Anymal", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_anymal_terrain_determinism_gg(self):
        self.assertTrue(_check_determinism("AnymalTerrain", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_ball_balance_determinism_gg(self):
        self.assertTrue(_check_determinism("BallBalance", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_franka_cabinet_determinism_gg(self):
        self.assertTrue(_check_determinism("FrankaCabinet", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_ingenuity_determinism_gg(self):
        self.assertTrue(_check_determinism("Ingenuity", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_quadcopter_determinism_gg(self):
        self.assertTrue(_check_determinism("Quadcopter", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_crazyflie_determinism_gg(self):
        self.assertTrue(_check_determinism("Crazyflie", "gpu", "gpu"))

    @unittest.skipUnless(os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_GG", "Minimal")
    async def test_allegro_hand_determinism_gg(self):
        self.assertTrue(_check_determinism("AllegroHand", "gpu", "gpu"))

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_SH_GG", "ShadowHand"
    )
    async def test_shadow_hand_determinism_gg(self):
        self.assertTrue(_check_determinism("ShadowHand", "gpu", "gpu"))

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_SH_GG", "ShadowHand"
    )
    async def test_shadow_hand_dr_determinism_gg(self):
        self.assertTrue(_check_determinism("ShadowHand", "gpu", "gpu", True))

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_SH_GG", "ShadowHand"
    )
    async def test_shadow_hand_openai_ff_determinism_gg(self):
        self.assertTrue(_check_determinism("ShadowHandOpenAI_FF", "gpu", "gpu"))

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") == "WEEKLY_DETERMINISM_SH_GG", "ShadowHand"
    )
    async def test_shadow_hand_openai_lstm_determinism_gg(self):
        self.assertTrue(_check_determinism("ShadowHandOpenAI_LSTM", "gpu", "gpu"))
