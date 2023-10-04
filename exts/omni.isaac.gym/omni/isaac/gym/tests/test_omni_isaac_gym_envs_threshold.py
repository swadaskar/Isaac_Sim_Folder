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


def _test_cartpole_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 400.0
    assert ep_len >= 450.0
    if test_time:
        assert train_time <= 60.0


def _test_ant_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 4500.0
    assert ep_len >= 900.0
    if test_time:
        assert train_time <= 4.0 * 60


def _test_humanoid_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 4000.0
    assert ep_len >= 850.0
    if test_time:
        assert train_time <= 12.0 * 60


def _test_anymal_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 35.0
    assert ep_len >= 2000.0
    if test_time:
        assert train_time <= 10 * 60.0


def _test_anymal_terrain_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)
    terrain_level = utils._extract_feature(log_data, "Episode/terrain_level")

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)
    print("terrain_level:", terrain_level)

    assert reward >= 8.0
    assert ep_len >= 600.0
    if test_time:
        assert train_time <= 60 * 60.0
    assert terrain_level >= 3.0


def _test_ball_balance_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    assert reward >= 350.0
    assert ep_len >= 400.0
    if test_time:
        assert train_time <= 5 * 60.0


def _test_franka_cabinet_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 2000.0
    assert ep_len >= 480.0
    if test_time:
        assert train_time <= 7 * 60.0


def _test_ingenuity_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 4000.0
    assert ep_len >= 1900.0
    if test_time:
        assert train_time <= 5 * 60.0


def _test_quadcopter_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 1000.0
    assert ep_len >= 450.0
    if test_time:
        assert train_time <= 7 * 60.0


def _test_crazyflie_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)

    assert reward >= 1000.0
    assert ep_len >= 450.0
    if test_time:
        assert train_time <= 10 * 60.0


def _test_allegro_hand_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)
    consecutive_successes = utils._extract_feature(log_data, "consecutive_successes/iter")

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)
    print("success:", consecutive_successes)

    assert reward >= 1500.0
    assert ep_len >= 450.0
    if test_time:
        assert train_time <= 120 * 60.0
    assert consecutive_successes >= 5


def _test_shadow_hand_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)
    consecutive_successes = utils._extract_feature(log_data, "consecutive_successes/iter")

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)
    print("success:", consecutive_successes)

    assert reward >= 6000.0
    assert ep_len >= 500.0
    if test_time:
        assert train_time <= 60 * 60.0
    assert consecutive_successes >= 20


def _test_shadow_hand_dr_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)
    consecutive_successes = utils._extract_feature(log_data, "consecutive_successes/iter")

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)
    print("success:", consecutive_successes)

    assert reward >= 3000.0
    assert ep_len >= 450.0
    if test_time:
        assert train_time <= 1.2 * 60 * 60.0
    assert consecutive_successes >= 10


def _test_shadow_hand_openai_ff_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)
    consecutive_successes = utils._extract_feature(log_data, "consecutive_successes/iter")

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)
    print("success:", consecutive_successes)

    assert reward >= 2500.0
    assert ep_len >= 450.0
    if test_time:
        assert train_time <= 3 * 60 * 60.0
    assert consecutive_successes >= 10


def _test_shadow_hand_openai_lstm_train(experiment_name, test_time=True):
    log_data = utils._retrieve_logs(experiment_name)
    reward = utils._extract_reward(log_data)
    ep_len = utils._extract_episode_length(log_data)
    train_time = utils._extract_time(log_data)
    consecutive_successes = utils._extract_feature(log_data, "consecutive_successes/iter")

    print("reward:", reward)
    print("ep len:", ep_len)
    print("train_time:", train_time)
    print("success:", consecutive_successes)

    assert reward >= 6500.0
    assert ep_len >= 1300.0
    if test_time:
        assert train_time <= 3.5 * 60 * 60.0
    assert consecutive_successes >= 25


class TestOmniIsaacGymEnvsTrainThreshold(utils.OmniIsaacGymEnvsTestCase):
    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_cartpole_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Cartpole", self._sim_device, self._pipeline, 75)
        _test_cartpole_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_ant_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Ant", self._sim_device, self._pipeline, 300)
        _test_ant_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_humanoid_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Humanoid", self._sim_device, self._pipeline, 500)
        _test_humanoid_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_anymal_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Anymal", self._sim_device, self._pipeline, 500)
        _test_anymal_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_anymal_terrain_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "AnymalTerrain", self._sim_device, self._pipeline, 800)
        _test_anymal_terrain_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_ball_balance_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "BallBalance", self._sim_device, self._pipeline, 250)
        _test_ball_balance_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_franka_cabinet_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "FrankaCabinet", self._sim_device, self._pipeline, 300)
        _test_franka_cabinet_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_ingenuity_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Ingenuity", self._sim_device, self._pipeline, 400)
        _test_ingenuity_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_quadcopter_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Quadcopter", self._sim_device, self._pipeline, 500)
        _test_quadcopter_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_GG", "WEEKLY_THRESH_GGMT"], "Minimal"
    )
    async def test_crazyflie_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "Crazyflie", self._sim_device, self._pipeline, 500)
        _test_crazyflie_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_AH_GG", "WEEKLY_THRESH_AH_GGMT"],
        "AllegroHand",
    )
    async def test_allegro_hand_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "AllegroHand", self._sim_device, self._pipeline, 2000)
        _test_allegro_hand_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["DAILY_THRESH_SH_GG", "WEEKLY_THRESH_SH_GGMT"],
        "ShadowHand",
    )
    async def test_shadow_hand_train_gg(self):
        experiment_name = utils._run_rlgames_train(self._script, "ShadowHand", self._sim_device, self._pipeline, 1000)
        _test_shadow_hand_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT") in ["WEEKLY_THRESH_SH_DR_GG", "WEEKLY_THRESH_SH_DR_GGMT"],
        "ShadowHand DR",
    )
    async def test_shadow_hand_dr_train_gg(self):
        experiment_name = utils._run_rlgames_train(
            self._script, "ShadowHand", self._sim_device, self._pipeline, 1000, True
        )
        _test_shadow_hand_dr_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT")
        in ["WEEKLY_THRESH_SH_OPENAIFF_GG", "WEEKLY_THRESH_SH_OPENAIFF_GGMT"],
        "ShadowHand OpenAI FF",
    )
    async def test_shadow_hand_openai_ff_train_gg(self):
        experiment_name = utils._run_rlgames_train(
            self._script, "ShadowHandOpenAI_FF", self._sim_device, self._pipeline, 2000
        )
        _test_shadow_hand_openai_ff_train(experiment_name, self._test_time)

    @unittest.skipUnless(
        os.environ.get("ISAACSIM_OIGE_TEST_MODE", "ONCOMMIT")
        in ["WEEKLY_THRESH_SH_OPENAILSTM_GG", "WEEKLY_THRESH_SH_OPENAILSTM_GGMT"],
        "ShadowHand OpenAI LSTM",
    )
    async def test_shadow_hand_openai_lstm_train_gg(self):
        experiment_name = utils._run_rlgames_train(
            self._script, "ShadowHandOpenAI_LSTM", self._sim_device, self._pipeline, 2000
        )
        _test_shadow_hand_openai_lstm_train(experiment_name, self._test_time)


class TestOmniIsaacGymEnvsTrainThresholdGG(TestOmniIsaacGymEnvsTrainThreshold):
    def __init__(self, *args, **kwargs):
        super(TestOmniIsaacGymEnvsTrainThresholdGG, self).__init__(*args, **kwargs)
        self._script = utils.RLGAMES_SCRIPT
        self._sim_device = "gpu"
        self._pipeline = "gpu"
        self._test_time = True


class TestOmniIsaacGymEnvsTrainThresholdGGMT(TestOmniIsaacGymEnvsTrainThreshold):
    def __init__(self, *args, **kwargs):
        super(TestOmniIsaacGymEnvsTrainThresholdGGMT, self).__init__(*args, **kwargs)
        self._script = utils.RLGAMES_MT_SCRIPT
        self._sim_device = "gpu"
        self._pipeline = "gpu"
        self._test_time = True


class TestOmniIsaacGymEnvsTrainThresholdGC(TestOmniIsaacGymEnvsTrainThreshold):
    def __init__(self, *args, **kwargs):
        super(TestOmniIsaacGymEnvsTrainThresholdGC, self).__init__(*args, **kwargs)
        self._script = utils.RLGAMES_SCRIPT
        self._sim_device = "gpu"
        self._pipeline = "cpu"
        self._test_time = False


class TestOmniIsaacGymEnvsTrainThresholdGCMT(TestOmniIsaacGymEnvsTrainThreshold):
    def __init__(self, *args, **kwargs):
        super(TestOmniIsaacGymEnvsTrainThresholdGCMT, self).__init__(*args, **kwargs)
        self._script = utils.RLGAMES_MT_SCRIPT
        self._sim_device = "gpu"
        self._pipeline = "cpu"
        self._test_time = False
