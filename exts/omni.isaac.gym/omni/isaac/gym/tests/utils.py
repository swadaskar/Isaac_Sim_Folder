# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import carb
import unittest
import glob
import sys
import subprocess
import shutil
import stat
import torch
import platform
import psutil
import omni.kit
import numpy as np


def _is_windows():
    plat_os = platform.system().lower()
    return "windows" in plat_os


# get path to python .sh/bat using app folder path
PYTHON_EXE = (
    os.path.abspath(os.path.join(carb.tokens.get_tokens_interface().resolve("${app}") + "/../"))
    + "/python"
    + carb.tokens.get_tokens_interface().resolve("${shell_ext}")
)

REPO_PATH = os.path.join(carb.tokens.get_tokens_interface().resolve("${temp}"), "omniisaacgymenvs")

RLGAMES_SCRIPT = "rlgames_train"
RLGAMES_MT_SCRIPT = "rlgames_train_mt"


def _make_folder_writable(func, path, exc_info):
    os.chmod(path, stat.S_IWRITE)
    os.remove(path)


def _delete_repo_folder(repo_path):
    if os.path.exists(repo_path):
        shutil.rmtree(repo_path, onerror=_make_folder_writable)


def _run_rlgames_train(script, task, sim_device, pipeline, max_iterations=0, dr=False):
    os.chdir(os.path.join(REPO_PATH, "omniisaacgymenvs"))
    cmd = [
        PYTHON_EXE,
        f"scripts/{script}.py",
        "headless=True",
        f"task={task}",
        f"sim_device={sim_device}",
        f"pipeline={pipeline}",
        "seed=42",
    ]
    if dr:
        cmd.append(f"task.domain_randomization.randomize=True")
    if max_iterations > 0:
        cmd.append(f"max_iterations={max_iterations}")

    experiment_name = f"{task}_{sim_device}_{pipeline}"
    if script == RLGAMES_MT_SCRIPT:
        experiment_name += "_mt"
        cmd.append("mt_timeout=900")
    if dr:
        experiment_name += "_dr"
    cmd.append(f"experiment={experiment_name}")

    subprocess.check_call(cmd)

    return experiment_name


def _kill_process(p):
    parent = psutil.Process(p.pid)
    children = parent.children(recursive=True)
    for child in children:
        child.kill()
    gone, still_alive = psutil.wait_procs(children, timeout=5)


def _run_rlgames_test(script, task, sim_device, pipeline, time=180, dr=False, pretrained=False):
    os.chdir(os.path.join(REPO_PATH, "omniisaacgymenvs"))
    cmd = [
        PYTHON_EXE,
        f"scripts/{script}.py",
        f"num_envs=25",
        f"task={task}",
        f"sim_device={sim_device}",
        f"pipeline={pipeline}",
        "seed=42",
        "test=True",
    ]
    if dr:
        cmd.append(f"task.domain_randomization.randomize=True")

    experiment_name = f"{task}_{sim_device}_{pipeline}"
    if script == RLGAMES_MT_SCRIPT:
        experiment_name += "_mt"
        cmd.append("mt_timeout=900")
    if dr:
        experiment_name += "_dr"

    if pretrained:
        checkpoints_dict = {
            "AllegroHand": "allegro_hand",
            "Ant": "ant",
            "Anymal": "anymal",
            "AnymalTerrain": "anymal_terrain",
            "BallBalance": "ball_balance",
            "Cartpole": "cartpole",
            "Crazyflie": "crazyflie",
            "FrankaCabinet": "franka_cabinet",
            "Humanoid": "humanoid",
            "Ingenuity": "ingenuity",
            "Quadcopter": "quadcopter",
            "ShadowHand": "shadow_hand",
        }
        cmd.append(
            f"checkpoint=http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1/Isaac/Samples/OmniIsaacGymEnvs/Checkpoints/{checkpoints_dict[task]}.pth"
        )
    else:
        cmd.append(f"checkpoint=runs/{experiment_name}/nn/{experiment_name}.pth")

    process = subprocess.Popen(cmd, shell=False)
    try:
        process.wait(time)
    except subprocess.TimeoutExpired:
        _kill_process(process)


def _extract_feature(log_data, feature):
    return max(np.array(log_data[feature])[:, 1])


def _extract_reward(log_data):
    return _extract_feature(log_data, "rewards/iter")


def _extract_episode_length(log_data):
    return _extract_feature(log_data, "episode_lengths/iter")


def _extract_time(log_data):
    return log_data["rewards/time"][-1][0]


def _parse_tf_logs(log):
    from tensorboard.backend.event_processing import event_accumulator

    log_data = {}
    ea = event_accumulator.EventAccumulator(log)
    ea.Reload()
    tags = ea.Tags()["scalars"]
    for tag in tags:
        log_data[tag] = []
        for event in ea.Scalars(tag):
            log_data[tag].append((event.step, event.value))

    return log_data


def _retrieve_logs(experiment_name):
    log_dir = os.path.join(REPO_PATH, f"omniisaacgymenvs/runs/{experiment_name}/summaries")
    log_files = glob.glob(log_dir)
    latest_log = max(log_files, key=os.path.getctime)
    print("parsing log file", latest_log)
    log_data = _parse_tf_logs(latest_log)
    return log_data


def _setup_OIGE():
    omni.kit.pipapi.install(
        "gitpython", ignore_import_check=True
    )  # need to ignore import check if import name does not match pip package name
    omni.kit.pipapi.install("tensorboard")
    from git import Repo

    git_url = "https://gitlab-master.nvidia.com/carbon-gym/omniisaacgymenvs.git"

    # clone and install OIGE
    if not os.path.exists(REPO_PATH):
        Repo.clone_from(git_url, REPO_PATH, branch="dev")
    subprocess.check_call([PYTHON_EXE, "-m", "pip", "install", "-e", REPO_PATH])


# base class to use for Gym test cases
class OmniIsaacGymEnvsTestCase(omni.kit.test.AsyncTestCase):
    @classmethod
    def setUpClass(self):
        # set up OIGE repo
        _setup_OIGE()

    async def setUp(self):
        pass

    async def tearDown(self):
        pass
