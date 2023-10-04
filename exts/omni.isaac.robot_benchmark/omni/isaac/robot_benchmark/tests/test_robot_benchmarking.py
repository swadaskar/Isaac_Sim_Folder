# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.usd
import omni.kit.test
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.isaac.motion_generation as motion_generation
from omni.isaac.robot_benchmark.robot_benchmarking import RobotBenchmark
from omni.isaac.robot_benchmark.benchmark_robots import BenchmarkRobotRegistry
from omni.isaac.robot_benchmark.benchmark_logger import BenchmarkLogger
from omni.isaac.benchmark_environments.environments import EnvironmentCreator
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.world import World

import numpy as np
import os
import json

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestRobotBenchmark(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_rate = 60  # fps
        omni.usd.get_context().new_stage()

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.set_extension_enabled_immediate("omni.isaac.robot_benchmark", True)
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.robot_benchmark")
        self.benchmark_extension_path = ext_manager.get_extension_path(ext_id)

        self._log_file_path = os.path.join(self.benchmark_extension_path, "omni", "isaac", "robot_benchmark", "tests")

        self.benchmark_robot_registry = BenchmarkRobotRegistry()

        """
        Use self._write_new_golden_vals to cause any test that logs data to overwrite its reference file
        of golden values.  The golden values that are checked are the configuration-space positions of the 
        robot during the simulation.  
        This should be done when a change is made in robot_benchmark, benchmark_environment,
        or motion_generation that causes the robot to follow a different trajectory.  The new trajectories being
        recorded as golden values should be visually checked for correctness before being recorded.
        """
        self._write_new_golden_vals = False

        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        self._robot_benchmark = None
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        World.clear_instance()
        pass

    async def _set_determinism_settings(self, robot):
        World()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        carb.settings.get_settings().set_bool("/app/file/ignoreUnsavedOnExit", True)

        robot.disable_gravity()
        robot.set_solver_position_iteration_count(64)
        robot.set_solver_velocity_iteration_count(64)

    """
    All tests of the form test_{environment}_{robot}_{policy}() check that the robot followed exactly the same
    trajectory as it has in the past.  The robot's c-space configuration in every frame is checked against a
    log that has been written to a local file.  When robot behavior changes in a way that is intentional and correct,
    new golden values for the test may be recorded by changing self.write_new_golden_values to True inside setUp().
    
    The correctness of robot behavior can be ascertained by running the test through the test_runner in the Sim GUI.
    The robot should be effectively avoiding obstacles after reaching the start position (the blue target), and it should
    be clearly attempting to reach the red targets.

    An error is thrown after writing new golden values to keep the user from accidentally leaving the 
    self.write_new_golden_values flag as True.
    """

    async def test_cubby_franka_rmpflow(self):
        file_name = os.path.join(self._log_file_path, "recorded_tests/Cubby_Franka_RMP_Test.json")
        benchmark_logger = BenchmarkLogger(file_name)
        await self._run_benchmark("Cubby", "Franka", "RMPflow", 1000, benchmark_logger)

        if self._write_new_golden_vals:
            benchmark_logger.write_to_json(skip_headerless_tests=False)
            self.assertTrue(
                False,
                "Wrote new Golden Values to Log File. Change WriteGoldenValues back to False for the test to pass",
            )
        else:
            await self._assert_benchmark_log_matches_golden_values(file_name, benchmark_logger)

    async def test_cubby_ur10_rmpflow(self):
        file_name = os.path.join(self._log_file_path, "./recorded_tests/Cubby_UR10_RMP_Test.json")
        benchmark_logger = BenchmarkLogger(file_name)
        await self._run_benchmark("Cubby", "UR10", "RMPflow", 1000, benchmark_logger)

        if self._write_new_golden_vals:
            benchmark_logger.write_to_json(skip_headerless_tests=False)
            self.assertTrue(
                False,
                "Wrote new Golden Values to Log File. Change WriteGoldenValues back to False for the test to pass",
            )
        else:
            await self._assert_benchmark_log_matches_golden_values(file_name, benchmark_logger)

    async def test_window_ur10_rmpflow(self):
        file_name = os.path.join(self._log_file_path, "./recorded_tests/Window_UR10_RMP_Test.json")
        benchmark_logger = BenchmarkLogger(file_name)
        await self._run_benchmark("Window", "UR10", "RMPflow", 600, benchmark_logger)

        if self._write_new_golden_vals:
            benchmark_logger.write_to_json(skip_headerless_tests=False)
            self.assertTrue(
                False,
                "Wrote new Golden Values to Log File. Change WriteGoldenValues back to False for the test to pass",
            )
        else:
            await self._assert_benchmark_log_matches_golden_values(file_name, benchmark_logger)

    async def test_evasion_franka_rmpflow(self):
        file_name = os.path.join(self._log_file_path, "./recorded_tests/Evasion_Franka_RMP_Test.json")
        benchmark_logger = BenchmarkLogger(file_name)
        await self._run_benchmark("Evasion", "Franka", "RMPflow", 600, benchmark_logger)

        if self._write_new_golden_vals:
            benchmark_logger.write_to_json(skip_headerless_tests=False)
            self.assertTrue(
                False,
                "Wrote new Golden Values to Log File. Change WriteGoldenValues back to False for the test to pass",
            )
        else:
            await self._assert_benchmark_log_matches_golden_values(file_name, benchmark_logger)

    async def _get_test_assets(self, benchmark_config_util, env_name, robot_name, policy_name):
        robot_assets = benchmark_config_util.get_robot_assets(robot_name)

        env_kwargs = benchmark_config_util.get_environment_params(env_name, robot_name)

        default_policy_config = motion_generation.interface_config_loader.load_supported_motion_policy_config(
            robot_name, policy_name
        )
        final_policy_config = benchmark_config_util.overwrite_default_policy_config(
            env_name, robot_name, policy_name, default_policy_config
        )

        if policy_name == "RMPflow":
            motion_policy = motion_generation.lula.motion_policies.RmpFlow(**final_policy_config)
        else:
            carb.log_error("Unsupported MotionPolicy used in RobotBenchmark")

        return robot_assets, env_kwargs, motion_policy

    async def _assert_benchmark_log_matches_golden_values(self, file_name, benchmark_logger):
        # There is a very high tolerance of 4e-1 on tests matching the log due to physx indeterminacy

        with open(file_name) as golden_value_file:
            golden_values = json.load(golden_value_file)

        logged_values = benchmark_logger.to_json_dict(skip_headerless_tests=False)

        self.assertTrue(len(logged_values) == len(golden_values), "Golden Values do not match Logged Values in length")
        for i in range(len(logged_values)):
            log_body = logged_values[i]["body"]
            truth_body = golden_values[i]["body"]
            for b1, b2 in zip(log_body, truth_body):
                dbg_str = (
                    "(Logged, Golden) = " + str(b1["robot_cspace_config"]) + "," + str(b2["robot_cspace_config"]) + "\n"
                )
                dbg_str += "Index of mismatch: " + str(i)
                self.assertTrue(np.allclose(b1["robot_cspace_config"], b2["robot_cspace_config"], atol=4e-1), dbg_str)

    async def get_environment_params(self, env_name, robot_name):
        # Loads robot-specific parameters (if any) for an environment that are stored in the ../benchmark_config directory
        # For example, the Cubby environment is rotated for the UR10

        benchmark_config_dir = os.path.join(self.benchmark_extension_path, "benchmark_config")
        with open(os.path.join(benchmark_config_dir, "benchmark_config_map.json")) as benchmark_config_map:
            benchmark_config_map = json.load(benchmark_config_map)

        local_env_config_path = (
            benchmark_config_map.get(robot_name, {}).get("environment_config_paths", {}).get(env_name, None)
        )
        if local_env_config_path is None:
            return {}

        env_config_path = os.path.join(benchmark_config_dir, local_env_config_path)

        if os.path.exists(env_config_path):
            with open(env_config_path) as env_file:
                config = json.load(env_file)
        else:
            carb.log_error(
                "Invalid path to config file specified in benchmark_config_map.json for the "
                + robot_name
                + " in the "
                + env_name
                + "environment"
            )
            config = {}

        return config

    async def _run_benchmark(self, env_name, robot_name, policy_name, num_frames, benchmark_logger):

        env_creator = EnvironmentCreator()

        env_kwargs = await self.get_environment_params(env_name, robot_name)
        env = env_creator.create_environment(env_name, **env_kwargs)

        robot_loader = self.benchmark_robot_registry.get_robot_loader(robot_name)

        robot_benchmark = RobotBenchmark()

        set_camera_view(eye=env.camera_position, target=env.camera_target, camera_prim_path="/OmniverseKit_Persp")

        robot_benchmark.initialize_test(env, robot_loader, policy_name, benchmark_logger=benchmark_logger)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

        for frame in range(num_frames):
            await omni.kit.app.get_app().next_update_async()
            robot_benchmark.step(1 / 60.0)
            if frame == 0:
                robot_benchmark.toggle_testing()
                await self._set_determinism_settings(robot_benchmark._robot)

        self._timeline.stop()
