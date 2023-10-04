# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.kit.test
import numpy as np
import carb

from omni.isaac.core.utils.stage import update_stage_async, open_stage_async
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core import World, SimulationContext
import asyncio


class TestArticulationDeterminism(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        World.clear_instance()
        self._timeline = omni.timeline.get_timeline_interface()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(60))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(60))

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        pass

    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()
        pass

    async def test_inconsistent_result(self):
        frames_to_converge = np.empty(5)
        for i in range(5):
            num_frames = await self._test_franka_slow_convergence()
            frames_to_converge[i] = num_frames

        # Takes the same number of frames to converge every time
        print(f"Over 5 trials, the Franka converged to target in {frames_to_converge} frames.")
        self.assertTrue(
            np.unique(frames_to_converge).shape[0] == 1,
            f"Non-deterministic test converged in varying number of frames: {frames_to_converge}",
        )

        # On the develop branch, this test always takes 31 frames to converge
        self.assertTrue(frames_to_converge[0] == 31, "Took too many frames to converge!")

    async def _test_franka_slow_convergence(self):
        (result, error) = await open_stage_async(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")
        robot_prim_path = "/panda"
        World()  # Create a new default world to reset any physics settings.
        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        self._robot.get_articulation_controller().set_gains(1e4 * np.ones(9), 1e3 * np.ones(9))
        self._robot.set_solver_position_iteration_count(64)
        self._robot.set_solver_velocity_iteration_count(64)
        self._robot.post_reset()
        await update_stage_async()

        timeout = 200

        action = ArticulationAction(
            joint_positions=np.array(
                [
                    -0.40236897393760085,
                    -0.44815597748391767,
                    -0.16028112816211953,
                    -2.4554393933564986,
                    -0.34608791253975374,
                    2.9291361940824485,
                    0.4814803907662416,
                    None,
                    None,
                ]
            )
        )

        self._robot.get_articulation_controller().apply_action(action)

        for i in range(timeout):
            await omni.kit.app.get_app().next_update_async()
            diff = self._robot.get_joint_positions() - action.joint_positions
            if np.linalg.norm(diff) < 0.01:
                return i

        return timeout
