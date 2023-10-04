# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

import omni.kit.commands
from omni.isaac.conveyor.scripts.commands import CreateConveyorBelt
import carb.tokens
import asyncio
import numpy as np
import time
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema

from usdrt import Usd, Sdf


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
async def simulate_async(seconds: float, steps_per_sec: int = 60) -> None:
    """Helper function to simulate async for seconds * steps_per_sec frames.

    Args:
        seconds (float): time in seconds to simulate for
        steps_per_sec (int, optional): steps per second. Defaults to 60.
        callback (Callable, optional): optional function to run every step. Defaults to None.
    """
    for frame in range(int(steps_per_sec * seconds)):
        await omni.kit.app.get_app().next_update_async()


def add_cube(stage, path, size, offset, physics=False):
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)

    cubeGeom.CreateSizeAttr(size)
    cubeGeom.AddTranslateOp().Set(offset)
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)

    UsdPhysics.CollisionAPI.Apply(cubePrim)
    return cubePrim


def create_physics_scene(stage, gravity=9.81):
    scene = UsdPhysics.Scene.Define(stage, "/physics")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(gravity)

    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physics"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physics")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")


class TestConveyor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))

        self.conveyor_node = None
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        create_physics_scene(self._stage)
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        self.conveyor_node = None
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_add_conveyor(self, physics=True):
        stage = omni.usd.get_context().get_stage()
        cube_prim = add_cube(self._stage, "/cube", 1.00, (0, 0, 0), physics=physics)
        _, og_prim = omni.kit.commands.execute("CreateConveyorBelt", conveyor_prim=cube_prim)
        self.assertIsNotNone(og_prim)
        self.conveyor_node = og_prim
        self.assertTrue(self.conveyor_node.IsValid())
        pass

    async def test_add_conveyor_without_physics(self):
        await self.test_add_conveyor(physics=False)

    async def test_set_velocity(self, direction=[1.0, 0.0, 0.0]):
        await self.test_add_conveyor()
        dir_attr = self.conveyor_node.GetAttribute("inputs:direction")
        dir_attr.Set(Gf.Vec3f(*direction))
        attr = self.conveyor_node.GetAttribute("inputs:velocity")
        attr.Set(0.10)
        self.assertAlmostEqual(attr.Get(), 0.10, delta=1e-4)
        self._timeline.play()
        await simulate_async(0.4)
        rigid_prim = UsdPhysics.RigidBodyAPI(self._stage.GetPrimAtPath("/cube"))
        usd_velocity = rigid_prim.GetVelocityAttr().Get()
        self.assertAlmostEqual(usd_velocity.GetLength(), 0.10, delta=1e-4)
        self._timeline.stop()
        pass

    async def test_set_angular_velocity(self, direction=[0.0, 0.0, 1.0]):
        await self.test_add_conveyor()
        dir_attr = self.conveyor_node.GetAttribute("inputs:curved")
        dir_attr.Set(True)
        dir_attr = self.conveyor_node.GetAttribute("inputs:direction")
        dir_attr.Set(Gf.Vec3f(*direction))
        attr = self.conveyor_node.GetAttribute("inputs:velocity")
        attr.Set(0.10)
        self.assertAlmostEqual(attr.Get(), 0.10, delta=1e-4)
        self._timeline.play()
        await simulate_async(0.4)
        rigid_prim = UsdPhysics.RigidBodyAPI(self._stage.GetPrimAtPath("/cube"))
        usd_velocity = rigid_prim.GetAngularVelocityAttr().Get()
        self.assertAlmostEqual(usd_velocity.GetLength(), 0.10, delta=1e-4)
        self._timeline.stop()
        pass

    async def test_conveyor(self, d=[1.0, 0.0, 0.0]):
        await self.test_set_velocity(d)

        cube_prim = add_cube(self._stage, "/cube2", 0.1, (0, 0, 0.55), physics=True)
        self._timeline.play()
        await simulate_async(0.4)
        rigid_prim = UsdPhysics.RigidBodyAPI(cube_prim)
        rt_stage = Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        rt_prim = rt_stage.GetPrimAtPath(Sdf.Path(str(cube_prim.GetPath())))
        # usd_velocity = rigid_prim.GetVelocityAttr().Get()
        usd_velocity = rt_prim.GetAttribute(rigid_prim.GetVelocityAttr().GetName()).Get()
        print(usd_velocity)
        self.assertTrue(
            np.allclose([a * 0.10 for a in d], usd_velocity, atol=5e-2),
            "{} != {}".format([a * 0.10 for a in d], usd_velocity),
        )
        pass

    async def test_conveyor_y(self):
        await self.test_conveyor(d=[0.0, 1.0, 0.0])

    async def test_100_conveyors(self):
        # self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", False)
        # carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        # carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))

        conveyor_nodes = []
        for i in range(10):
            for j in range(10):
                cube_prim = add_cube(self._stage, f"/cube_{i}_{j}", 1.00, (i, j, 0), physics=True)
                _, og_prim = omni.kit.commands.execute("CreateConveyorBelt", conveyor_prim=cube_prim)
                self.assertIsNotNone(og_prim)
                conveyor_nodes.append(og_prim)
                self.assertTrue(conveyor_nodes[-1].IsValid())
                dir_attr = conveyor_nodes[-1].GetAttribute("inputs:direction")
                dir_attr.Set(Gf.Vec3f(*[float(i <= j), float(i > j), 0.0]))
                attr = conveyor_nodes[-1].GetAttribute("inputs:velocity")
                attr.Set(1)
        self._timeline.play()
        await simulate_async(2.0)
        t = time.time()
        # SImulate exacly 100 frames
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
        rtt = time.time() - t
        self.assertLessEqual(rtt, 1.0)  # Must run 100 frames in less than one second
