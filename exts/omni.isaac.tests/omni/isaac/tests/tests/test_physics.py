# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from pxr import UsdGeom, Sdf, UsdPhysics, Gf
import carb
import numpy as np
import omni.physx
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage_async
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.dynamic_control import _dynamic_control


class TestPhysics(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        carb.settings.get_settings().set("persistent/app/stage/upAxis", "Z")
        # force editor and physics to have the same rate (should be 60)
        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        pass

    # simple fastcache smoke test, disabling because fastcache is not used anymore
    # async def test_fast_cache(self):
    #     carb.settings.get_settings().set_int("persistent/physics/useFastCache", True)

    #     cubePath = "/World/Cube"
    #     cubeGeom = UsdGeom.Cube.Define(self._stage, cubePath)
    #     cubeGeom.CreateSizeAttr(100)
    #     cubePrim = self._stage.GetPrimAtPath(cubePath)
    #     # await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    #     rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)

    #     omni.timeline.get_timeline_interface().play()
    #     await omni.kit.app.get_app().next_update_async()
    #     omni.timeline.get_timeline_interface().stop()
    #     pass

    # Disabling test, because fastcache is not used anymore
    # async def test_fast_cache_no_usd_updates(self):

    #     carb.settings.get_settings().set_int("persistent/physics/useFastCache", True)
    #     carb.settings.get_settings().set_int("persistent/physics/updateToUsd", False)

    #     cubePath = "/World/Cube"
    #     cubeGeom = UsdGeom.Cube.Define(self._stage, cubePath)
    #     cubeGeom.CreateSizeAttr(100)
    #     cubePrim = self._stage.GetPrimAtPath(cubePath)
    #     # await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    #     UsdPhysics.RigidBodyAPI.Apply(cubePrim)

    #     omni.timeline.get_timeline_interface().play()
    #     for frame in range(60):
    #         await omni.kit.app.get_app().next_update_async()
    #     physx_interface = omni.physx.acquire_physx_interface()
    #     position = physx_interface.get_rigidbody_transformation("/World/Cube")["position"]
    #     self.assertNotAlmostEqual(position[2], 0, 0)
    #     usd_position = omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation()
    #     self.assertAlmostEqual(usd_position[2], 0, 0)
    #     # carb.settings.get_settings().set_int("persistent/physics/updateToUsd", True)
    #     # await omni.kit.app.get_app().next_update_async()
    #     # calling this forces the pose to update
    #     physx_interface.update_transformations(True, True, True, False)
    #     await omni.kit.app.get_app().next_update_async()
    #     usd_position = omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation()
    #     self.assertNotAlmostEqual(usd_position[2], 0, 0)
    #     omni.timeline.get_timeline_interface().pause()
    #     pass

    async def test_fast_cache_with_usd_updates(self):
        carb.settings.get_settings().set_int("persistent/physics/useFastCache", True)
        carb.settings.get_settings().set_int("persistent/physics/updateToUsd", True)

        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(self._stage, cubePath)
        cubeGeom.CreateSizeAttr(100)
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        # await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        omni.timeline.get_timeline_interface().play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell due to gravity
        position = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
        self.assertNotAlmostEqual(position[2], 0, 0)
        omni.timeline.get_timeline_interface().stop()
        pass

    async def test_rigid_body(self):

        carb.settings.get_settings().set_int("persistent/physics/useFastCache", False)
        carb.settings.get_settings().set_int("persistent/physics/updateToUsd", True)

        dt = 1.0 / self._physics_rate

        # def physics_update(dt):
        #     print("physics update step:", dt, "seconds")

        # physics_sub = omni.physx.acquire_physx_interface().subscribe_physics_step_events(physics_update)

        # add scene
        self._scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        self._scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Add a cube
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(self._stage, cubePath)
        cubeGeom.CreateSizeAttr(100)
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        # await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        await omni.kit.app.get_app().next_update_async()

        # test acceleration, velocity, position
        omni.timeline.get_timeline_interface().play()
        # warm up simulation
        await omni.kit.app.get_app().next_update_async()
        # get initial position
        a = -9.81

        # simulate for one second
        time_elapsed = dt
        for frame in range(30):
            p_0 = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
            v_0 = np.array(rigidBodyAPI.GetVelocityAttr().Get())
            await omni.kit.app.get_app().next_update_async()

            p_1 = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
            v_1 = np.array(rigidBodyAPI.GetVelocityAttr().Get())
            # print("time elapsed", time_elapsed)
            v_expected = v_0[2] + a * dt
            self.assertAlmostEqual(v_1[2], v_expected, 0)
            # print(v_1[2], v_expected)
            p_expected = p_0[2] + v_expected * dt
            # print(p_1[2], p_expected)
            self.assertAlmostEqual(p_1[2], p_expected, 0)
            time_elapsed += dt
        omni.timeline.get_timeline_interface().stop()
        pass

    async def test_reparenting(self):
        timeline = omni.timeline.get_timeline_interface()
        omni.kit.commands.execute("CreatePrim", prim_type="Xform")
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute("MovePrim", path_from="/Xform", path_to="/AnotherPath")
        timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()

    # test is a known failure on 102, will be fixed on 103
    # async def test_subscription(self):
    #     await omni.usd.get_context().new_stage_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     timeline = omni.timeline.get_timeline_interface()
    #     self.check_dt = 0.0  # set this to zero to start

    #     def on_update(dt):
    #         print("on_update called")
    #         self.check_dt = dt

    #     sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_update)
    #     timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     self.assertNotEqual(self.check_dt, 0.0)
    #     timeline.stop()
    #     await omni.kit.app.get_app().next_update_async()
    #     self.check_dt = 0.0  # reset this to zero to see if it changes after a stop/play
    #     timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     self.assertNotEqual(self.check_dt, 0.0)
    #     sub = None

    async def test_stage_up_axis(self):
        timeline = omni.timeline.get_timeline_interface()
        # Make a new stage Z up
        carb.settings.get_settings().set("persistent/app/stage/upAxis", "Z")
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        # Add a cube for testing gravity
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(stage, cubePath)
        cubeGeom.CreateSizeAttr(1.00)
        cubePrim = stage.GetPrimAtPath(cubePath)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell -Z
        position = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
        self.assertAlmostEqual(position[2], -4.9867, delta=0.01)
        timeline.stop()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell -Y
        position = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
        self.assertAlmostEqual(position[1], -4.9867, delta=0.01)

    async def test_stage_units(self):
        timeline = omni.timeline.get_timeline_interface()
        # Make a new stage Z up
        carb.settings.get_settings().set("persistent/app/stage/upAxis", "Z")
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        # Add a cube for testing gravity
        cubePath = "/World/Cube"
        cubeGeom = UsdGeom.Cube.Define(stage, cubePath)
        cubeGeom.CreateSizeAttr(1.00)
        cubePrim = stage.GetPrimAtPath(cubePath)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        # check to make sure that the cube fell -Z
        position = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
        self.assertAlmostEqual(position[2], -4.9867, delta=0.01)
        timeline.stop()
        # switch to meters
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        await omni.kit.app.get_app().next_update_async()

        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        position = np.array(omni.usd.utils.get_world_transform_matrix(cubePrim).ExtractTranslation())
        self.assertAlmostEqual(position[2], -4.9867, delta=0.01)

    async def test_articulation_reference(self):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        stage = omni.usd.get_context().get_stage()
        timeline = omni.timeline.get_timeline_interface()

        add_reference_to_stage(asset_path, "/franka")

        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()

        hand_prim = stage.GetPrimAtPath("/franka/panda_hand")

        await omni.kit.app.get_app().next_update_async()
        trans_a = np.array(omni.usd.utils.get_world_transform_matrix(hand_prim).ExtractTranslation())

        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        timeline.play()
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()

        trans_b = np.array(omni.usd.utils.get_world_transform_matrix(hand_prim).ExtractTranslation())

        self.assertAlmostEqual(np.linalg.norm(trans_a - trans_b), 0, delta=0.03)

    async def test_articulation_drive(self):
        timeline = omni.timeline.get_timeline_interface()
        extension_path = get_extension_path_from_name("omni.isaac.tests")
        usd_path = extension_path + "/data/tests/articulation_drives_opposite.usd"
        (result, error) = await open_stage_async(usd_path)
        # Make sure the stage loaded
        self.assertTrue(result)

        # get handle to each robot's chassis
        stage = omni.usd.get_context().get_stage()
        robot_joints = stage.GetPrimAtPath("/World/mock_robot/body")
        robot_articulation = stage.GetPrimAtPath("/World/mock_robot_with_articulation_root/body")

        # simulate for 60 frames
        timeline.play()
        for frame in range(120):
            await omni.kit.app.get_app().next_update_async()

        # compare position in the x direction
        xpos_1 = np.array(omni.usd.utils.get_world_transform_matrix(robot_joints).ExtractTranslation())[0]
        xpos_2 = np.array(omni.usd.utils.get_world_transform_matrix(robot_articulation).ExtractTranslation())[0]
        pos_diff = np.linalg.norm(xpos_1 - xpos_2)
        self.assertAlmostEqual(pos_diff, 0, delta=1)
        self.assertGreater(xpos_1, 2)
        self.assertGreater(xpos_2, 2)

    async def test_delete(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        prim_a = self._stage.DefinePrim("/World/Franka_1", "Xform")
        prim_a.GetReferences().AddReference(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")
        prim_b = self._stage.DefinePrim("/World/Franka_2", "Xform")
        prim_b.GetReferences().AddReference(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        with Sdf.ChangeBlock():
            omni.usd.commands.DeletePrimsCommand(["/World/Franka_1"]).do()
            omni.usd.commands.DeletePrimsCommand(["/World/Franka_2"]).do()
        await omni.kit.app.get_app().next_update_async()
