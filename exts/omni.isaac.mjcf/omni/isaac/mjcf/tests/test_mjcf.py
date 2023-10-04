# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
import os
from pxr import Sdf, Gf, UsdShade, PhysicsSchemaTools, UsdGeom, UsdPhysics
import pxr
import asyncio
import numpy as np


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestMJCF(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.mjcf")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_mjcf_ant(self):
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_fix_base(True)
        import_config.set_import_inertia_tensor(True)
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._extension_path + "/data/mjcf/nv_ant.xml",
            import_config=import_config,
            prim_path="/ant",
        )
        await omni.kit.app.get_app().next_update_async()

        # check if object is there
        prim = stage.GetPrimAtPath("/ant")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints and links exist
        root_joint = stage.GetPrimAtPath("/ant/joints/rootJoint")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        front_left_leg_joint = stage.GetPrimAtPath("/ant/joints/hip_1")
        self.assertNotEqual(front_left_leg_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(front_left_leg_joint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertAlmostEqual(front_left_leg_joint.GetAttribute("physics:upperLimit").Get(), 40)
        self.assertAlmostEqual(front_left_leg_joint.GetAttribute("physics:lowerLimit").Get(), -40)

        front_left_leg = stage.GetPrimAtPath("/ant/front_left_leg")
        self.assertAlmostEqual(front_left_leg.GetAttribute("physics:diagonalInertia").Get()[0], 0.0)
        self.assertAlmostEqual(front_left_leg.GetAttribute("physics:mass").Get(), 0.0)

        front_left_foot_joint = stage.GetPrimAtPath("/ant/joints/ankle_1")
        self.assertNotEqual(front_left_foot_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(front_left_foot_joint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertAlmostEqual(front_left_foot_joint.GetAttribute("physics:upperLimit").Get(), 100)
        self.assertAlmostEqual(front_left_foot_joint.GetAttribute("physics:lowerLimit").Get(), 30)

        front_left_foot = stage.GetPrimAtPath("/ant/front_left_foot")
        self.assertAlmostEqual(front_left_foot.GetAttribute("physics:diagonalInertia").Get()[0], 0.0)
        self.assertAlmostEqual(front_left_foot.GetAttribute("physics:mass").Get(), 0.0)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)

    async def test_mjcf_humanoid(self):
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_fix_base(True)
        import_config.set_import_inertia_tensor(True)
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._extension_path + "/data/mjcf/nv_humanoid.xml",
            import_config=import_config,
            prim_path="/humanoid",
        )
        await omni.kit.app.get_app().next_update_async()

        # check if object is there
        prim = stage.GetPrimAtPath("/humanoid")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints and link exist
        root_joint = stage.GetPrimAtPath("/humanoid/joints/rootJoint")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        pelvis_joint = stage.GetPrimAtPath("/humanoid/joints/abdomen_x")
        self.assertNotEqual(pelvis_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(pelvis_joint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertAlmostEqual(pelvis_joint.GetAttribute("physics:upperLimit").Get(), 35)
        self.assertAlmostEqual(pelvis_joint.GetAttribute("physics:lowerLimit").Get(), -35)

        lower_waist_joint = stage.GetPrimAtPath("/humanoid/joints/lower_waist")
        self.assertNotEqual(lower_waist_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(lower_waist_joint.GetTypeName(), "PhysicsJoint")
        self.assertAlmostEqual(lower_waist_joint.GetAttribute("limit:rotX:physics:high").Get(), 45)
        self.assertAlmostEqual(lower_waist_joint.GetAttribute("limit:rotX:physics:low").Get(), -45)
        self.assertAlmostEqual(lower_waist_joint.GetAttribute("limit:rotY:physics:high").Get(), 30)
        self.assertAlmostEqual(lower_waist_joint.GetAttribute("limit:rotY:physics:low").Get(), -75)
        self.assertAlmostEqual(lower_waist_joint.GetAttribute("limit:rotZ:physics:high").Get(), -1)
        self.assertAlmostEqual(lower_waist_joint.GetAttribute("limit:rotZ:physics:low").Get(), 1)

        left_foot = stage.GetPrimAtPath("/humanoid/left_foot")
        self.assertAlmostEqual(left_foot.GetAttribute("physics:diagonalInertia").Get()[0], 0.0)
        self.assertAlmostEqual(left_foot.GetAttribute("physics:mass").Get(), 0.0)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)

    # This sample corresponds to the example in the docs, keep this and the version in the docs in sync
    async def test_doc_sample(self):
        import omni.kit.commands
        from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools

        # setting up import configuration:
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_fix_base(True)
        import_config.set_import_inertia_tensor(True)

        # Get path to extension data:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.mjcf")
        extension_path = ext_manager.get_extension_path(ext_id)

        # import MJCF
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=extension_path + "/data/mjcf/nv_ant.xml",
            import_config=import_config,
            prim_path="/ant",
        )

        # get stage handle
        stage = omni.usd.get_context().get_stage()

        # enable physics
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        # set gravity
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        # add lighting
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)

    async def test_mjcf_scale(self):
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_distance_scale(1.0)
        import_config.set_fix_base(True)
        import_config.set_import_inertia_tensor(True)
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._extension_path + "/data/mjcf/nv_ant.xml",
            import_config=import_config,
            prim_path="/ant",
        )
        await omni.kit.app.get_app().next_update_async()

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        pass

    async def test_mjcf_self_collision(self):
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_self_collision(True)
        import_config.set_fix_base(True)
        import_config.set_import_inertia_tensor(True)
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._extension_path + "/data/mjcf/nv_ant.xml",
            import_config=import_config,
            prim_path="/ant",
        )
        await omni.kit.app.get_app().next_update_async()

        prim = stage.GetPrimAtPath("/ant")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(prim.GetAttribute("physxArticulation:enabledSelfCollisions").Get(), True)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        pass

    async def test_mjcf_default_prim(self):
        stage = omni.usd.get_context().get_stage()
        mjcf_path = os.path.abspath(self._extension_path + "/data/mjcf/nv_ant.xml")
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_fix_base(True)
        import_config.set_import_inertia_tensor(True)
        import_config.set_make_default_prim(True)
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._extension_path + "/data/mjcf/nv_ant.xml",
            import_config=import_config,
            prim_path="/ant_1",
        )
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._extension_path + "/data/mjcf/nv_ant.xml",
            import_config=import_config,
            prim_path="/ant_2",
        )
        await omni.kit.app.get_app().next_update_async()

        default_prim = stage.GetDefaultPrim()
        self.assertNotEqual(default_prim.GetPath(), Sdf.Path.emptyPath)
        prim_2 = stage.GetPrimAtPath("/ant_2")
        self.assertNotEqual(prim_2.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(default_prim.GetPath(), prim_2.GetPath())
        pass
