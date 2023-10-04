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
from asyncio import sleep
from math import inf

import pxr
import omni.kit.test
import gc

# Transition between 104 and 105, deprecation of namespace omni.usd.utils
try:
    from omni.usd.utils import get_world_transform_matrix, get_local_transform_matrix
except:
    from omni.usd import get_world_transform_matrix, get_local_transform_matrix


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the World of module will make it auto-discoverable by omni.kit.test

from pxr import UsdGeom, Usd, UsdPhysics
from omni.isaac.onshape.widgets import documents_widget, elements_widget, assembly_widget, parts_widget
from omni.isaac.onshape.scripts import usd_generator

test_documents = {
    "slider_mate_no_limits": {
        "d": "1c63e5841a223bcbf036032b",
        "dtype": "w",
        "wdid": "8aaeabcc6493aa804fa78aa7",
        "e": "14c9dde4ec7335462ba5fc9c",
    },
    "slider_mate_limits": {
        "d": "1c63e5841a223bcbf036032b",
        "dtype": "w",
        "wdid": "8aaeabcc6493aa804fa78aa7",
        "e": "703edd0e5ac36655ab1c32b2",
    },
    "revolute_mate_no_limits_no_offset": {
        "d": "dea86f9df44a48d3f0566330",
        "dtype": "w",
        "wdid": "6665f1f2355104927ae3ec1d",
        "e": "d58fadb7f9bae5a8e61b4ad1",
    },
    "revolute_mate_no_limits_offset": {
        "d": "dea86f9df44a48d3f0566330",
        "dtype": "w",
        "wdid": "6665f1f2355104927ae3ec1d",
        "e": "b9eea3cbcacf6e0bc44aa713",
    },
    "revolute_mate_no_limits_no_offset_limit_enabled": {
        "d": "dea86f9df44a48d3f0566330",
        "dtype": "w",
        "wdid": "6665f1f2355104927ae3ec1d",
        "e": "90e7600aec97761e4585b461",
    },
    "revolute_mate_limits_no_offset": {
        "d": "dea86f9df44a48d3f0566330",
        "dtype": "w",
        "wdid": "6665f1f2355104927ae3ec1d",
        "e": "19b01a95ee2b758ba74f35a5",
    },
    "revolute_mate_low_limit_no_offset": {
        "d": "dea86f9df44a48d3f0566330",
        "dtype": "w",
        "wdid": "6665f1f2355104927ae3ec1d",
        "e": "8b6329dbdf888d59044e5461",
    },
    "revolute_mate_high_limit_no_offset": {
        "d": "dea86f9df44a48d3f0566330",
        "dtype": "w",
        "wdid": "6665f1f2355104927ae3ec1d",
        "e": "5a4e143260a74acb372f59c9",
    },
    "cylindric_mate": {},
    "spherical_mate": {},
    "group_mate": {},
    "sub_assemblies": {},
    "colors": {},
    "mass_properties": {},
}


class TestOnshape(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self.assembly_model = None
        self.usd_gen = None
        self.parts = None
        self.stage = None
        self.parts_model = None
        self.auth_callback = False
        pass

    # After running each test
    async def tearDown(self):
        if self.usd_gen:
            del self.assembly_model
            self.assembly_model = None
            del self.parts
            self.parts = None
            del self.parts_model
            self.parts_model = None
            self.auth_callback = False
            self.usd_gen.on_shutdown()
            self.usd_gen.delete_folder()
            del self.usd_gen
            self.usd_gen = None
            gc.collect()
            await omni.usd.get_context().new_stage_async()
        pass

    # Run for a single frame and exit
    async def test_01_load_client(self):
        # this should be installed automatically when the extension is loaded
        omni.isaac.onshape.onshape_client

        self.assertIsNotNone(omni.isaac.onshape.onshape_client)
        pass

    async def test_02_authenticate(self):
        from omni.isaac.onshape.client import OnshapeClient

        self.auth_callback = False

        def on_callback():
            self.auth_callback = True

        OnshapeClient.authenticate(on_callback)
        timeout = 100
        while not self.auth_callback and not OnshapeClient.is_authenticated():
            timeout -= 1
            if timeout == 0:
                raise TimeoutError()
            await sleep(1.0)
        self.assertTrue(OnshapeClient.is_authenticated())

    async def test_03_import(self, doc=None):
        await self.test_02_authenticate()
        if doc is None:
            doc = documents_widget.DocumentItem(
                test_documents["slider_mate_no_limits"]["d"],
                workspace_id=test_documents["slider_mate_no_limits"]["wdid"],
                element=test_documents["slider_mate_no_limits"]["e"],
            )
        # element = elements_widget.ElementItem(doc, 0)
        self.assembly_model = assembly_widget.OnshapeAssemblyModel(
            doc,
            doc.get_elements()[0],
            # assembly_loaded_fn=lambda a=weakref.proxy(self): a.assembly_reloaded(),
            rig_physics=True,
        )
        self.usd_gen = usd_generator.UsdGenerator(
            doc, self.assembly_model, UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())
        )
        self.assembly_model.get_assembly_definition_sync()
        self.parts = self.assembly_model.get_parts()
        self.parts_model = parts_widget.OnshapePartListModel(self.parts)
        self.usd_gen.create_all_stages(self.parts_model._children)
        # parts_model.import_meshes(import_all = True)
        timeout = 200
        last_count = self.parts_model.get_num_pending_meshes()
        while self.parts_model.get_num_pending_meshes() > 0 and not self.usd_gen.finished_meshes() and timeout > 0:
            timeout = timeout - 1
            # Resets timeout at every donwloaded piece
            if self.parts_model.get_num_pending_meshes() < last_count:
                timeout = 200
                last_count = self.parts_model.get_num_pending_meshes()
            self.usd_gen._on_update_ui(0)
            await sleep(0.5)

        self.assertEqual(self.parts_model.get_num_pending_meshes(), 0)

        for part in self.parts_model._children:
            self.usd_gen.create_part_stage(part)
        await omni.kit.app.get_app().next_update_async()
        self.assembly_model.assembly_features_sync()
        self.usd_gen.build_assemblies_sync()
        await omni.kit.app.get_app().next_update_async()
        self.assertIsInstance(self.usd_gen.assembly_stage, Usd.Stage)
        pass

    async def test_04_slider_mate_no_limits(self):
        await self.test_02_authenticate()
        d = test_documents["slider_mate_no_limits"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/no_limits/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[1] * self.usd_gen.stage_unit, -0.011339846067130566, 4)
        prim = stage.GetPrimAtPath("/World/no_limits/Part_1_01")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[1] * self.usd_gen.stage_unit, 0, 4)
        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/no_limits/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/no_limits/Slider_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.PrismaticJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -inf)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), inf)

    async def test_slider_mate_limits(self):
        await self.test_02_authenticate()
        d = test_documents["slider_mate_limits"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/limits/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[1] * self.usd_gen.stage_unit, -0.011339846067130566, 4)
        prim = stage.GetPrimAtPath("/World/limits/Part_1_01")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[1] * self.usd_gen.stage_unit, 0, 4)
        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/limits/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/limits/Slider_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.PrismaticJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), 0)
        self.assertAlmostEqual(joint.GetUpperLimitAttr().Get() * self.usd_gen.stage_unit, 0.1, 4)

    async def test_revolute_mate_no_limits_no_offset(self):
        await self.test_02_authenticate()
        d = test_documents["revolute_mate_no_limits_no_offset"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/no_limits_no_offset/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[2] * self.usd_gen.stage_unit, 0.025, 4)

        # Check pose

        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/no_limits_no_offset/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/no_limits_no_offset/Revolute_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -inf)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), inf)

    async def test_revolute_mate_no_limits_offset(self):
        await self.test_02_authenticate()
        d = test_documents["revolute_mate_no_limits_offset"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/no_limits_offset/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[2] * self.usd_gen.stage_unit, 0.025, 4)
        self.assertAlmostEqual(position[1] * self.usd_gen.stage_unit, 0.10, 4)

        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/no_limits_offset/Revolute_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -inf)
        self.assertAlmostEqual(joint.GetUpperLimitAttr().Get(), inf)

    async def test_revolute_mate_no_limits_no_offset_limits_enabled(self):
        await self.test_02_authenticate()
        d = test_documents["revolute_mate_no_limits_no_offset_limit_enabled"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/no_limits_no_offset_limits_enabled/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[2] * self.usd_gen.stage_unit, 0.025, 4)

        # Check pose

        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/no_limits_no_offset_limits_enabled/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/no_limits_no_offset_limits_enabled/Revolute_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -inf)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), inf)

    async def test_revolute_mate_limits_no_offset(self):
        await self.test_02_authenticate()
        d = test_documents["revolute_mate_limits_no_offset"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/limits_no_offset/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[2] * self.usd_gen.stage_unit, 0.025, 4)

        # Check pose

        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/limits_no_offset/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/limits_no_offset/Revolute_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), 0)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), 180)

    async def test_revolute_mate_low_limit_no_offset(self):
        await self.test_02_authenticate()
        d = test_documents["revolute_mate_low_limit_no_offset"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/low_limit_no_offset/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[2] * self.usd_gen.stage_unit, 0.025, 4)

        # Check pose

        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/low_limit_no_offset/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/low_limit_no_offset/Revolute_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), 0)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), inf)

    async def test_revolute_mate_high_limit_no_offset(self):
        await self.test_02_authenticate()
        d = test_documents["revolute_mate_high_limit_no_offset"]
        doc = documents_widget.DocumentItem(d["d"], workspace_id=d["wdid"], element=d["e"])
        await self.test_03_import(doc)
        stage = self.usd_gen.assembly_stage
        # Check prim was made
        prim = stage.GetPrimAtPath("/World/high_limit_no_offset/Part_1")
        self.assertTrue(prim.IsValid())

        # Check pose

        pose = get_world_transform_matrix(prim)
        position = pose.ExtractTranslation()
        self.assertAlmostEqual(position[2] * self.usd_gen.stage_unit, 0.025, 4)

        # Check pose

        # Check it has the reference
        prim = stage.GetPrimAtPath("/World/high_limit_no_offset/Part_1/Part_1")
        self.assertTrue(prim.IsValid())
        # Check the slider made was build
        prim = stage.GetPrimAtPath("/World/high_limit_no_offset/Revolute_1")
        self.assertTrue(prim.IsValid())
        joint = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -inf)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), 180)
