# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import os
import carb
import omni.kit.app


class TestExternalDependencies(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self.ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = self.ext_manager.get_enabled_extension_id("omni.isaac.tests")
        self._extension_path = self.ext_manager.get_extension_path(ext_id)

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_asset_converter(self):
        import omni.kit.asset_converter

        def progress_callback(progress, total_steps):
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        # setup converter and flags
        # converter_context.ignore_materials = False
        # converter_context.ignore_animation = False
        # converter_context.ignore_cameras = True
        # converter_context.single_mesh = True
        # converter_context.smooth_normals = True
        # converter_context.preview_surface = False
        # converter_context.support_point_instancer = False
        # converter_context.embed_mdl_in_usd = False
        # converter_context.use_meter_as_world_unit = True
        # converter_context.create_world_as_default_root_prim = False
        instance = omni.kit.asset_converter.get_instance()

        input_obj = os.path.abspath(self._extension_path + "/data/tests/test_mtl/test_mtl.obj")
        output_usd = os.path.abspath(self._extension_path + "/data/tests/test_mtl/test_mtl.usd")
        task = instance.create_converter_task(input_obj, output_usd, progress_callback, converter_context)
        success = await task.wait_until_finished()
        if not success:
            carb.log_error(task.get_status(), task.get_detailed_error())
        print("converting done")
        self.assertTrue(os.path.isfile(output_usd))
