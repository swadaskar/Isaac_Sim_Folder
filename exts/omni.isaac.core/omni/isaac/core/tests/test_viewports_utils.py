# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from omni.isaac.core.utils.viewports import (
    get_intrinsics_matrix,
    get_window_from_id,
    get_id_from_index,
    set_intrinsics_matrix,
    get_viewport_names,
    destroy_all_viewports,
)
import carb
from pxr import Sdf
import numpy as np
from omni.kit.viewport.utility import (
    get_active_viewport,
    create_viewport_window,
    get_active_viewport_window,
    get_num_viewports,
)


class TestViewports(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_get_intrinsics(self):
        viewport_api = get_active_viewport()
        viewport_api.set_texture_resolution((800, 600))
        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)
        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=6, prev=0
        )

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6, prev=0
        )

        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)

        self.assertAlmostEqual(matrix[0, 0], 2419, delta=1)
        self.assertAlmostEqual(matrix[0, 2], 400, delta=1)
        self.assertAlmostEqual(matrix[1, 1], 1814, delta=1)
        self.assertAlmostEqual(matrix[1, 2], 300, delta=1)
        pass

    async def test_set_intrinsics(self):
        viewport_api = get_active_viewport()
        viewport_api.set_texture_resolution((800, 600))
        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)
        matrix = np.array([[3871, 0.0, 400.0], [0.0, 2177, 300.0], [0.0, 0.0, 1.0]])
        set_intrinsics_matrix(viewport_api, matrix)
        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)
        self.assertAlmostEqual(matrix[0, 0], 3871, delta=1)
        self.assertAlmostEqual(matrix[0, 2], 400, delta=1)
        self.assertAlmostEqual(matrix[1, 1], 2177, delta=1)
        self.assertAlmostEqual(matrix[1, 2], 300, delta=1)
        pass

    async def test_get_viewport_names(self):
        self.assertEquals(len(get_viewport_names()), 1)
        await omni.kit.app.get_app().next_update_async()
        window_1 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        window_2 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        self.assertEquals(len(get_viewport_names()), 3)
        window_1.destroy()
        await omni.kit.app.get_app().next_update_async()
        window_2.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_window_from_id(self):
        window_0 = get_active_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        window_1 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        window_2 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()

        print(window_0.title, window_0.viewport_api.id)
        print(window_1.title, window_1.viewport_api.id)
        print(window_2.title, window_2.viewport_api.id)

        window_test = get_window_from_id(window_1.viewport_api.id)
        self.assertEquals(window_test.title, window_1.title)
        window_test = get_window_from_id(1000)
        self.assertIsNone(window_test)
        window_1.destroy()
        await omni.kit.app.get_app().next_update_async()
        window_2.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_id_from_index(self):
        window_0 = get_active_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        # get the first viewport and check if titles match
        window_test = get_window_from_id(get_id_from_index(0))
        self.assertEquals(window_test.title, window_0.title)
        # second viewport should not exist yet
        window_test = get_window_from_id(get_id_from_index(1))
        self.assertIsNone(window_test)
        # create second viewport
        window_1 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()

        window_test = get_window_from_id(get_id_from_index(1))
        self.assertIsNotNone(window_test)
        self.assertEquals(window_test.title, window_1.title)
        window_1.destroy()

    async def test_create_destroy_window(self):
        from omni.kit.viewport.utility import create_viewport_window

        window_1 = create_viewport_window()
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
        window_1.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_destroy_windows(self):
        from omni.kit.viewport.utility import create_viewport_window

        window_1 = create_viewport_window()
        window_2 = create_viewport_window()
        window_3 = create_viewport_window()
        window_4 = create_viewport_window()
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        destroy_all_viewports(None, False)
