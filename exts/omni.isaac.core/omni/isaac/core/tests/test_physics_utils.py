# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from omni.isaac.core.utils.physics import get_rigid_body_enabled, set_rigid_body_enabled
import omni.kit.commands
from pxr import UsdPhysics
import numpy as np


class TestPhysics(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_rigid_body_enabled(self):
        from omni.isaac.core.utils.prims import create_prim
        from omni.isaac.core.utils.stage import clear_stage

        clear_stage()
        create_prim("/World/Floor")
        cube = create_prim(
            "/World/Floor/thefloor", "Cube", position=np.array([75, 75, -150.1]), attributes={"size": 300}
        )
        create_prim("/World/Room", "Sphere", attributes={"radius": 1e3})
        UsdPhysics.RigidBodyAPI.Apply(cube)

        # None if rbapi not there
        result = get_rigid_body_enabled("/World/Room")
        self.assertEqual(result, None)

        # True is the default
        result = get_rigid_body_enabled("/World/Floor/thefloor")
        self.assertEqual(result, True)

        # Test set to False.
        set_rigid_body_enabled(False, "/World/Floor/thefloor")
        result = get_rigid_body_enabled("/World/Floor/thefloor")
        self.assertEqual(result, False)
