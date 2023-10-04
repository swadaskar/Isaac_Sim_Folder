# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.kit.test
from omni.isaac.core.utils.prims import get_all_matching_child_prims
import omni.kit.commands
import numpy as np
import torch


class TestPrims(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_get_all_matching_child_prims(self):
        from omni.isaac.core.utils.prims import create_prim, get_prim_path
        from omni.isaac.core.utils.stage import clear_stage

        clear_stage()
        create_prim("/World/Floor")
        create_prim("/World/Floor/thefloor", "Cube", position=np.array([75, 75, -150.1]), attributes={"size": 300})
        create_prim("/World/Room", "Sphere", attributes={"radius": 1e3})

        result = get_all_matching_child_prims("/World")
        result = [get_prim_path(prim) for prim in result]
        self.assertListEqual(result, ["/World", "/World/Floor", "/World/Room", "/World/Floor/thefloor"])

    async def test_create_prim(self):
        from omni.isaac.core.utils.prims import create_prim, get_prim_path
        from omni.isaac.core.utils.stage import clear_stage

        clear_stage()
        create_prim("/World")
        create_prim(
            "/World/thebox", "Cube", position=[175, 75, 0.0], orientation=[0.0, 0.0, 0.0, 1.0], attributes={"size": 150}
        )
        create_prim(
            "/World/thechair1",
            "Cube",
            position=(-75, 75, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            attributes={"size": 150},
        )
        create_prim("/World/thechair2", "Cube", position=np.array([75, 75, 0.0]), attributes={"size": 150})
        create_prim("/World/thetable", "Cube", position=torch.Tensor([-175, 75, 0.0]), attributes={"size": 150})

        result = get_all_matching_child_prims("/World")
        result = [get_prim_path(prim) for prim in result]
        self.assertListEqual(
            result, ["/World", "/World/thebox", "/World/thechair1", "/World/thechair2", "/World/thetable"]
        )

    async def test_is_prim_non_root_articulation_link(self):
        from omni.isaac.core.utils.stage import clear_stage
        from omni.isaac.core.utils.prims import is_prim_non_root_articulation_link
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        from omni.isaac.core.objects import DynamicCuboid

        clear_stage()
        add_reference_to_stage(usd_path="", prim_path="/World/Franka")
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise Exception("Asset root path doesn't exist")
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        DynamicCuboid(prim_path="/World/Franka/panda_link1/Cube")
        XFormPrim(prim_path="/World/Franka/panda_link1/test_1")
        XFormPrim(prim_path="/World/Franka/test_1")
        self.assertFalse(is_prim_non_root_articulation_link(prim_path="/World/Franka"))
        self.assertTrue(is_prim_non_root_articulation_link(prim_path="/World/Franka/panda_link1"))
        self.assertTrue(is_prim_non_root_articulation_link(prim_path="/World/Franka/panda_link0"))
        self.assertFalse(is_prim_non_root_articulation_link(prim_path="/World/Franka/panda_link1/test_1"))
        self.assertFalse(is_prim_non_root_articulation_link(prim_path="/World/Franka/test_1"))
