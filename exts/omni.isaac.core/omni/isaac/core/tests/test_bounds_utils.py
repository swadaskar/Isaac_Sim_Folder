# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from scipy.spatial.transform import Rotation
import numpy as np
from pxr import UsdGeom, Usd, Gf
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.bounds import compute_aabb, compute_combined_aabb, create_bbox_cache, recompute_extents
import omni.kit.commands


class TestBounds(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_recompute_extents(self):
        stage = get_current_stage()
        cubeGeom = UsdGeom.Cube.Define(stage, "/cube_shape")
        cubePrim = stage.GetPrimAtPath("/cube_shape")
        size = 123
        offset = (100, 200, 300)
        cubeGeom.AddTranslateOp().Set(offset)
        cubeGeom.CreateSizeAttr(size)
        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache()
        recompute_extents(cubeGeom)
        # it shouldn't error even if we cast it to a base prim
        recompute_extents(cubeGeom.GetPrim())
        await omni.kit.app.get_app().next_update_async()
        aabb = compute_aabb(cache, "/cube_shape")
        self.assertListEqual(aabb.tolist(), [38.5, 138.5, 238.5, 161.5, 261.5, 361.5])

        result, cube_path = omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube")
        await omni.kit.app.get_app().next_update_async()
        cube_prim = stage.GetPrimAtPath(cube_path)
        cube_mesh = UsdGeom.Mesh(cube_prim)
        points = cube_mesh.GetPointsAttr().Get()

        for i in range(len(points)):
            points[i] = points[i] * 1.5
        cube_mesh.GetPointsAttr().Set(points)

        # recompute extents after changing points
        await omni.kit.app.get_app().next_update_async()
        recompute_extents(cube_prim)
        await omni.kit.app.get_app().next_update_async()

        aabb = compute_aabb(cache, cube_path)
        self.assertListEqual(aabb.tolist(), [-0.75, -0.75, -0.75, 0.75, 0.75, 0.75])
        combined_aabb = compute_combined_aabb(cache, ["/cube_shape", cube_path])
        self.assertListEqual(combined_aabb.tolist(), [-0.75, -0.75, -0.75, 161.5, 261.5, 361.5])
        # this should be the same as including children when calculating bbox for entire scene
        aabb_with_children = compute_aabb(cache, "/", include_children=True)
        self.assertListEqual(combined_aabb.tolist(), aabb_with_children.tolist())

    async def test_nested_recompute_extents(self):
        stage = get_current_stage()
        cubeA = UsdGeom.Cube.Define(stage, "/nested_cube")
        cubeB = UsdGeom.Cube.Define(stage, "/nested_cube/nested_cube")
        cubeC = UsdGeom.Cube.Define(stage, "/nested_cube/nested_cube/nested_cube")
        size = 123
        offset = (100, 200, 300)
        cubeA.AddTranslateOp().Set(offset)
        cubeA.CreateSizeAttr(size)

        cubeB.AddTranslateOp().Set(offset)
        cubeB.CreateSizeAttr(size)

        cubeC.AddTranslateOp().Set(offset)
        cubeC.CreateSizeAttr(size)

        await omni.kit.app.get_app().next_update_async()
        recompute_extents(cubeA, include_children=False)
        await omni.kit.app.get_app().next_update_async()
        cache = create_bbox_cache(use_extents_hint=False)
        await omni.kit.app.get_app().next_update_async()

        aabb = compute_aabb(cache, "/nested_cube")
        self.assertListEqual(aabb.tolist(), [38.5, 138.5, 238.5, 161.5, 261.5, 361.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube")
        self.assertListEqual(aabb.tolist(), [199.0, 399.0, 599.0, 201.0, 401.0, 601.0])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube/nested_cube")
        self.assertListEqual(aabb.tolist(), [299.0, 599.0, 899.0, 301.0, 601.0, 901.0])
        recompute_extents(cubeA, include_children=True)
        cache = create_bbox_cache()

        aabb = compute_aabb(cache, "/nested_cube")
        self.assertListEqual(aabb.tolist(), [38.5, 138.5, 238.5, 161.5, 261.5, 361.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube")
        self.assertListEqual(aabb.tolist(), [138.5, 338.5, 538.5, 261.5, 461.5, 661.5])
        aabb = compute_aabb(cache, "/nested_cube/nested_cube/nested_cube")
        self.assertListEqual(aabb.tolist(), [238.5, 538.5, 838.5, 361.5, 661.5, 961.5])

        # This should not fail
        recompute_extents(cubeA.GetPrim(), include_children=True)
