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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import numpy as np


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestXformPrimPose(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_position_orientation_scale(self):
        # Test constructor setting of pose
        position = [1.0, 2.0, 3.0]
        orientation = np.array(euler_angles_to_quat([45, -60, 180], degrees=True))
        scale = np.array([0.1, 0.1, 0.1])
        xform_prim = XFormPrim("/test", "test", position=np.array(position), orientation=orientation, scale=scale)

        real_position, real_orientation = xform_prim.get_local_pose()
        real_scale = xform_prim.get_world_scale()
        for i in range(3):
            self.assertAlmostEqual(real_position[i], position[i])
            self.assertAlmostEqual(real_orientation[i], orientation[i])
            self.assertAlmostEqual(scale[i], real_scale[i])

        xform_prim = XFormPrim("/test_2", "test", scale=scale)
        real_position, real_orientation = xform_prim.get_local_pose()
        real_scale = xform_prim.get_world_scale()
        for i in range(3):
            print(scale[i])
            self.assertAlmostEqual(scale[i], real_scale[i])

        xform_prim = XFormPrim("/test_3", "test")

        xform_prim.set_local_scale(scale)
        real_position, real_orientation = xform_prim.get_local_pose()
        real_scale = xform_prim.get_world_scale()
        for i in range(3):
            print(scale[i])
            self.assertAlmostEqual(scale[i], real_scale[i])
