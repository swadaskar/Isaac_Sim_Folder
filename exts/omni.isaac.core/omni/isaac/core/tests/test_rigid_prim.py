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
from omni.isaac.core.utils.prims import define_prim
import omni.kit.test
from omni.isaac.core import World

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import update_stage_async, create_new_stage_async
import numpy as np


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRigidPrimPose(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World()
        await self._my_world.initialize_simulation_context_async()
        pass

    async def tearDown(self):
        self._my_world.clear_instance()
        await update_stage_async()
        pass

    async def test_position_orientation_scale(self):
        # Test constructor setting of pose
        position = [1.0, 2.0, 3.0]
        orientation = np.array(euler_angles_to_quat([45, -60, 180], degrees=True))
        scale = np.array([0.1, 0.1, 0.1])
        define_prim("/test", prim_type="Cube")
        await update_stage_async()
        rigid_prim = RigidPrim("/test", "test", position=np.array(position), orientation=orientation)
        rigid_prim.set_local_scale(scale)
        real_position, real_orientation = rigid_prim.get_local_pose()
        # TODO: this is buggy for some reason, world scale is equal to 1.0 in this case for some reason
        # real_scale = rigid_prim.get_world_scale()
        real_scale = rigid_prim.get_local_scale()
        for i in range(3):
            self.assertAlmostEqual(real_position[i], position[i])
            self.assertAlmostEqual(real_orientation[i], orientation[i])
            self.assertAlmostEqual(scale[i], real_scale[i])

        define_prim("/test_2", prim_type="Cube")
        await update_stage_async()
        rigid_prim = RigidPrim("/test_2", "test")
        rigid_prim.set_local_scale(scale)
        real_position, real_orientation = rigid_prim.get_local_pose()
        real_scale = rigid_prim.get_local_scale()
        for i in range(3):
            self.assertAlmostEqual(scale[i], real_scale[i])

        define_prim("/test_3", prim_type="Cube")
        rigid_prim = RigidPrim("/test_3", "test")
        rigid_prim.set_local_scale(scale)
        real_position, real_orientation = rigid_prim.get_local_pose()
        real_scale = rigid_prim.get_local_scale()
        for i in range(3):
            self.assertAlmostEqual(scale[i], real_scale[i])
        return

    async def test_set_local_pose(self):
        # Test constructor setting of pose
        position = [1.0, 2.0, 3.0]
        orientation = np.array(euler_angles_to_quat([45, -60, 180], degrees=True))
        define_prim("/test_5", prim_type="Cube")
        await update_stage_async()
        rigid_prim = RigidPrim("/test_5", "test_5", position=np.array(position), orientation=orientation)
        self._my_world.scene.add(rigid_prim)
        await self._my_world.reset_async()
        rigid_prim.set_local_pose(translation=np.array([0, 2.0, 0]))
        return
