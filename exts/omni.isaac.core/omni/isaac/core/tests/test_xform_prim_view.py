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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.prims import XFormPrimView
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
import numpy as np
from omni.isaac.core.utils.stage import create_new_stage_async, add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
from pxr import UsdGeom


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestXFormPrimView(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World()
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
        define_prim(prim_path="/World/Frame_1")
        define_prim(prim_path="/World/Frame_2")
        define_prim(prim_path="/World/Frame_3")
        define_prim(prim_path="/World/Frame_1/Target")
        define_prim(prim_path="/World/Frame_2/Target")
        define_prim(prim_path="/World/Frame_3/Target")
        self._frankas_view = XFormPrimView(prim_paths_expr="/World/Franka_[1-2]", name="frankas_view")
        self._targets_view = XFormPrimView(prim_paths_expr="/World/Frame_[1-3]/Target", name="targets_view")
        self._frames_view = XFormPrimView(prim_paths_expr="/World/Frame_[1-3]", name="frames_view")
        pass

    async def tearDown(self):
        self._my_world.clear_instance()

    async def test_world_poses(self):
        current_positions, current_orientations = self._frankas_view.get_world_poses()
        self.assertTrue(np.isclose(current_positions, np.zeros([2, 3], dtype=np.float32)).all())
        expected_orientations = np.zeros([2, 4], dtype=np.float32)
        expected_orientations[:, 0] = 1
        self.assertTrue(np.isclose(current_orientations, expected_orientations).all())

        new_positions = np.array([[10.0, 10.0, 0], [-40, -40, 0]])
        new_orientations = euler_angles_to_quats(np.array([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]]))
        self._frankas_view.set_world_poses(positions=new_positions, orientations=new_orientations)
        current_positions, current_orientations = self._frankas_view.get_world_poses()
        self.assertTrue(np.isclose(current_positions, new_positions).all())
        self.assertTrue(
            np.logical_or(
                np.isclose(current_orientations, new_orientations, atol=1e-05).all(axis=1),
                np.isclose(current_orientations, -new_orientations, atol=1e-05).all(axis=1),
            ).all()
        )
        return

    async def test_local_pose(self):
        print(euler_angles_to_quats(np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])))
        self._frames_view.set_local_poses(
            translations=np.array([[0, 0, 0], [0, 10, 5], [0, 3, 5]]),
            orientations=euler_angles_to_quats(np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])),
        )
        self._targets_view.set_local_poses(translations=np.array([[0, 20, 10], [0, 30, 20], [0, 50, 10]]))
        current_translations, current_orientations = self._targets_view.get_local_poses()
        self.assertTrue(np.isclose(current_translations, np.array([[0, 20, 10], [0, 30, 20], [0, 50, 10]])).all())
        return

    async def test_local_on_init(self):
        initial_translations = np.array([[0, 0, 0], [0, 10, 5]])
        view = XFormPrimView(
            prim_paths_expr="/World/Franka_[1-2]", name="frankas_view_2", translations=initial_translations
        )
        current_translations, current_orientations = view.get_local_poses()
        self.assertTrue(np.isclose(current_translations, initial_translations).all())
        return

    async def test_visibilities(self):
        prim_paths = "/World/Franka_[1-2]"
        visibilities = np.array([False, True])
        view = XFormPrimView(prim_paths_expr=prim_paths, visibilities=visibilities)
        for i in range(len(view.prim_paths)):
            imageable = UsdGeom.Imageable(view.prims[i])
            visibility_attr = imageable.GetVisibilityAttr().Get()
            if visibilities[i]:
                self.assertEqual(visibility_attr, "inherited")
            else:
                self.assertEqual(visibility_attr, "invisible")
