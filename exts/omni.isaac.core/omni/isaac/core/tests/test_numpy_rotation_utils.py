# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test

import omni.isaac.core.utils.numpy.rotations as rotation_conversions
import numpy as np

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRotationUtils(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_rotation_conversions(self):
        # ith index of rotations are equivalent

        quats = np.array([[0.4374512, 0.4021533, 0.8043066, 0], [0.7602446, -0.4593627, 0, 0.4593627]])
        euler_angs = np.array([[2.62352489, 0.78057745, 1.99712744], [-0.879511, 0.43568131, 0.879511]])
        rot_vecs = np.array([[1, 2, 0], [-1, 0, 1]])

        rot_mats = np.array(
            [
                [
                    [-0.2938183, 0.6469092, 0.7036898],
                    [0.6469092, 0.6765454, -0.3518449],
                    [-0.7036898, 0.3518449, -0.6172729],
                ],
                [
                    [0.5779719, -0.6984560, -0.4220282],
                    [0.6984560, 0.1559437, 0.6984560],
                    [-0.4220282, -0.6984560, 0.5779719],
                ],
            ]
        )

        self.assertTrue(np.all(np.isclose(rotation_conversions.quats_to_rot_matrices(quats), rot_mats, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.quats_to_rotvecs(quats), rot_vecs, rtol=1e-3)))

        self.assertTrue(np.all(np.isclose(rotation_conversions.quats_to_euler_angles(quats), euler_angs, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.rot_matrices_to_quats(rot_mats), quats, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.euler_angles_to_quats(euler_angs), quats, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.rotvecs_to_quats(rot_vecs), quats, rtol=1e-3)))

        self.assertTrue(rotation_conversions.quats_to_rot_matrices(quats[0]).shape == (3, 3))
        self.assertTrue(rotation_conversions.quats_to_rotvecs(quats[0]).shape == (3,))
        self.assertTrue(rotation_conversions.quats_to_euler_angles(quats[0]).shape == (3,))
        self.assertTrue(rotation_conversions.rot_matrices_to_quats(rot_mats[0]).shape == (4,))
        self.assertTrue(rotation_conversions.rotvecs_to_quats(rot_vecs[0]).shape == (4,))
        self.assertTrue(rotation_conversions.euler_angles_to_quats(euler_angs[0]).shape == (4,))
