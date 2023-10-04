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


class TestRotations(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_euler_angles_to_quat(self):
        roll, pitch, yaw = np.pi * np.random.rand(3)
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(np.all(euler_angles_to_quat(np.array([roll, pitch, yaw])) == np.array([w, x, y, z])))
        pass

    async def test_quat_to_euler_angles(self):
        roll, pitch, yaw = np.pi * np.random.rand(3)
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
        )
        roll, pitch, yaw = [0.94965366, 2.3579875, 1.43057573]
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
        )
        pass
