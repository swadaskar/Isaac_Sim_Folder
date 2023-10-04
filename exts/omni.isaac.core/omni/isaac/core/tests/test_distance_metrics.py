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
from omni.isaac.core.utils.distance_metrics import *

import numpy as np
from pxr import Gf
from scipy.spatial.transform import Rotation as R

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDistanceMetrics(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    async def tearDown(self):
        pass

    async def is_distance_metric(self, t1, t2, t3, dist_fun, *args):
        # satisfies identity
        self.assertAlmostEqual(dist_fun(t1, t1, *args), 0)
        self.assertAlmostEqual(dist_fun(t2, t2, *args), 0)
        self.assertAlmostEqual(dist_fun(t3, t3, *args), 0)

        # satisfies symmetry
        self.assertAlmostEqual(dist_fun(t1, t2, *args), dist_fun(t2, t1, *args))
        self.assertAlmostEqual(dist_fun(t1, t3, *args), dist_fun(t3, t1, *args))
        self.assertAlmostEqual(dist_fun(t3, t2, *args), dist_fun(t2, t3, *args))

        # satisfies triangle inequality
        self.assertGreaterEqual(dist_fun(t1, t2, *args) + dist_fun(t2, t3, *args), dist_fun(t1, t3, *args))
        self.assertGreaterEqual(dist_fun(t1, t3, *args) + dist_fun(t2, t3, *args), dist_fun(t1, t2, *args))
        self.assertGreaterEqual(dist_fun(t1, t2, *args) + dist_fun(t1, t3, *args), dist_fun(t2, t3, *args))

        pass

    """
    There are distance metrics implemented for 3d translation and rotation independently.  The inputs
    to these functions can be 4x4 transformation matrices, or they can be purely translations/rotations.
    """

    async def test_weighted_translational_distance(self):
        # Check that the conditions for being a distance metric are met for 200 random translations/weights.
        for i in range(200):
            transform1 = np.random.uniform(size=(4, 4))
            transform2 = np.random.uniform(size=(4, 4))
            transform3 = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*np.random.uniform(size=3)))

            translate1 = transform1[:3, 3]
            translate2 = transform2[:3, 3]

            # Check that translation is extracted properly from transformation matrices.
            self.assertTrue(
                weighted_translational_distance(translate1, translate2)
                == weighted_translational_distance(transform1, transform2)
            )

            # A weight matrix can be added to the distance calculation to put weight on a specific axis.
            weight_matrix = np.eye(3)
            weight_matrix[0, 0] = np.random.uniform()
            weight_matrix[1, 1] = np.random.uniform()
            weight_matrix[2, 2] = np.random.uniform()  # z distance is multiplied by 3

            await self.is_distance_metric(
                transform1, transform2, transform3, weighted_translational_distance, weight_matrix
            )

        """
        The weight matrix can be used to weight the difference of two translations along one or more 
        arbitrary axes
        """

        # Example: Only weight the difference along the z axis of translate1 - translate2.
        weight_matrix[0, 0] = 0
        weight_matrix[1, 1] = 0
        weight_matrix[2, 2] = 3
        self.assertAlmostEqual(
            weighted_translational_distance(translate1, translate2, weight_matrix),
            np.sqrt(3) * abs(translate1[-1] - translate2[-1]),
        )

        # Example: Weight the difference along the axis [1,1,0] by sqrt(2)
        translate1 = np.zeros(3)
        # translate2 is made of 3 basis vectors. The weight will cause the norm of each basis vector to be one
        translate2 = np.array([1, 1, 0]) / 2 + np.array([-1, 1, 0]) / np.sqrt(2) + np.array([0, 0, 1])
        weight_matrix = np.eye(3)
        weight_matrix[0, 0] = 2  # weight the x axis
        rot = R.from_rotvec([0, 0, -np.pi / 4]).as_matrix()  # rotates [1,1,0] onto the x axis
        # Thus the [1,1,0] axis is weighted by sqrt(2) in the resulting weight matrix
        weight_matrix = rot.T @ weight_matrix @ rot
        self.assertAlmostEqual(weighted_translational_distance(translate1, translate2, weight_matrix), np.sqrt(3))

        pass

    async def test_rotational_distance_angle(self):
        rad2deg = 360 / np.pi / 2
        transform1_gf = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), np.pi / 4 * rad2deg))
        transform1_np = np.eye(4)
        transform1_np[:3, :3] = R.from_rotvec([0, 0, np.pi / 4]).as_matrix()

        rotate1_np = R.from_rotvec([0, 0, np.pi]).as_matrix()

        # Rotational distance is equal to the magnitude of the angle from r1 ro r2.
        self.assertAlmostEqual(rotational_distance_angle(transform1_gf, transform1_np), 0)
        self.assertAlmostEqual(rotational_distance_angle(transform1_np, rotate1_np), 3 * np.pi / 4)
        self.assertAlmostEqual(rotational_distance_angle(transform1_gf, rotate1_np), 3 * np.pi / 4)

        # Check that the conditions for being a distance metric are met for 200 random rotations.
        for i in range(200):
            n = np.random.uniform(size=3)
            r1 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            n = np.random.uniform(size=3)
            r2 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            r3 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*np.random.uniform(size=3)), np.random.uniform(high=180)))

            await self.is_distance_metric(r1, r2, r3, rotational_distance_angle)

        pass

    async def test_rotational_distance_identity_matrix_deviation(self):
        # Check that the conditions for being a distance metric are met for 200 random rotations.
        for i in range(200):
            n = np.random.uniform(size=3)
            r1 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            n = np.random.uniform(size=3)
            r2 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            r3 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*np.random.uniform(size=3)), np.random.uniform(high=180)))

            await self.is_distance_metric(r1, r2, r3, rotational_distance_identity_matrix_deviation)

        pass

    async def test_rotational_distance_single_axis(self):
        r1 = R.from_rotvec([0, 0, np.pi / 2]).as_matrix()
        r2 = R.from_rotvec([0.1, 0, 0]).as_matrix()

        axis = [0, 0, 1]  # ignore rotation about the z axis

        self.assertAlmostEqual(rotational_distance_single_axis(r1, r2, axis), 0.1)

        # Check that the conditions for being a distance metric are met for 200 random rotations.
        for i in range(200):
            n = np.random.uniform(size=3)
            r1 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            n = np.random.uniform(size=3)
            r2 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            r3 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*np.random.uniform(size=3)), np.random.uniform(high=180)))

            axis = np.random.uniform(size=3)
            await self.is_distance_metric(r1, r2, r3, rotational_distance_single_axis, axis)

        pass
