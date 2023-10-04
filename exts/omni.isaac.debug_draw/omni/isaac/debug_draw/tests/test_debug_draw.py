# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
from omni.isaac.debug_draw import _debug_draw
import random

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDebugDraw(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        self._draw = _debug_draw.acquire_debug_draw_interface()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_draw_points(self):
        N = 10000
        point_list_1 = [
            (random.uniform(-1000, 1000), random.uniform(-1000, 1000), random.uniform(-1000, 1000)) for _ in range(N)
        ]
        point_list_2 = [
            (random.uniform(-1000, 1000), random.uniform(1000, 3000), random.uniform(-1000, 1000)) for _ in range(N)
        ]
        point_list_3 = [
            (random.uniform(-1000, 1000), random.uniform(-3000, -1000), random.uniform(-1000, 1000)) for _ in range(N)
        ]
        colors = [(random.uniform(0.5, 1), random.uniform(0.5, 1), random.uniform(0.5, 1), 1) for _ in range(N)]
        sizes = [random.randint(1, 50) for _ in range(N)]
        self._draw.draw_points(point_list_1, [(1, 0, 0, 1)] * N, [10] * N)
        self._draw.draw_points(point_list_2, [(0, 1, 0, 1)] * N, [10] * N)
        self._draw.draw_points(point_list_3, colors, sizes)
        self.assertEqual(self._draw.get_num_points(), 3 * N)
        self._draw.clear_points()
        self.assertEqual(self._draw.get_num_points(), 0)
        pass

    async def test_draw_lines(self):
        N = 10000
        point_list_1 = [
            (random.uniform(1000, 3000), random.uniform(-1000, 1000), random.uniform(-1000, 1000)) for _ in range(N)
        ]
        point_list_2 = [
            (random.uniform(1000, 3000), random.uniform(-1000, 1000), random.uniform(-1000, 1000)) for _ in range(N)
        ]
        colors = [(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), 1) for _ in range(N)]
        sizes = [random.randint(1, 25) for _ in range(N)]
        self._draw.draw_lines(point_list_1, point_list_2, colors, sizes)
        self.assertEqual(self._draw.get_num_lines(), N)
        self._draw.clear_lines()
        self.assertEqual(self._draw.get_num_lines(), 0)
        pass

    async def test_draw_spline(self):
        point_list_1 = [
            (random.uniform(-300, -100), random.uniform(-100, 100), random.uniform(-100, 100)) for _ in range(10)
        ]
        self._draw.draw_lines_spline(point_list_1, (1, 1, 1, 1), 10, False)
        point_list_2 = [
            (random.uniform(-300, -100), random.uniform(-100, 100), random.uniform(-100, 100)) for _ in range(10)
        ]
        self._draw.draw_lines_spline(point_list_2, (1, 1, 1, 1), 5, True)

        self.assertGreater(self._draw.get_num_lines(), 0)
        self._draw.clear_lines()
        self.assertEqual(self._draw.get_num_lines(), 0)
        pass
