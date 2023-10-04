# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async
from omni.isaac.core.utils.semantics import add_update_semantics
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera
import math
import asyncio


class TestCameraSensor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()
        self.my_world.scene.add_default_ground_plane()
        self.cube_2 = self.my_world.scene.add(
            DynamicCuboid(
                prim_path="/new_cube_2",
                name="cube_1",
                position=np.array([5.0, 3, 1.0]),
                scale=np.array([0.6, 0.5, 0.2]),
                size=1.0,
                color=np.array([255, 0, 0]),
            )
        )

        self.cube_3 = self.my_world.scene.add(
            DynamicCuboid(
                prim_path="/new_cube_3",
                name="cube_2",
                position=np.array([-5, 1, 3.0]),
                scale=np.array([0.1, 0.1, 0.1]),
                size=1.0,
                color=np.array([0, 0, 255]),
                linear_velocity=np.array([0, 0, 0.4]),
            )
        )
        self.xform = self.my_world.scene.add(
            XFormPrim(
                prim_path="/World/rig",
                name="rig",
                position=np.array([5.0, 0.0, 5.0]),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, -90, 0]), degrees=True),
            )
        )
        self.camera = self.my_world.scene.add(
            Camera(
                prim_path="/World/rig/camera",
                name="camera",
                position=np.array([0.0, 0.0, 25.0]),
                frequency=20,
                resolution=(256, 256),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
            )
        )
        add_update_semantics(self.cube_2.prim, "cube")
        add_update_semantics(self.cube_3.prim, "cube")
        await update_stage_async()
        await update_stage_async()
        await self.my_world.reset_async()
        await update_stage_async()
        await update_stage_async()
        return

    # After running each test
    async def tearDown(self):
        self.camera = None
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    async def test_world_poses(self):
        position, orientation = self.camera.get_world_pose()
        self.assertTrue(np.isclose(position, [0, 0, 25], atol=1e-05).all())
        self.assertTrue(
            np.isclose(
                orientation, rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True), atol=1e-05
            ).all()
        )
        translation, orientation = self.camera.get_local_pose()
        self.assertTrue(np.isclose(translation, [20, 0, 5], atol=1e-05).all())
        self.assertTrue(
            np.isclose(
                orientation, rot_utils.euler_angles_to_quats(np.array([0, 180, 0]), degrees=True), atol=1e-05
            ).all()
        )
        self.camera.set_local_pose(
            translation=[0, 0, 25], orientation=rot_utils.euler_angles_to_quats(np.array([0, 180, 0]), degrees=True)
        )
        return

    async def test_local_poses(self):
        return

    async def test_projection(self):
        for i in range(100):
            await update_stage_async()
        points_2d = self.camera.get_image_coords_from_world_points(
            np.array([self.cube_3.get_world_pose()[0], self.cube_2.get_world_pose()[0]])
        )
        # visual inspection golden values
        print(points_2d)
        self.assertTrue(np.isclose(points_2d[0], [103.51783101, 250.41131911]).all())
        self.assertTrue(np.isclose(points_2d[1], [54.40569416, 5.34284676]).all())
        points_3d = self.camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))
        self.assertTrue(np.isclose(points_3d[0], [-4.99799372, 0.99959505, 0.06]).all())
        self.assertTrue(np.isclose(points_3d[1], [4.99999901, 2.99999974, 0.1]).all())
        return

    async def test_data_acquisition(self):
        for i in range(100):
            await update_stage_async()
        self.camera.resume()
        for annotator in [
            "pointcloud",
            "normals",
            "motion_vectors",
            "occlusion",
            "distance_to_image_plane",
            "distance_to_camera",
            "bounding_box_2d_tight",
            "bounding_box_2d_loose",
            "semantic_segmentation",
            "instance_id_segmentation",
            "instance_segmentation",
        ]:
            getattr(self.camera, "add_{}_to_frame".format(annotator))()
            # frequency is set to 20, rendering rate is set to 120, so do 6 updates to make sure always have a frame
            await update_stage_async()
            await update_stage_async()
            await update_stage_async()
            await update_stage_async()
            await update_stage_async()
            await update_stage_async()
            data = self.camera.get_current_frame()
            self.assertTrue(len(data[annotator]) > 0, f"{annotator}")
            getattr(self.camera, "remove_{}_from_frame".format(annotator))()
            await update_stage_async()
            data = self.camera.get_current_frame()
            self.assertTrue(annotator not in data.keys(), f"{annotator}")
        return

    async def test_properties(self):
        self.camera.set_focal_length(5.0)
        self.assertTrue(self.camera.get_focal_length() == 5.0)
        self.camera.set_focus_distance(0.01)
        self.assertTrue(math.isclose(self.camera.get_focus_distance(), 0.01, abs_tol=0.005))
        self.camera.set_lens_aperture(0.01)
        self.assertTrue(math.isclose(self.camera.get_lens_aperture(), 0.01, abs_tol=0.005))
        self.camera.set_horizontal_aperture(1.2)
        self.assertTrue(math.isclose(self.camera.get_horizontal_aperture(), 1.2, abs_tol=0.1))
        self.camera.set_vertical_aperture(1.2)
        self.assertTrue(math.isclose(self.camera.get_vertical_aperture(), 1.2, abs_tol=0.1))
        self.camera.set_clipping_range(1.0, 1.0e5)
        clipping_range = self.camera.get_clipping_range()
        self.assertTrue(math.isclose(clipping_range[0], 1.0, abs_tol=0.1))
        self.assertTrue(math.isclose(clipping_range[1], 1.0e5, abs_tol=0.1))
        self.camera.set_projection_type("fisheyeOrthographic")
        self.assertTrue(self.camera.get_projection_type() == "fisheyeOrthographic")
        # TODO: this causes a segfault
        # self.camera.set_projection_mode("orthographic")
        # self.assertTrue(self.camera.get_projection_mode() == "orthographic")
        self.camera.set_stereo_role("left")
        self.assertTrue(self.camera.get_stereo_role() == "left")
        self.camera.set_fisheye_polynomial_properties(
            nominal_width=120,
            nominal_height=240,
            optical_centre_x=24,
            optical_centre_y=25,
            max_fov=560,
            polynomial=[1, 2, 3, 4, 5],
        )
        nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, polynomial = (
            self.camera.get_fisheye_polynomial_properties()
        )
        self.assertTrue(math.isclose(nominal_width, 120, abs_tol=2))
        self.assertTrue(math.isclose(nominal_height, 240, abs_tol=2))
        self.assertTrue(math.isclose(optical_centre_x, 24, abs_tol=2))
        self.assertTrue(math.isclose(optical_centre_y, 25, abs_tol=2))
        self.assertTrue(math.isclose(max_fov, 560, abs_tol=2))
        self.assertTrue(np.isclose(polynomial, [1, 2, 3, 4, 5]).all())
        self.camera.set_shutter_properties(delay_open=2.0, delay_close=3.0)
        delay_open, delay_close = self.camera.get_shutter_properties()
        self.assertTrue(math.isclose(delay_open, 2.0, abs_tol=0.1))
        self.assertTrue(math.isclose(delay_close, 3.0, abs_tol=0.1))
        self.camera.set_resolution((300, 300))
        resolution = self.camera.get_resolution()
        self.assertTrue(math.isclose(resolution[0], 300, abs_tol=0.1))
        self.assertTrue(math.isclose(resolution[1], 300, abs_tol=0.1))
        self.camera.get_aspect_ratio()
        self.camera.get_horizontal_fov()
        self.camera.get_vertical_fov()
        return
