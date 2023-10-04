# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import omni.kit.commands
import carb

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.range_sensor import _range_sensor
from omni.syntheticdata.tests.utils import add_semantics
from pxr import UsdGeom, UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools, Semantics
import numpy as np
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestLidar(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()

        # light
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # set up axis to z
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        # Physics scene
        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.range_sensor")
        self._extension_path = ext_manager.get_extension_path(ext_id)

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        pass

    async def sweep_parameter(self, parameter, min_v, max_v, step):
        print(parameter.GetName())
        for value in np.arange(min_v, max_v, step):
            # print(value)
            parameter.Set(float(value))
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

    async def add_cube(self, path, size, offset):

        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)

        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)
        UsdPhysics.CollisionAPI.Apply(cubePrim)

        return cubeGeom

    # Tests a static lidar with a cube in front of it
    async def test_static_lidar(self):
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.250))

        # Run for a second
        self._timeline.play()
        await simulate_async(1.0)
        self._timeline.pause()

        # Get depth, and check that we hit the cube in front, and hit nothing in back
        depth = self._lidar.get_depth_data(lidarPath)

        self.assertLess(depth[0, 0], 2000)
        self.assertEqual(depth[450, 0], 65535)
        self._timeline.play()

    # Tests a lidar on a falling cube, with a cube in front of it after it lands
    async def test_dynamic_lidar(self):
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )
        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add falling cube
        cubePath2 = "/World/Cube2"
        await self.add_cube(cubePath2, 0.500, Gf.Vec3f(0.0, 0.0, 2.500))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent="/World/Cube2",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.500))
        self._timeline.play()
        # get data before it falls and make sure that lidar is parented properly and does not have block infront of it
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        depth = self._lidar.get_depth_data(lidarPath)
        self.assertEqual(depth[0, 0], 65535)

        # wait for it to drop
        await simulate_async(2.0)
        self._timeline.pause()
        # Get depth, and check that we hit the cube in front, and hit nothing in back
        depth = self._lidar.get_depth_data(lidarPath)
        self.assertLess(depth[0, 0], 2000)
        self.assertEqual(depth[450, 0], 65535)

    async def test_parameter_ranges(self):
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )
        cubePath2 = "/World/Cube2"
        await self.add_cube(cubePath2, 50.0, Gf.Vec3f(0.0, 0.0, 250.0))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Cube2/Lidar",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=True,
            yaw_offset=0.0,
        )

        self._timeline.play()
        lidar.GetHighLodAttr().Set(True)
        lidar.GetDrawPointsAttr().Set(False)
        await self.sweep_parameter(lidar.GetRotationRateAttr(), -1024, 1024, 256)
        lidar.GetRotationRateAttr().Set(0)
        await self.sweep_parameter(lidar.GetHorizontalFovAttr(), -1024, 1024, 256)
        lidar.GetHorizontalFovAttr().Set(360)
        await self.sweep_parameter(lidar.GetVerticalFovAttr(), -1024, 1024, 256)
        lidar.GetHorizontalFovAttr().Set(120)
        lidar.GetVerticalFovAttr().Set(30)
        await self.sweep_parameter(lidar.GetHorizontalResolutionAttr(), -0.1, 1.0, 0.1)
        await self.sweep_parameter(lidar.GetVerticalResolutionAttr(), -0.1, 1.0, 0.1)
        await self.sweep_parameter(lidar.GetMinRangeAttr(), -1024, 1024, 256)
        await self.sweep_parameter(lidar.GetMaxRangeAttr(), -1024, 1024, 256)
        lidar.GetHighLodAttr().Set(False)

    async def test_carter_lidar(self):
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        (result, error) = await open_stage_async(self._assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd")
        self._stage = omni.usd.get_context().get_stage()

        # Add a cube
        cubePath = "/Cube"
        await self.add_cube(cubePath, 0.750, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/NewLidar",
            parent="/carter/chassis_link",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(-0.060, 0.0, 0.370))

        # Run for two seconds

        self._timeline.play()
        await simulate_async(2.0)
        self._timeline.pause()
        # Get depth, and check that we hit the cube in front, and hit nothing in back
        depth = self._lidar.get_depth_data(lidarPath)
        self.assertLess(depth[0, 0], 2000)
        self.assertEqual(depth[450, 0], 65535)

    # Prints out average fps for an expensive lidar
    async def test_lidar_fps(self):
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=45.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.4,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
        )

        # Run for a second
        self._timeline.play()
        for frame in range(int(60 * 5)):
            await omni.kit.app.get_app().next_update_async()
            if frame % 60 == 0:
                print("FPS: ", viewport_api.fps)
        self._timeline.pause()

        self._timeline.play()

    # Tests a static lidar with a cube in front of it and get semantic id for hit points
    async def test_static_lidar_semantic(self):
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, -1.000, 0.500))
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        add_semantics(cubePrim, "cube")

        # Add another cube
        cubePath = "/World/Cube1"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 1.000, 0.500))
        cubePrim = self._stage.GetPrimAtPath(cubePath)
        add_semantics(cubePrim, "cube1")

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=True,
        )
        lidarPath = str(lidar.GetPath())

        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.250))

        # Run for a second
        self._timeline.play()
        await simulate_async(1.0)
        self._timeline.pause()

        # Get semantic data of hit points, and check that we get two non-zero semantic IDs
        semantic = self._lidar.get_semantic_data(lidarPath)

        # print(np.unique(semantic))
        self.assertEqual(len(np.unique(semantic)), 3)
        self._timeline.play()

    # test currently not working
    # async def test_raycast_targets(self):

    #     # Add lidar
    #     lidarPath = "/World/Lidar"
    #     lidar = self.add_lidar(lidarPath)

    #     lidar.GetRotationRateAttr().Set(0.0)
    #     lidar.GetHighLodAttr().Set(True)

    #     vFOV = 45
    #     hFOV = 360
    #     vRes = 3
    #     hRes = 3

    #     vFOV = float(np.clip(vFOV, 0, 180))
    #     hFOV = float(np.clip(hFOV, 0, 360))

    #     azimuth = np.arange(-hFOV / 2, hFOV / 2, hRes)
    #     zenith = np.arange(90 - vFOV / 2, 90 + vFOV / 2, vRes)

    #     lidar.GetHorizontalFovAttr().Set(hFOV)
    #     lidar.GetVerticalFovAttr().Set(vFOV)
    #     lidar.GetHorizontalResolutionAttr().Set(hRes)
    #     lidar.GetVerticalResolutionAttr().Set(vRes)

    #     # Run for a second
    #     self._timeline.play()
    #     await simulate_async(2.0)
    #     self._timeline.pause()

    #     kitZenith = self._lidar.get_zenith_data(lidarPath)
    #     kitAzimuth = self._lidar.get_azimuth_data(lidarPath)

    #     self.assertEqual(len(list(zenith.flatten())), len(kitZenith))
    #     self.assertEqual(len(list(azimuth.flatten())), len(kitAzimuth))

    #     for (a, b) in zip(list(zenith.flatten()), kitZenith):
    #         epsilon = vRes / 10.0
    #         self.assertTrue(a <= b + epsilon)
    #         self.assertTrue(a >= b - epsilon)

    #     for (a, b) in zip(list(azimuth.flatten()), kitAzimuth):
    #         epsilon = hRes / 10.0
    #         self.assertTrue(a <= b + epsilon)
    #         self.assertTrue(a >= b - epsilon)
