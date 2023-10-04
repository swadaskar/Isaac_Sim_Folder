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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.range_sensor import _range_sensor
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools, UsdShade
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from omni.isaac.core.utils.physics import simulate_async
import numpy as np
import carb.tokens


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestUltrasonic(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):

        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))

        self._ultrasonic = _range_sensor.acquire_ultrasonic_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

        # light
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(6.500, 0.0, 11.500))

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
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def sweep_parameter(self, parameter, min_v, max_v, step):
        print(parameter.GetName())
        for value in np.arange(min_v, max_v, step):
            # print(value)
            parameter.Set(float(value))
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

    # Create a cube, set physics to False to make it static with collision only
    async def add_cube(self, path, size, offset, physics=True):

        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        if physics:
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
            rigid_api.CreateRigidBodyEnabledAttr(True)
        UsdPhysics.CollisionAPI.Apply(cubePrim)

        return cubePrim

    def get_full_array_poses(self):
        emitter_poses = [
            (Gf.Quatd(0.951057, 0, 0, -0.309017), Gf.Vec3d(0.25, 0.0, 0.25)),
            (Gf.Quatd(0.987688, 0, 0, -0.156434), Gf.Vec3d(0.25, 0.500, 0.25)),
            (Gf.Quatd(0.987688, 0, 0, 0.156434), Gf.Vec3d(0.25, 1.00, 0.25)),
            (Gf.Quatd(0.951057, 0, 0, 0.309017), Gf.Vec3d(0.25, 1.50, 0.25)),
            (Gf.Quatd(-0.309017, 0, 0, 0.951056), Gf.Vec3d(-0.25, 0.0, 0.25)),
            (Gf.Quatd(-0.156435, 0, 0, 0.987688), Gf.Vec3d(-0.25, 0.500, 0.25)),
            (Gf.Quatd(0.156434, 0, 0, 0.987688), Gf.Vec3d(-0.25, 1.00, 0.25)),
            (Gf.Quatd(0.309017, 0, 0, 0.951057), Gf.Vec3d(-0.25, 1.50, 0.25)),
            (Gf.Quatd(0.760406, 0, 0, -0.649448), Gf.Vec3d(0.125, 0.0, 0.25)),
            (Gf.Quatd(0.649448, 0, 0, -0.760406), Gf.Vec3d(0.125, 0.0, 0.25)),
            (Gf.Quatd(0.760406, 0, 0, 0.649448), Gf.Vec3d(0.125, 1.50, 0.25)),
            (Gf.Quatd(0.649448, 0, 0, 0.760406), Gf.Vec3d(0.125, 1.50, 0.25)),
        ]
        return emitter_poses

    # Test to make sure that command can create emitter and array without any errors
    # Simulate and stop to make sure it doesn't crash
    # Check to see if data returned matches parameters used to create
    async def test_command(self):

        result, emitter = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0],
        )
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0)],
            receiver_modes=[(0, 0)],
        )
        result, group_2 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 1)],
            receiver_modes=[(0, 1)],
        )
        horizontal_fov = 30.0
        vertical_fov = 5.0
        horizontal_res = 0.3
        vertical_res = 0.1
        num_bins = 300
        max_range = 3.45
        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path="/World/UltrasonicArray",
            min_range=0.4,
            max_range=max_range,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=horizontal_fov,
            vertical_fov=vertical_fov,
            horizontal_resolution=horizontal_res,
            vertical_resolution=vertical_res,
            num_bins=num_bins,
            emitter_prims=[emitter.GetPath()],
            firing_group_prims=[group_1.GetPath(), group_2.GetPath()],
        )
        self.assertTrue(result)
        self._timeline.play()
        await simulate_async(1.5)
        self.assertEqual(self._ultrasonic.get_num_rows("/World/UltrasonicArray"), int(vertical_fov / vertical_res))
        self.assertEqual(self._ultrasonic.get_num_cols("/World/UltrasonicArray"), int(horizontal_fov / horizontal_res))
        self.assertEqual(self._ultrasonic.get_num_emitters("/World/UltrasonicArray"), 1)
        self.assertEqual(len(self._ultrasonic.get_envelope("/World/UltrasonicArray", 0)), num_bins)
        # There are no obstacles so depth should all be the same
        depth = self._ultrasonic.get_linear_depth_data("/World/UltrasonicArray", 0)
        self.assertAlmostEquals(np.min(depth), max_range, delta=0.001)
        self.assertAlmostEquals(np.max(depth), max_range, delta=0.001)
        self._timeline.stop()
        await simulate_async(0.1)
        self._timeline.play()
        await simulate_async(0.5)

    # TODO: test scenario where you have specified emitter modes and receiver modes but no adjacency list --
    # it will return a single 2d list, the inner of length numBins where all elements are zeros. This should
    # instead result in a carbon error

    # Create two emitters, test to make sure that data from them is correct
    async def test_active_envelope_interface_two_emitters(self):

        result, emitter0 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter0",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0, 1],
        )
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0), (1, 0), (1, 1)],
            receiver_modes=[(0, 0), (0, 1), (1, 0), (1, 1)],
        )

        emitter0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))
        # Rotate 90 degrees about z
        emitter0.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(0, 0, 90))

        result, emitter1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter1",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0, 1],
        )
        emitter1.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path="/World/UltrasonicArray",
            min_range=0.4,
            max_range=3.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=20.0,
            vertical_fov=10.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            emitter_prims=[emitter0.GetPath(), emitter1.GetPath()],
            firing_group_prims=[group_1.GetPath()],
        )
        self.assertTrue(result)

        await self.add_cube("/World/Cube0", 0.250, Gf.Vec3f(0.0, 1.000, 0.0), physics=False)
        await self.add_cube("/World/Cube2", 0.250, Gf.Vec3f(0.800, 0.0, 0.0), physics=False)

        self._timeline.play()
        await simulate_async(2.0)
        # TODO test to make sure that the sensor is firing at correct times
        # TODO test to make sure that distances are correct
        active_env = self._ultrasonic.get_active_envelope_array("/World/UltrasonicArray")
        active_env = np.array(active_env)
        self.assertEqual(len(active_env), 4)
        self.assertTrue(
            np.allclose(
                active_env[0][50:67],
                np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 460.0, 32.0]),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[1][50:67],
                np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[2][50:67],
                np.array([498.0, 102.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[3][50:67],
                np.array([498.0, 102.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
        )

    # Create two emitters, test to make sure that data from them is correct
    async def test_two_emitter(self):

        result, emitter0 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter0",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[],
        )
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0)],
            receiver_modes=[(0, 0)],
        )
        result, group_2 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(1, 1)],
            receiver_modes=[(1, 1)],
        )
        emitter0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))
        # Rotate 90 degrees about z
        emitter0.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(0, 0, 90))

        result, emitter1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter1",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[],
        )
        emitter1.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path="/World/UltrasonicArray",
            min_range=0.4,
            max_range=2.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=20.0,
            vertical_fov=10.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            emitter_prims=[emitter0.GetPath(), emitter1.GetPath()],
            firing_group_prims=[group_1.GetPath(), group_2.GetPath()],
        )
        self.assertTrue(result)

        await self.add_cube("/World/Cube0", 0.250, Gf.Vec3f(0.0, 1.000, 0.0), physics=False)
        await self.add_cube("/World/Cube1", 0.250, Gf.Vec3f(0.0, -0.900, 0.0), physics=False)
        await self.add_cube("/World/Cube2", 0.250, Gf.Vec3f(0.800, 0.0, 0.0), physics=False)
        await self.add_cube("/World/Cube3", 0.250, Gf.Vec3f(-0.700, 0.0, 0.0), physics=False)

        self._timeline.play()
        await simulate_async(2.0)
        # TODO test to make sure that the sensor is firing at correct times
        # TODO test to make sure that distances are correct
        depth = self._ultrasonic.get_linear_depth_data("/World/UltrasonicArray", 0)

    # TODO: Add test that makes emitter on dynamic object
    # TODO: Add test that changes a parameter on the array and the emitter to make sure USD updates are working

    # TODO: re-work the existing tests to make sure that they work
    # # ensures that envelope changes when cube is moved
    async def test_static_ultrasonic_moving_box(self):
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Add a cube
        cubePath = "/World/Cube"
        cubePrim = await self.add_cube(cubePath, 0.250, Gf.Vec3f(0.0, -0.900, 0.125), physics=False)

        emitter_poses = self.get_full_array_poses()
        emitters = []
        for pose in emitter_poses:
            result, emitter_prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicEmitter",
                path="/World/UltrasonicEmitter",
                per_ray_intensity=0.4,
                yaw_offset=0.0,
                adjacency_list=[],
            )
            emitter_prim.GetPrim().GetAttribute("xformOp:translate").Set(pose[1])
            emitter_prim.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
                Gf.Rotation(pose[0]).Decompose((1, 0, 0), (0, 1, 0), (0, 0, 1))
            )
            emitters.append(emitter_prim)
        emitter_paths = [emitter.GetPath() for emitter in emitters]

        # Add ultrasonic
        ultrasonicPath = "/World/UltrasonicArray"
        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=ultrasonicPath,
            min_range=0.4,
            max_range=2.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=20.0,
            vertical_fov=10.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[],
        )

        # run for 12s @ 50Hz
        steps_per_sec = 50
        seconds = 3
        self._timeline.play()

        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr = self._ultrasonic.get_envelope_array(ultrasonicPath)
        cubePrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.200, 2.200, 0.125))
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr2 = self._ultrasonic.get_envelope_array(ultrasonicPath)
        envelope_diff = envelope_arr - envelope_arr2
        self.assertFalse(envelope_diff[0].any())
        self.assertTrue(envelope_diff[10].any())
        self.assertTrue(envelope_diff[11].any())

    # # ensures that envelope changes when cube is moved progressively further away from sensor
    async def test_move_box_to_multiple_distances(self):
        # Plane
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
        )

        # Add a cube
        cubePath = "/World/Cube"
        cubePrim = await self.add_cube(cubePath, 0.250, Gf.Vec3f(0.0, -0.900, 0.125), physics=False)

        emitter_poses = self.get_full_array_poses()

        emitters = []
        for pose in emitter_poses:
            result, emitter_prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicEmitter",
                path="/World/UltrasonicEmitter",
                per_ray_intensity=0.4,
                yaw_offset=0.0,
                adjacency_list=[],
            )
            emitter_prim.GetPrim().GetAttribute("xformOp:translate").Set(pose[1])
            emitter_prim.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
                Gf.Rotation(pose[0]).Decompose((1, 0, 0), (0, 1, 0), (0, 0, 1))
            )
            emitters.append(emitter_prim)
        emitter_paths = [emitter.GetPath() for emitter in emitters]

        # Add ultrasonic
        ultrasonicPath = "/World/UltrasonicArray"
        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=ultrasonicPath,
            min_range=0.4,
            max_range=2.0,
            draw_points=True,
            horizontal_fov=10.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[],
        )

        # run for 3s @ 50Hz
        steps_per_sec = 50
        seconds = 3
        self._timeline.play()
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr = self._ultrasonic.get_envelope_array(ultrasonicPath)
        self.assertTrue(np.allclose(envelope_arr[9][87:93], np.array([104.0, 137.0, 129.0, 59.0, 20.0, 1.0])))

        # move box then confirm that the envelopes have changed
        cubePrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, -1.200, 0.125))
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr2 = self._ultrasonic.get_envelope_array(ultrasonicPath)
        self.assertTrue(np.allclose(envelope_arr2[9][87:93], np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self.assertTrue(np.allclose(envelope_arr2[9][120:127], np.array([14.0, 84.0, 99.0, 92.0, 67.0, 19.0, 2.0])))

        # move box further and again confirm that the envelopes have changed
        cubePrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, -1.900, 0.125))
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr3 = self._ultrasonic.get_envelope_array(ultrasonicPath)
        self.assertTrue(np.allclose(envelope_arr3[9][120:127], np.array([0.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0])))
        self.assertTrue(np.allclose(envelope_arr3[9][199:203], np.array([23.0, 49.0, 24.0, 4.0])))

    def get_front_bumper_emitter_paths(self):
        poses = [
            (Gf.Quatd(0.951057, 0, 0, -0.309017), Gf.Vec3d(0.25, 0.0, 0.25)),
            (Gf.Quatd(0.760406, 0, 0, -0.649448), Gf.Vec3d(0.125, 0.0, 0.25)),
            (Gf.Quatd(0.649448, 0, 0, -0.760406), Gf.Vec3d(0.125, 0.0, 0.25)),
            (Gf.Quatd(-0.309017, 0, 0, 0.951056), Gf.Vec3d(-0.25, 0.0, 0.25)),
        ]
        emitter_prims = [None] * len(poses)
        cmd_name = "RangeSensorCreateUltrasonicEmitter"
        path = "/World/UltrasonicEmitter"
        res, emitter_prims[0] = omni.kit.commands.execute(
            cmd_name, path=path, per_ray_intensity=0.4, yaw_offset=0.0, adjacency_list=[0, 1]
        )
        res, emitter_prims[1] = omni.kit.commands.execute(
            cmd_name, path=path, per_ray_intensity=0.4, yaw_offset=0.0, adjacency_list=[0, 1, 2]
        )
        result, emitter_prims[2] = omni.kit.commands.execute(
            cmd_name, path=path, per_ray_intensity=0.4, yaw_offset=0.0, adjacency_list=[1, 2, 3]
        )
        result, emitter_prims[3] = omni.kit.commands.execute(
            cmd_name, path=path, per_ray_intensity=0.4, yaw_offset=0.0, adjacency_list=[2, 3]
        )

        emitter_paths = []
        for i in range(len(poses)):
            emitter_prims[i].GetPrim().GetAttribute("xformOp:translate").Set(poses[i][1])
            emitter_prims[i].GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
                Gf.Rotation(poses[i][0]).Decompose((1, 0, 0), (0, 1, 0), (0, 0, 1))
            )
            emitter_paths.append(emitter_prims[i].GetPath())
        return emitter_paths

    # check that indirects are working --> single emitter will show up in two receivers
    async def test_front_bumper_firing_single_group_hi_lo(self):
        emitter_paths = self.get_front_bumper_emitter_paths()

        # mode 0 is not receiving from itself on same freq so it gets no directs
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0)],  # , (1,1), (2,1), (3, 1)],
            receiver_modes=[(0, 0), (1, 0)],  # , (2, 0), (2,1), (3, 1)],
        )
        self.ultrasonicPath = "/World/UltrasonicArray"

        result, self.ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=self.ultrasonicPath,
            min_range=0.4,
            max_range=3.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=15.0,  # set wedge vertical extent in degrees
            vertical_fov=5.0,  # set wedge horizontal extent in degrees
            horizontal_resolution=0.5,
            vertical_resolution=0.5,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[group_1.GetPath()],
        )
        cubePrim = await self.add_cube("/World/Cube0", 0.750, Gf.Vec3f(0.950, -0.850, 0.0), physics=False)
        steps_per_sec = 50
        seconds = 3
        self._timeline.play()
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr = self._ultrasonic.get_envelope_array(self.ultrasonicPath)
        self.assertTrue(
            np.allclose(envelope_arr[0][51:61], np.array([10.0, 20.0, 20.0, 20.0, 20.0, 17.0, 13.0, 20.0, 10.0, 10.0]))
        )
        # self.assertTrue(
        #     np.allclose(envelope_arr[1][55:65], np.array([20.0, 20.0, 20.0, 20.0, 10.0, 20.0, 10.0, 20.0, 10.0, 15.0]))
        # )

    # test to ensure that if receiving on both low and high frequencies, the response should be higher
    async def test_front_bumper_combined_frequencies(self):
        emitter_paths = self.get_front_bumper_emitter_paths()

        # mode 0 is not receiving from itself on same freq so it gets no directs
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0), (0, 1)],
            receiver_modes=[(0, 0), (0, 1)],
        )
        self.ultrasonicPath = "/World/UltrasonicArray"

        result, self.ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=self.ultrasonicPath,
            min_range=0.4,
            max_range=3.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=15.0,  # set wedge vertical extent in degrees
            vertical_fov=5.0,  # set wedge horizontal extent in degrees
            horizontal_resolution=0.5,
            vertical_resolution=0.5,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[group_1.GetPath()],
        )
        cubePrim = await self.add_cube("/World/Cube0", 0.750, Gf.Vec3f(0.950, -0.850, 0.0), physics=False)
        steps_per_sec = 50
        seconds = 3
        self._timeline.play()
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr = self._ultrasonic.get_envelope_array(self.ultrasonicPath)
        self.assertTrue(
            np.allclose(envelope_arr[0][51:61], np.array([20.0, 40.0, 40.0, 40.0, 40.0, 34.0, 26.0, 40.0, 20.0, 20.0]))
        )

    # test to ensure that a single emitter will produce a response in three receivers, including its own
    async def test_front_bumper_three_receivers(self):
        emitter_paths = self.get_front_bumper_emitter_paths()

        # mode 0 is not receiving from itself on same freq so it gets no directs
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(1, 0)],
            receiver_modes=[(0, 0), (1, 0), (2, 0)],
        )
        self.ultrasonicPath = "/World/UltrasonicArray"

        result, self.ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=self.ultrasonicPath,
            min_range=0.4,
            max_range=3.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=15.0,  # set wedge vertical extent in degrees
            vertical_fov=5.0,  # set wedge horizontal extent in degrees
            horizontal_resolution=0.5,
            vertical_resolution=0.5,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[group_1.GetPath()],
        )
        cubePrim = await self.add_cube("/World/Cube0", 0.750, Gf.Vec3f(0.0, -0.850, 0.0), physics=False)
        steps_per_sec = 50
        seconds = 3
        self._timeline.play()
        await simulate_async(seconds, steps_per_sec=steps_per_sec)
        envelope_arr = self._ultrasonic.get_envelope_array(self.ultrasonicPath)
        # self.assertTrue(np.allclose(envelope_arr[0][35:37], np.array([249.0, 51.0])))
        self.assertTrue(np.allclose(envelope_arr[1][35:37], np.array([170.0, 130.0])))
        # self.assertTrue(np.allclose(envelope_arr[2][35:37], np.array([170.0, 130.0])))

    async def test_firing_modes(self):
        result, group_0 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup_0",
            emitter_modes=[(0, 1), (3, 0), (4, 1), (7, 0), (8, 1), (11, 0)],
            receiver_modes=[
                (0, 1),
                (1, 1),
                (2, 0),
                (3, 0),
                (3, 1),
                (4, 0),
                (4, 1),
                (5, 1),
                (6, 0),
                (7, 0),
                (7, 1),
                (8, 0),
                (8, 1),
                (9, 1),
                (10, 0),
                (11, 0),
            ],
        )

        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup_1",
            emitter_modes=[(1, 1), (2, 0), (5, 1), (6, 0), (9, 1), (10, 0)],
            receiver_modes=[
                (0, 1),
                (1, 0),
                (1, 1),
                (2, 0),
                (2, 1),
                (3, 0),
                (4, 1),
                (5, 1),
                (6, 0),
                (7, 0),
                (8, 1),
                (9, 0),
                (9, 1),
                (10, 0),
                (10, 1),
                (11, 0),
            ],
        )
        adjacency = [
            [0, 1],
            [0, 1, 2],
            [1, 2, 3],
            [2, 3, 4],
            [3, 4, 5],
            [4, 5],
            [6, 7],
            [6, 7, 8],
            [7, 8, 9],
            [8, 9, 10],
            [9, 10, 11],
            [10, 11],
        ]
        emitter_poses = self.get_full_array_poses()
        emitters = []
        for i in range(len(emitter_poses)):
            pose = emitter_poses[i]
            adjacent = adjacency[i]
            result, emitter_prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicEmitter",
                path="/World/UltrasonicEmitter",
                per_ray_intensity=0.4,
                yaw_offset=0.0,
                adjacency_list=adjacent,
            )
            emitter_prim.GetPrim().GetAttribute("xformOp:translate").Set(pose[1])
            emitter_prim.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(
                Gf.Rotation(pose[0]).Decompose((1, 0, 0), (0, 1, 0), (0, 0, 1))
            )
            emitters.append(emitter_prim)
        emitter_paths = [emitter.GetPath() for emitter in emitters]

        # Add ultrasonic
        ultrasonicPath = "/World/UltrasonicArray"
        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=ultrasonicPath,
            min_range=0.4,
            max_range=2.0,
            draw_points=True,
            horizontal_fov=10.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[group_0.GetPath(), group_1.GetPath()],
        )

        await self.add_cube("/World/Cube0", 0.250, Gf.Vec3f(0.0, 1.000, 0.0), physics=False)
        await self.add_cube("/World/Cube1", 0.250, Gf.Vec3f(0.0, -0.900, 0.0), physics=False)
        await self.add_cube("/World/Cube2", 0.250, Gf.Vec3f(0.800, 0.0, 0.0), physics=False)
        await self.add_cube("/World/Cube3", 0.250, Gf.Vec3f(-0.700, 0.0, 0.0), physics=False)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # await simulate_async(2.0)
        envelope_arr = self._ultrasonic.get_active_envelope_array(ultrasonicPath)
        # print(envelope_arr)
        print("Group A")
        await omni.kit.app.get_app().next_update_async()
        emitter_info = self._ultrasonic.get_emitter_firing_info(ultrasonicPath)
        print("emitter info:", emitter_info)

        receiver_info = self._ultrasonic.get_receiver_firing_info(ultrasonicPath)
        print("receiver info:", receiver_info)
        await omni.kit.app.get_app().next_update_async()
        print("Group B")
        emitter_info = self._ultrasonic.get_emitter_firing_info(ultrasonicPath)
        print("emitter info:", emitter_info)

        receiver_info = self._ultrasonic.get_receiver_firing_info(ultrasonicPath)
        print("receiver info:", receiver_info)

    # Create two emitters, test to make sure that data from them is correct when using cos(theta)
    # to weight the
    async def test_active_envelope_interface_two_emitters_with_brdf(self):

        result, emitter0 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter0",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0, 1],
        )
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0), (1, 0), (1, 1)],
            receiver_modes=[(0, 0), (0, 1), (1, 0), (1, 1)],
        )

        emitter0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))
        # Rotate 90 degrees about z
        emitter0.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(0, 0, 90))

        result, emitter1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter1",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0, 1],
        )
        emitter1.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path="/World/UltrasonicArray",
            min_range=0.4,
            max_range=3.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=20.0,
            vertical_fov=10.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            use_brdf=True,
            emitter_prims=[emitter0.GetPath(), emitter1.GetPath()],
            firing_group_prims=[group_1.GetPath()],
        )
        self.assertTrue(result)

        await self.add_cube("/World/Cube0", 0.250, Gf.Vec3f(0.0, 1.000, 0.0), physics=False)
        await self.add_cube("/World/Cube2", 0.250, Gf.Vec3f(0.800, 0.0, 0.0), physics=False)

        self._timeline.play()
        await simulate_async(2.0)
        # TODO test to make sure that the sensor is firing at correct times
        # TODO test to make sure that distances are correct
        active_env = self._ultrasonic.get_active_envelope_array("/World/UltrasonicArray")
        active_env = np.array(active_env)
        self.assertEqual(len(active_env), 4)
        self.assertTrue(
            np.allclose(
                active_env[0][50:67],
                np.array(
                    [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        452.37457275,
                        30.56451797,
                    ]
                ),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[1][50:67],
                np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[2][50:67],
                np.array(
                    [
                        488.72018433,
                        96.2740097,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                ),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[3][50:67],
                np.array(
                    [
                        488.72018433,
                        96.2740097,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                ),
            )
        )

    # Test USS Materials with BRDF
    async def test_uss_brdf_materials(self):

        result, emitter0 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter0",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0, 1],
        )
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup",
            emitter_modes=[(0, 0), (1, 0), (1, 1)],
            receiver_modes=[(0, 0), (0, 1), (1, 0), (1, 1)],
        )

        emitter0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))
        # Rotate 90 degrees about z
        emitter0.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(0, 0, 90))

        result, emitter1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/World/UltrasonicEmitter1",
            per_ray_intensity=0.4,
            yaw_offset=0.0,
            adjacency_list=[0, 1],
        )
        emitter1.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path="/World/UltrasonicArray",
            min_range=0.4,
            max_range=3.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=20.0,
            vertical_fov=10.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.8,
            num_bins=224,
            use_brdf=True,
            use_uss_materials=True,
            emitter_prims=[emitter0.GetPath(), emitter1.GetPath()],
            firing_group_prims=[group_1.GetPath()],
        )
        self.assertTrue(result)

        cube0 = await self.add_cube("/World/Cube0", 0.250, Gf.Vec3f(0.0, 1.000, 0.0), physics=False)
        cube1 = await self.add_cube("/World/Cube2", 0.250, Gf.Vec3f(0.800, 0.0, 0.0), physics=False)

        uss_material_path = "/cube_uss_material"
        stage = omni.usd.get_context().get_stage()
        UsdShade.Material.Define(stage, uss_material_path)
        uss_material_prim = stage.GetPrimAtPath(uss_material_path)
        UsdPhysics.MaterialAPI.Apply(uss_material_prim)
        uss_schema = RangeSensorSchema.UltrasonicMaterialAPI.Apply(uss_material_prim)

        uss_schema.CreatePerceptualRoughnessAttr().Set(1.0)
        uss_schema.CreateReflectanceAttr().Set(1.0)
        uss_schema.CreateMetallicAttr().Set(1.0)

        def add_physics_material_to_prim(stage, prim, materialPath):
            bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
            materialPrim = UsdShade.Material(stage.GetPrimAtPath(materialPath))
            bindingAPI.Bind(materialPrim, UsdShade.Tokens.weakerThanDescendants, "physics")

        add_physics_material_to_prim(stage, cube0, uss_material_path)
        add_physics_material_to_prim(stage, cube1, uss_material_path)

        self._timeline.play()
        await simulate_async(2.0)
        # TODO test to make sure that the sensor is firing at correct times
        # TODO test to make sure that distances are correct
        active_env = self._ultrasonic.get_active_envelope_array("/World/UltrasonicArray")
        active_env = np.array(active_env)
        print(active_env[0][50:67])
        self.assertEqual(len(active_env), 4)
        self.assertTrue(
            np.allclose(
                active_env[0][50:67],
                np.array(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 73.51676941, 5.1510067]
                ),
            )
        )
        self.assertTrue(
            np.allclose(
                active_env[1][50:67],
                np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
        )
        print(active_env[2][50:67])
        self.assertTrue(
            np.allclose(
                active_env[2][50:67],
                np.array(
                    [
                        79.63156128,
                        16.46644402,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                ),
            )
        )
        print(active_env[3][50:67])
        self.assertTrue(
            np.allclose(
                active_env[3][50:67],
                np.array(
                    [
                        79.63156128,
                        16.46644402,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                ),
            )
        )
