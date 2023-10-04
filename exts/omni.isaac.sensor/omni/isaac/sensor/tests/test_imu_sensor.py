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
import omni.kit.commands

import carb.tokens
import asyncio
import math
import numpy as np
from pxr import Gf, UsdGeom
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.sensor import _sensor
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestIMUSensor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_rate = 60
        self._sensor_rate = 120
        self._is = _sensor.acquire_imu_sensor_interface()
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self.my_world = None
        pass

    async def createAnt(self):
        self.leg_paths = ["/Ant/Arm_{:02d}/Lower_Arm".format(i + 1) for i in range(4)]
        self.sphere_path = "/Ant/Sphere"
        self.sensor_offsets = [
            Gf.Vec3d(0, 0, 0),
            Gf.Vec3d(0, 0, 0),
            Gf.Vec3d(0, 0, 0),
            Gf.Vec3d(0, 0, 0),
            Gf.Vec3d(0, 0, 0),
        ]

        self.sensor_quatd = [
            Gf.Quatd(1, 0, 0, 0),
            Gf.Quatd(1, 0, 0, 0),
            Gf.Quatd(1, 0, 0, 0),
            Gf.Quatd(1, 0, 0, 0),
            Gf.Quatd(1, 0, 0, 0),
        ]

        self.shoulder_joints = ["/Ant/Arm_{:02d}/Upper_Arm/shoulder_joint".format(i + 1) for i in range(4)]

        self.lower_joints = ["{}/lower_arm_joint".format(i) for i in self.leg_paths]

        await omni.usd.get_context().open_stage_async(self._assets_root_path + "/Isaac/Robots/Simple/ant.usd")
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

        self.my_world = World(
            stage_units_in_meters=1.0, physics_dt=1.0 / self._physics_rate, rendering_dt=1.0 / self._physics_rate
        )
        await self.my_world.initialize_simulation_context_async()
        self.ant = XFormPrim("/Ant")
        pass

    async def createSimpleArticulation(self):

        self.pivot_path = "/Articulation/CenterPivot"
        self.slider_path = "/Articulation/Slider"
        self.arm_path = "/Articulation/Arm"

        # load nucleus asset
        await omni.usd.get_context().open_stage_async(
            self._assets_root_path + "/Isaac/Robots/Simple/simple_articulation.usd"
        )

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        self.my_world = World(
            stage_units_in_meters=1.0, physics_dt=1.0 / self._physics_rate, rendering_dt=1.0 / self._physics_rate
        )
        await self.my_world.initialize_simulation_context_async()

        pass

    # After running each test
    async def tearDown(self):
        if self.my_world:
            self.my_world.stop()
            self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_add_sensor_prim(self):
        await self.createAnt()
        self.sensorGeoms = []
        for i in range(4):
            await omni.kit.app.get_app().next_update_async()
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateImuSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                sensor_period=1 / self._sensor_rate,
                translation=self.sensor_offsets[i],
                orientation=self.sensor_quatd[i],
            )
            self.sensorGeoms.append(sensor)
            self.assertTrue(result)
            self.assertIsNotNone(sensor)
            # Add sensor on body sphere
            await omni.kit.app.get_app().next_update_async()
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateImuSensor",
                path="/sensor",
                parent=self.sphere_path,
                sensor_period=self._sensor_rate,
                translation=self.sensor_offsets[4],
                orientation=self.sensor_quatd[4],
                visualize=True,
            )
            self.assertTrue(result)
            self.sensorGeoms.append(sensor)
            self.assertIsNotNone(sensor)
        pass

    # notice the ways of reading data for get_sensor_readings
    # and get_sensor_sim_reading are very different
    async def test_get_sensor_readings(self):
        await self.test_add_sensor_prim()  # And is also created
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        for i in range(120):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_readings(self.leg_paths[0] + "/sensor")
            # this should be
            num_readings = self._sensor_rate / self._physics_rate
            # print(len(sensor_reading))
            self.assertTrue(abs(len(sensor_reading) - num_readings) <= 1)
            # the last is the newest
            sensor_reading = sensor_reading[-1]
            # print(sensor_reading["lin_acc_x"], "\t", sensor_reading["ang_vel_x"])
            self.assertIsNotNone(sensor_reading["lin_acc_x"])
            self.assertIsNotNone(sensor_reading["ang_vel_x"])
            self.assertIsNotNone(sensor_reading["orientation"])
        pass

    async def test_get_sensor_sim_reading(self):
        await self.test_add_sensor_prim()
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(self.leg_paths[0] + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.ang_vel_x)
            self.assertIsNotNone(sensor_reading.lin_acc_x)
            self.assertIsNotNone(sensor_reading.ang_vel_x)
            self.assertIsNotNone(sensor_reading.orientation)
        pass

    async def test_orientation_imu(self):
        await self.createSimpleArticulation()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/arm_imu",
            parent=self.arm_path,
            sensor_period=self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        self.my_world.play()

        await omni.kit.app.get_app().next_update_async()

        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        slider_body = self._dc.find_articulation_body(art, "Arm")
        await omni.kit.app.get_app().next_update_async()
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)

        # set both dof state and targets for position
        for i in range(num_dofs):
            props[i]["stiffness"] = 1e8
            props[i]["damping"] = 1e8

        self._dc.set_articulation_dof_properties(art, props)

        ang = 0
        for i in range(70):
            new_state = [math.radians(ang), 0.5]

            state["pos"] = new_state
            self._dc.set_articulation_dof_states(art, state, _dynamic_control.STATE_POS)
            self._dc.set_articulation_dof_position_targets(art, new_state)
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            orientation = quat_to_euler_angles(
                np.array(self._is.get_sensor_readings(self.arm_path + "/arm_imu")[-1]["orientation"]), True
            )[0]

            angtest = ang % 360
            if ang >= 180:
                angtest = ang - 360

            self.assertAlmostEqual(orientation, angtest, delta=1e-1)
            ang += 5

        pass

    async def test_ang_vel_imu(self):
        await self.createSimpleArticulation()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/slider_imu",
            parent=self.slider_path,
            sensor_period=self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        self.my_world.play()

        await omni.kit.app.get_app().next_update_async()

        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        slider_body = self._dc.find_articulation_body(art, "Slider")
        dof_ptr = self._dc.find_articulation_dof(art, "RevoluteJoint")
        await omni.kit.app.get_app().next_update_async()
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)

        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)

        self._dc.set_articulation_dof_properties(art, props)

        ang_vel_l = [x * 30 for x in range(0, 20)]
        for x in ang_vel_l:

            new_state = [math.radians(x), 0]
            state["pos"] = new_state

            self._dc.set_articulation_dof_states(art, state, _dynamic_control.STATE_VEL)
            self._dc.set_articulation_dof_velocity_targets(art, new_state)

            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            ang_vel_z = self._is.get_sensor_readings(self.slider_path + "/slider_imu")[-1]["ang_vel_z"]

            # reset state before next test
            self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
            self._dc.set_dof_position_target(dof_ptr, 0)

            self.assertAlmostEqual(ang_vel_z, math.radians(x), delta=2.0e-2)

        pass

    async def test_lin_acc_imu(self):
        await self.createSimpleArticulation()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/slider_imu",
            parent=self.slider_path,
            sensor_period=self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        # await self.test_add_arm_imu()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/arm_imu",
            parent=self.arm_path,
            sensor_period=self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        self.my_world.play()

        await omni.kit.app.get_app().next_update_async()

        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        slider_body = self._dc.find_articulation_body(art, "Slider")
        dof_ptr = self._dc.find_articulation_dof(art, "RevoluteJoint")
        await omni.kit.app.get_app().next_update_async()
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)

        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)

        # set all joints to effort mode
        for i in range(num_dofs):
            props["stiffness"][i] = 0
            props["damping"][i] = 0

        self._dc.set_articulation_dof_properties(art, props)

        x = 0
        for i in range(60):

            new_state = [math.radians(x), 0]
            state["effort"] = new_state

            self._dc.set_articulation_dof_states(art, state, _dynamic_control.STATE_EFFORT)
            self._dc.set_articulation_dof_efforts(art, new_state)

            await omni.kit.app.get_app().next_update_async()
            slider_mag = np.linalg.norm(
                [
                    self._is.get_sensor_readings(self.slider_path + "/slider_imu")[-1]["lin_acc_x"],
                    self._is.get_sensor_readings(self.slider_path + "/slider_imu")[-1]["lin_acc_y"],
                ]
            )
            arm_mag = np.linalg.norm(
                [
                    self._is.get_sensor_readings(self.arm_path + "/arm_imu")[-1]["lin_acc_x"],
                    self._is.get_sensor_readings(self.arm_path + "/arm_imu")[-1]["lin_acc_y"],
                ]
            )

            self.assertGreaterEqual(slider_mag, arm_mag)

            x += 1000

        pass

    async def test_gravity_m(self):
        await self.test_add_sensor_prim()
        self.ant.set_world_pose([0, 0, 1])
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 0, delta=0.1)
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 9.81, delta=0.1)
        pass

    async def test_gravity_moon_m(self):
        await self.test_add_sensor_prim()
        self.ant.set_world_pose([0, 0, 1])
        self.my_world.get_physics_context().set_gravity(-1.62)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 0, delta=0.1)
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 1.62, delta=0.1)
        pass

    async def test_gravity_cm(self):
        await self.createAnt()
        await self.test_add_sensor_prim()

        UsdGeom.SetStageMetersPerUnit(self._stage, 0.01)

        await omni.kit.app.get_app().next_update_async()

        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 981, delta=0.1)

    pass

    async def test_stop_start(self):
        await self.test_add_sensor_prim()

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        first = True
        for i in range(200):
            await omni.kit.app.get_app().next_update_async()

            sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")
            if first:
                init_reading = sensor_reading
                first = False

            body_handle = self._dc.get_rigid_body(self.sphere_path)
            self._dc.apply_body_force(body_handle, (10, 10, 10), (0, 0, 0), True)

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()
        sensor_reading = self._is.get_sensor_sim_reading(self.sphere_path + "/sensor")

        self.assertEqual(sensor_reading.lin_acc_x, init_reading.lin_acc_x)
        self.assertEqual(sensor_reading.lin_acc_y, init_reading.lin_acc_y)
        self.assertEqual(sensor_reading.lin_acc_z, init_reading.lin_acc_z)

    pass

    async def test_no_physics_scene(self):
        await omni.usd.get_context().open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
        )
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        cube_path = "/new_cube"
        DynamicCuboid(prim_path=cube_path, name="cube_1", position=np.array([0, 0, 2]), color=np.array([255, 0, 0]))

        await omni.kit.app.get_app().next_update_async()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor", path="/sensor", parent=cube_path, visualize=True
        )

        await omni.kit.app.get_app().next_update_async()

        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(cube_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 0, delta=0.1)
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_sim_reading(cube_path + "/sensor")
            # print(sensor_reading.lin_acc_x, "\t", sensor_reading.lin_acc_y, "\t", sensor_reading.lin_acc_z)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 9.81, delta=0.1)
        omni.timeline.get_timeline_interface().stop()
        pass
