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
import omni
import omni.kit.commands
import carb.tokens
import asyncio
import numpy as np
import omni.graph.core as og
import omni.graph.action
import sys

from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Usd, PhysicsSchemaTools, Sdf

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.sensor import _sensor
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.isaac.IsaacSensorSchema as sensorSchema
from omni.isaac.core.utils.prims import add_reference_to_stage
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core import World

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test


async def add_cube(stage, path, size, offset, physics=True, mass=0.0) -> Usd.Prim:
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_prim = stage.GetPrimAtPath(path)
    cube_geom.CreateSizeAttr(size)
    cube_geom.AddTranslateOp().Set(offset)
    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        await omni.kit.app.get_app().next_update_async()
        rigid_api.CreateRigidBodyEnabledAttr(True)
        await omni.kit.app.get_app().next_update_async()
        if mass > 0:
            mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
            await omni.kit.app.get_app().next_update_async()
            mass_api.CreateMassAttr(mass)
            await omni.kit.app.get_app().next_update_async()
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    await omni.kit.app.get_app().next_update_async()
    return cube_prim


class TestContactSensor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60

        self._cs = _sensor.acquire_contact_sensor_interface()

        self.leg_paths = ["/Ant/Arm_{:02d}/Lower_Arm".format(i + 1) for i in range(4)]

        # sensor offset from the center of the leg, have to consider parent scaling.
        self.sensor_offsets = [Gf.Vec3d(40, 0, 0), Gf.Vec3d(40, 0, 0), Gf.Vec3d(40, 0, 0), Gf.Vec3d(40, 0, 0)]

        # colors for the sensor visualization (r,g,b,a)
        self.color = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]

        self.shoulder_joints = ["/Ant/Arm_{:02d}/Upper_Arm/shoulder_joint".format(i + 1) for i in range(4)]

        self.lower_joints = ["{}/lower_arm_joint".format(i) for i in self.leg_paths]
        self._sensor_handles = [0 for i in range(4)]

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.usd.get_context().open_stage_async(self._assets_root_path + "/Isaac/Robots/Simple/ant.usd")

        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        self.my_world = World(
            stage_units_in_meters=1.0, physics_dt=1.0 / self._physics_rate, rendering_dt=1.0 / self._physics_rate
        )
        await self.my_world.initialize_simulation_context_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        self.my_world.stop()
        self.my_world.clear_instance()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_add_sensor_prim(self):
        self.sensorGeoms = []
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=-1,
                translation=self.sensor_offsets[i],
                visualize=True,
            )
            self.sensorGeoms.append(sensor)
            self.assertTrue(result)
            self.assertIsNotNone(sensor)
        pass

    # test plan:
    # move the ground to -10, simulate 10 steps, test for no contact
    # move the ground to -0.78, simulate 60 steps, test for contact
    # test raw contact value, z-normal ~ 1.0
    # move teh ground to -15, simulate 30 steps, test for no contact
    async def test_lost_contacts(self):
        await self.test_add_sensor_prim()
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -10))

        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()
        await simulate_async(0.1)  # simulate 0.1 s
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 0)

        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -0.78))
        await simulate_async(1)  # simulate 60 steps, ant should touch ground
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 1)

        c = contacts_raw[0]
        body0 = self._cs.decode_body_name(c["body0"])
        body1 = self._cs.decode_body_name(c["body1"])
        if self.leg_paths[0] not in [body0, body1]:
            self.fail("Raw contact does not contain queried body {} ({},{})".format(self.leg_paths[0], body0, body1))
        self.assertAlmostEqual(1.0, c["normal"]["z"], delta=6)
        print(c)

        # move the ground to lose the contacts
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -15))
        await simulate_async(0.5)
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 0)

    async def test_get_body_raw_data(self):

        delete_prim("/Ant")
        block_0_prim = add_reference_to_stage(
            prim_path="/World/block_0", usd_path=self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"
        )
        block_0 = RigidPrim(
            prim_path="/World/block_0/Cube", name="block_0", position=np.array([10, 0, 5.0]), scale=np.ones(3) * 1.0
        )
        PhysxSchema.PhysxContactReportAPI.Apply(block_0.prim)

        block_1_prim = add_reference_to_stage(
            prim_path="/World/block_1", usd_path=self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"
        )
        block_1 = RigidPrim(
            prim_path="/World/block_1/Cube", name="block_1", position=np.array([10, 0, 10.0]), scale=np.ones(3) * 1.0
        )
        PhysxSchema.PhysxContactReportAPI.Apply(block_1.prim)

        def block_1_is_contacting_block_0():
            raw_data = self._cs.get_rigid_body_raw_data(block_1.prim_path)
            in_contact = False
            for c in raw_data:
                if block_0.prim_path in {self._cs.decode_body_name(c["body0"]), self._cs.decode_body_name(c["body1"])}:
                    in_contact = True
                    break

            return in_contact

        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()

        count = 0

        while not block_1_is_contacting_block_0() and count < 500:
            count += 1
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(count < 500)

    async def test_get_raw_data(self):
        await self.test_add_sensor_prim()
        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()
        await simulate_async(1)  # simulate 60 steps, ant should touch ground
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 1)

        c = contacts_raw[0]
        body0 = self._cs.decode_body_name(c["body0"])
        body1 = self._cs.decode_body_name(c["body1"])
        if self.leg_paths[0] not in [body0, body1]:
            self.fail("Raw contact does not contain queried body {} ({},{})".format(self.leg_paths[0], body0, body1))
        self.assertAlmostEqual(1.0, c["normal"]["z"], delta=6)
        print(c)

    async def test_persistent_raw_data(self):
        await self.test_add_sensor_prim()
        self.my_world.play()
        await simulate_async(2.0)  # simulate long enough that physx stops sending persistent contact raw data
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 1)

        c = contacts_raw[0]
        body0 = self._cs.decode_body_name(c["body0"])
        body1 = self._cs.decode_body_name(c["body1"])
        if self.leg_paths[0] not in [body0, body1]:
            self.fail("Raw contact does not contain queried body {} ({},{})".format(self.leg_paths[0], body0, body1))
        self.assertAlmostEqual(1.0, c["normal"]["z"], delta=6)
        print(c)

    async def test_get_sensor_readings(self):
        await self.test_add_sensor_prim()
        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()

        await simulate_async(1.0)
        for i in range(120):
            await omni.kit.app.get_app().next_update_async()
            contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
            sensor_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")
            self.assertTrue(len(sensor_reading) >= 1)
            sensor_reading = sensor_reading[0]
            if len((contacts_raw)):
                # there is a contact, compute force from impulse, compare to sensor reading
                force = (
                    np.linalg.norm(
                        [
                            contacts_raw[0]["impulse"]["x"],
                            contacts_raw[0]["impulse"]["y"],
                            contacts_raw[0]["impulse"]["z"],
                        ]
                    )
                    * 60.0
                )  # dt is 1/60
                self.assertAlmostEqual(force, sensor_reading["value"], 2)
            else:
                # No contact, reading should be zero
                self.assertEqual(sensor_reading["value"], 0)
        pass

    async def test_delayed_get_sensor_readings(self):
        await self.test_add_sensor_prim()
        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()
        await simulate_async(1.0)

        for i in range(120):
            await omni.kit.app.get_app().next_update_async()

        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        sensor_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(sensor_reading), 1)
        sensor_reading = sensor_reading[0]

        if len((contacts_raw)):
            # there is a contact, compute force from impulse, compare to sensor reading
            force = (
                np.linalg.norm(
                    [contacts_raw[0]["impulse"]["x"], contacts_raw[0]["impulse"]["y"], contacts_raw[0]["impulse"]["z"]]
                )
                * 60.0
            )  # dt is 1/60
            self.assertAlmostEqual(force, sensor_reading["value"], 2)
        else:
            # No contact, reading should be zero
            self.assertEqual(sensor_reading["value"], 0)
        pass

    # async def test_compare_sensor_force_to_mass(self):
    #     cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 0), physics=True, mass = 10)

    #     # create fully body sensor (radius -1)
    #     _, (result, sensor) =  omni.kit.commands.execute(
    #         "IsaacSensorCreateContactSensor",
    #         path="/sensor",
    #         parent="/cube",
    #         min_threshold=0,
    #         max_threshold=10000000,
    #         color=(1, 1, 1, 1),
    #         radius=-1,
    #         sensor_period=-1,
    #         visualize=True,
    #     )
    #     self.assertTrue(result)

    #     # need this sync to add the cube into the physics engine
    #     await omni.kit.app.get_app().next_update_async()
    #     self.my_world.play()
    #     await simulate_async(1.5)
    #     await omni.kit.app.get_app().next_update_async()

    #     sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #     self.assertEqual(len(sensor_reading), 1)
    #     sensor_reading = sensor_reading[0]
    #     self.assertAlmostEqual(sensor_reading["value"], mass * 9.81, 1)
    #     pass

    async def test_get_sensor_sim_reading(self):
        await self.test_add_sensor_prim()
        self.my_world.play()
        await simulate_async(1.0)
        for i in range(120):
            await omni.kit.app.get_app().next_update_async()
            contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
            sensor_reading = self._cs.get_sensor_sim_reading(self.leg_paths[0] + "/sensor")
            if len((contacts_raw)):
                # there is a contact, compute force from impulse, compare to sensor reading
                force = (
                    np.linalg.norm(
                        [
                            contacts_raw[0]["impulse"]["x"],
                            contacts_raw[0]["impulse"]["y"],
                            contacts_raw[0]["impulse"]["z"],
                        ]
                    )
                    * 60.0
                )  # dt is 1/60
                self.assertAlmostEqual(force, sensor_reading.value, 2)
            else:
                # No contact, reading should be zero
                self.assertEqual(sensor_reading.value, 0)
        pass

    async def test_contact_outside_range(self):
        self.sensorGeoms = []

        # create 4 sensors at the center of the leg
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=-1,
                translation=Gf.Vec3f(0, 0, 0),
                visualize=True,
            )
            self.sensorGeoms.append(sensor)
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()
        await simulate_async(1)

        for i in range(40):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")
            self.assertEqual(len(sensor_reading), 1)
            sensor_reading = sensor_reading[0]
            self.assertEqual(sensor_reading["value"], 0)
        pass

    async def test_sensor_period(self):
        # create four sensors that run at 120hz
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=1.0 / 120.0,
                translation=self.sensor_offsets[i],
                visualize=True,
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()
        # give it some time to reach the ground first
        await simulate_async(1.5)
        await omni.kit.app.get_app().next_update_async()
        readings = []

        for i in range(60):  # Simulate for one second
            await omni.kit.app.get_app().next_update_async()
            raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
            print(str(raw))
            sensor_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")
            print(str(sensor_reading))
            sensor_sim = self._cs.get_sensor_sim_reading(self.leg_paths[0] + "/sensor")
            print(str(sensor_sim.value))
            readings = readings + sensor_reading.tolist()

        # tolerance +-1 reading (119, 120, 121 will be accepted)
        print(len(readings))
        self.assertTrue(abs(len(readings) - 120) <= 1)
        pass

    async def test_stop_start(self):
        await self.test_add_sensor_prim()

        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        first = True
        for i in range(200):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._cs.get_sensor_sim_reading(self.leg_paths[0] + "/sensor")
            print(
                "sensor_reading: "
                + str(sensor_reading.inContact)
                + " "
                + str(sensor_reading.value)
                + " "
                + str(sensor_reading.time)
            )

            if first:
                init_reading = sensor_reading
                print(
                    "init_reading: "
                    + str(init_reading.inContact)
                    + " "
                    + str(init_reading.value)
                    + " "
                    + str(init_reading.time)
                )
                first = False

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        sensor_reading = self._cs.get_sensor_sim_reading(self.leg_paths[0] + "/sensor")

        print(
            "sensor_reading: "
            + str(sensor_reading.inContact)
            + " "
            + str(sensor_reading.value)
            + " "
            + str(sensor_reading.time)
        )

        self.assertEqual(init_reading.inContact, sensor_reading.inContact)
        self.assertEqual(init_reading.value, sensor_reading.value)
        self.assertEqual(init_reading.time, sensor_reading.time)

    pass

    # number of readings aggregated from node is same as number output from sensor

    # if time permits, add currently in contact with functionality to contact sensor node

    async def test_node_outputs_reset(self):
        # add a single contact sensor
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/sensor",
            parent=self.leg_paths[0],
            min_threshold=0,
            max_threshold=10000000,
            color=self.color[0],
            radius=0.12,
            sensor_period=-1,
            translation=self.sensor_offsets[0],
            visualize=True,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()

        keys = og.Controller.Keys
        (graph, (tick_node, test_node), _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadContactSensor", "omni.isaac.sensor.IsaacReadContactSensor"),
                ],
                keys.CONNECT: [("OnPlaybackTick.outputs:tick", "IsaacReadContactSensor.inputs:execIn")],
            },
        )
        set_target_prims(
            primPath="/controller_graph/IsaacReadContactSensor",
            inputName="inputs:csPrim",
            targetPrimPaths=[self.leg_paths[0] + "/sensor"],
        )

        await omni.kit.app.get_app().next_update_async()

        out_in_contact = og.Controller.attribute("outputs:inContact", test_node)
        out_value = og.Controller.attribute("outputs:value", test_node)

        first = True
        for i in range(5):
            self.my_world.play()

            await omni.kit.app.get_app().next_update_async()

            curr_in_contact = og.DataView.get(out_in_contact)
            curr_value = og.DataView.get(out_value)

            await omni.kit.app.get_app().next_update_async()

            print(curr_in_contact)
            print(curr_value)

            if first:
                init_in_contact = curr_in_contact
                init_value = curr_value

            await simulate_async(1.0)

            self.my_world.stop()
            await omni.kit.app.get_app().next_update_async()

            if not first:
                self.assertEqual(init_in_contact, curr_in_contact)
                self.assertEqual(init_value, curr_value)
            first = False

        self.my_world.stop()

        pass

    async def test_node_nonzero_outputs(self):
        # add a single contact sensor
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/sensor",
            parent=self.leg_paths[0],
            min_threshold=0,
            max_threshold=10000000,
            color=self.color[0],
            radius=0.12,
            sensor_period=-1,
            translation=self.sensor_offsets[0],
            visualize=True,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()

        keys = og.Controller.Keys
        (graph, (tick_node, test_node), _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadContactSensor", "omni.isaac.sensor.IsaacReadContactSensor"),
                ],
                keys.CONNECT: [("OnPlaybackTick.outputs:tick", "IsaacReadContactSensor.inputs:execIn")],
            },
        )
        set_target_prims(
            primPath="/controller_graph/IsaacReadContactSensor",
            inputName="inputs:csPrim",
            targetPrimPaths=[self.leg_paths[0] + "/sensor"],
        )

        await omni.kit.app.get_app().next_update_async()

        out_in_contact = og.Controller.attribute("outputs:inContact", test_node)
        out_value = og.Controller.attribute("outputs:value", test_node)

        self.my_world.play()

        await simulate_async(1.0)

        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            print(og.DataView.get(out_in_contact))
            print(og.DataView.get(out_value))
            self.assertNotEqual(og.DataView.get(out_in_contact), 0)
            self.assertNotEqual(og.DataView.get(out_value), 0)

        self.my_world.stop()

        pass

    # working:
    async def test_ant_not_touching_restart(self):
        await self.test_add_sensor_prim()

        cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 10), physics=True, mass=10)

        # need this sync to add the cube into the physics engine
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            pre_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            post_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.assertEqual(pre_reading["inContact"], 0)
        self.assertEqual(post_reading["inContact"], 0)

        pass

    # async def test_cubes_not_touching_restart(self):
    #     # TODO: not working on windows:
    #     if sys.platform == "win32":
    #         return
    #     print("before cube add")

    #     cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 10), physics=True, mass=10)
    #     cube_prim2 = await add_cube(self._stage, "/cube2", 1, (5, 2, 10), physics=True, mass=10)

    #     print("before contact sensor create")
    #     await omni.kit.app.get_app().next_update_async()
    #     # create fully body sensor (radius -1)
    #     result, sensor = omni.kit.commands.execute(
    #         "IsaacSensorCreateContactSensor",
    #         path="/sensor",
    #         parent="/cube",
    #         min_threshold=0,
    #         max_threshold=10000000,
    #         color=(1, 1, 1, 1),
    #         radius=-1,
    #         sensor_period=-1,
    #         visualize=False,
    #     )
    #     self.assertTrue(result)
    #     self.assertIsNotNone(sensor)

    #     print("before refresh")

    #     # need this sync to add the cube into the physics engine
    #     await omni.kit.app.get_app().next_update_async()

    #     print("before play")
    #     self.my_world.play()

    #     print("before loop")
    #     for i in range(30):  # Simulate for one second
    #         await omni.kit.app.get_app().next_update_async()
    #         print("before reading")
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     self.my_world.stop()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     self.my_world.play()
    #     print("TIMELINE RESTARTED")

    #     for i in range(30):  # Simulate for one second
    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     pass

    async def test_ant_not_touching_then_touching_restart(self):
        await self.test_add_sensor_prim()

        cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 10), physics=True, mass=10)

        # need this sync to add the cube into the physics engine
        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            pre_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()

        cube_prim3 = await add_cube(self._stage, "/cube3", 1, (0, 0, 2), physics=True, mass=10)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        for i in range(30):
            await omni.kit.app.get_app().next_update_async()
            post_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.assertEqual(pre_reading["inContact"], 0)
        self.assertEqual(post_reading["inContact"], 1)
        self.my_world.stop()

        pass

    # async def test_cubes_not_touching_then_touching_restart(self):
    #     # TODO: not working on windows:
    #     if sys.platform == "win32":
    #         return
    #     cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 10), physics=True, mass=10)
    #     cube_prim2 = await add_cube(self._stage, "/cube2", 1, (5, 2, 10), physics=True, mass=10)

    #     # create fully body sensor (radius -1)
    #     result, sensor = omni.kit.commands.execute(
    #         "IsaacSensorCreateContactSensor",
    #         path="/sensor",
    #         parent="/cube",
    #         min_threshold=0,
    #         max_threshold=10000000,
    #         color=(1, 1, 1, 1),
    #         radius=-1,
    #         sensor_period=-1,
    #         visualize=True,
    #     )
    #     self.assertTrue(result)
    #     self.assertIsNotNone(sensor)

    #     # need this sync to add the cube into the physics engine
    #     await omni.kit.app.get_app().next_update_async()
    #     self.my_world.play()

    #     for i in range(30):  # Simulate for one second
    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     self.my_world.stop()
    #     await omni.kit.app.get_app().next_update_async()

    #     cube_prim3 = await add_cube(self._stage, "/cube3", 1, (2, 3, 0), physics=True, mass=10)
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()

    #     self.my_world.play()
    #     print("TIMELINE RESTARTED")
    #     for i in range(30):  # Simulate for one second

    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     pass

    async def test_ant_touching_restart(self):
        await self.test_add_sensor_prim()
        cube_prim = await add_cube(self._stage, "/cube", 1, (0, 0, 1), physics=True, mass=10)

        # need this sync to add the cube into the physics engine
        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()

        for i in range(30):
            await omni.kit.app.get_app().next_update_async()
            pre_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.my_world.play()

        first = True
        for i in range(30):
            await omni.kit.app.get_app().next_update_async()
            if first:
                self.assertEqual(self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]["inContact"], False)
            post_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]
            first = False

        self.assertEqual(pre_reading["inContact"], 1)
        self.assertEqual(post_reading["inContact"], 1)

        pass

    # async def test_cubes_touching_restart(self):
    #     # TODO: not working on windows:
    #     if sys.platform == "win32":
    #         return
    #     cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 0), physics=True, mass=10)
    #     cube_prim2 = await add_cube(self._stage, "/cube2", 1, (2, 3, 0), physics=True, mass=10)
    #     await omni.kit.app.get_app().next_update_async()
    #     # create fully body sensor (radius -1)
    #     result, sensor = omni.kit.commands.execute(
    #         "IsaacSensorCreateContactSensor",
    #         path="/sensor",
    #         parent="/cube",
    #         min_threshold=0,
    #         max_threshold=10000000,
    #         color=(1, 1, 1, 1),
    #         radius=-1,
    #         sensor_period=-1,
    #         visualize=False,
    #     )
    #     self.assertTrue(result)
    #     self.assertIsNotNone(sensor)

    #     # need this sync to add the cube into the physics engine
    #     await omni.kit.app.get_app().next_update_async()
    #     self.my_world.play()

    #     for i in range(30):  # Simulate for one second
    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     self.my_world.stop()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     self.my_world.play()
    #     print("TIMELINE RESTARTED")
    #     for i in range(30):  # Simulate for one second

    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     pass

    async def test_ant_touching_then_not_touching_restart(self):
        await self.test_add_sensor_prim()
        cube_prim = await add_cube(self._stage, "/cube", 1, (0, 0, 1), physics=True, mass=10)

        # need this sync to add the cube into the physics engine
        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        for i in range(30):
            await omni.kit.app.get_app().next_update_async()
            pre_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()

        delete_prim("/cube")

        await omni.kit.app.get_app().next_update_async()

        self.my_world.play()

        first = True
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            if first:
                self.assertEqual(self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]["inContact"], False)
                first = False

            post_reading = self._cs.get_sensor_readings(self.leg_paths[0] + "/sensor")[-1]

        self.assertEqual(pre_reading["inContact"], 1)
        self.assertEqual(post_reading["inContact"], 0)

        pass

    # async def test_cubes_touching_then_not_touching_restart(self):
    #     # TODO: not working on windows:
    #     if sys.platform == "win32":
    #         return
    #     cube_prim = await add_cube(self._stage, "/cube", 1, (2, 2, 0), physics=True, mass=10)
    #     cube_prim2 = await add_cube(self._stage, "/cube2", 1, (2, 3, 0), physics=True, mass=10)

    #     await omni.kit.app.get_app().next_update_async()

    #     # create fully body sensor (radius -1)
    #     result, sensor = omni.kit.commands.execute(
    #         "IsaacSensorCreateContactSensor",
    #         path="/sensor",
    #         parent="/cube",
    #         min_threshold=0,
    #         max_threshold=10000000,
    #         color=(1, 1, 1, 1),
    #         radius=-1,
    #         sensor_period=-1,
    #         visualize=False,
    #     )

    #     self.assertTrue(result)
    #     self.assertIsNotNone(sensor)

    #     # need this sync to add the cube into the physics engine
    #     await omni.kit.app.get_app().next_update_async()
    #     print("next update fail")

    #     self.my_world.play()
    #     print("timeline play fail")

    #     for i in range(30):  # Simulate for one second
    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")

    #         print("sensor reading: " + str(sensor_reading))
    #         # sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #         # print("sensor sim: " + str(sensor_sim))

    #     self.my_world.stop()
    #     await omni.kit.app.get_app().next_update_async()

    #     delete_prim("/cube2")

    #     await omni.kit.app.get_app().next_update_async()

    #     self.my_world.play()
    #     print("TIMELINE RESTARTED")
    #     for i in range(30):  # Simulate for one second

    #         await omni.kit.app.get_app().next_update_async()
    #         sensor_reading = self._cs.get_sensor_readings("/cube/sensor")
    #         print("sensor reading: " + str(sensor_reading))
    #     #           sensor_sim = self._cs.get_sensor_sim_reading("/cube/sensor")
    #     #            print("sensor sim: " + str(sensor_sim))

    #     pass
