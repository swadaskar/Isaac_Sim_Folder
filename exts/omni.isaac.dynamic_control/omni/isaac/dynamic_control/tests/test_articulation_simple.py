# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import math
import carb  # carb data types are used as return values, need this
import numpy as np
from pxr import Gf, UsdPhysics, Sdf
import omni.physx as _physx
import asyncio

from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils
from omni.isaac.dynamic_control import conversions as dc_conversions

# from omni.isaac.core.utils.nucleus import get_assets_root_path
from .common import get_assets_root_path
from .common import open_stage_async

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestArticulationSimple(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.dynamic_control")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._assets_root_path = get_assets_root_path()

        await omni.kit.app.get_app().next_update_async()

        # open remote
        self.usd_path = self._assets_root_path + "/Isaac/Robots/Simple/simple_articulation.usd"
        (result, error) = await open_stage_async(self.usd_path)

        await omni.kit.app.get_app().next_update_async()

        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()
        dc_utils.set_physics_frequency(60)  # set this after loading
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_load(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Check the object type to make sure its an articulation
        obj_type = self._dc.peek_object_type("/Articulation")
        self.assertEqual(obj_type, _dynamic_control.ObjectType.OBJECT_ARTICULATION)
        # Get handle to articulation and make sure its valid
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        # Use articulation handle to do something and make sure its works
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertTrue(dof_states is not None)
        pass

    async def test_non_sim(self, gpu=False):

        dc_utils.set_scene_physics_type(gpu)
        # Articulation should be invalid as sim has not started
        obj_type = self._dc.peek_object_type("/Articulation")
        self.assertEqual(obj_type, _dynamic_control.ObjectType.OBJECT_NONE)
        art = self._dc.get_articulation("/Articulation")
        self.assertEqual(art, _dynamic_control.INVALID_HANDLE)
        # force physics to load and some information should be valid
        self._physx_interface.force_load_physics_from_usd()
        obj_type = self._dc.peek_object_type("/Articulation")
        self.assertEqual(obj_type, _dynamic_control.ObjectType.OBJECT_ARTICULATION)
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        rb = self._dc.get_rigid_body("/Articulation/Arm")
        self.assertNotEqual(rb, _dynamic_control.INVALID_HANDLE)
        # Not Implemented yet
        # joint = self._dc.get_joint("/Articulation/Arm/RevoluteJoint")
        # self.assertNotEqual(joint, _dynamic_control.INVALID_HANDLE)
        # dof = self._dc.get_dof("/Articulation/Arm/RevoluteJoint")
        # self.assertNotEqual(joint, _dynamic_control.INVALID_HANDLE)
        self.assertTrue(
            self._dc.peek_object_type("/Articulation/Arm/RevoluteJoint"), _dynamic_control.ObjectType.OBJECT_JOINT
        )
        # Dof states will still be none
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertTrue(dof_states is None)
        dof_props = self._dc.get_articulation_dof_properties(art)
        self.assertTrue(dof_props is None)

    async def test_physics_manual(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        self._physx_interface.start_simulation()
        self._physx_interface.force_load_physics_from_usd()
        dt = 1.0 / 60.0
        # manually load and step physics
        self._physx_interface.update_simulation(dt, 0)
        art = self._dc.get_articulation("/Articulation")
        dof_ptr = self._dc.find_articulation_dof(art, "RevoluteJoint")
        slider_body = self._dc.find_articulation_body(art, "Slider")
        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)

        # drive in velocity mode for one second
        for i in range(num_dofs):
            props[i]["stiffness"] = 0
            props[i]["damping"] = 1e15

        self._dc.set_articulation_dof_properties(art, props)

        new_state = [math.radians(45), 0]
        self._dc.set_articulation_dof_velocity_targets(art, new_state)
        for frame in range(0, 60):
            self._physx_interface.update_simulation(dt, frame * dt)
            state = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
            # print(state)
        new_pose = self._dc.get_rigid_body_pose(slider_body)

        # after one second it should reach this pose
        self.assertAlmostEqual(state.pos, math.radians(45), delta=1e-4, msg=f"{state.pos} != {math.radians(45)}")
        self.assertAlmostEqual(state.vel, math.radians(45), delta=1e-4, msg=f"{state.vel} != {math.radians(45)}")
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [1.06778, 1.06781, 0], atol=1e-2), f"{new_pose.p}"
        )
        self._physx_interface.update_transformations(
            updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True, outputVelocitiesLocalSpace=False
        )
        self._physx_interface.reset_simulation()

    def call_all_articulation_apis(self, art, joint, dof):
        self._dc.wake_up_articulation(art)
        self._dc.get_articulation_name(art)
        self._dc.get_articulation_path(art)
        self._dc.get_articulation_body_count(art)
        self._dc.get_articulation_body(art, 0)
        self._dc.get_articulation_body(art, 100)
        self._dc.find_articulation_body(art, "Arm")
        self._dc.find_articulation_body(art, "DoesntExist")
        self._dc.get_articulation_root_body(art)
        self._dc.get_articulation_body_states(art, _dynamic_control.STATE_ALL)
        self._dc.get_articulation_properties(art)
        self._dc.set_articulation_properties(art, _dynamic_control.ArticulationProperties())
        self._dc.get_articulation_joint_count(art)
        self._dc.get_articulation_joint(art, 0)
        self._dc.get_articulation_joint(art, 100)
        self._dc.find_articulation_joint(art, "RevoluteJoint")
        self._dc.get_articulation_dof_count(art)
        self._dc.get_articulation_dof(art, 0)
        self._dc.get_articulation_dof(art, 100)
        self._dc.find_articulation_dof(art, "RevoluteJoint")
        self._dc.find_articulation_dof(art, "DoesntExist")
        self._dc.find_articulation_dof_index(art, "RevoluteJoint")
        self._dc.find_articulation_dof_index(art, "DoesntExist")
        self._dc.get_articulation_dof_properties(art)
        self._dc.set_articulation_dof_properties(art, [])
        self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self._dc.set_articulation_dof_states(art, [], _dynamic_control.STATE_ALL)
        self._dc.set_articulation_dof_position_targets(art, [])
        self._dc.get_articulation_dof_position_targets(art)
        self._dc.set_articulation_dof_velocity_targets(art, [])
        self._dc.get_articulation_dof_velocity_targets(art)
        self._dc.set_articulation_dof_efforts(art, [])
        self._dc.get_articulation_dof_efforts(art)
        self._dc.get_articulation_dof_masses(art)

        self._dc.get_joint_name(joint)
        self._dc.get_joint_path(joint)
        self._dc.get_joint_type(joint)
        self._dc.get_joint_dof_count(joint)
        self._dc.get_joint_dof(joint, 0)
        self._dc.get_joint_dof(joint, 100)
        self._dc.get_joint_parent_body(joint)
        self._dc.get_joint_child_body(joint)

        self._dc.get_dof_name(dof)
        self._dc.get_dof_path(dof)
        self._dc.get_dof_type(dof)
        self._dc.get_dof_joint(dof)
        self._dc.get_dof_parent_body(dof)
        self._dc.get_dof_child_body(dof)
        self._dc.get_dof_state(dof, _dynamic_control.STATE_ALL)
        self._dc.set_dof_state(dof, _dynamic_control.DofState(), _dynamic_control.STATE_ALL)
        self._dc.get_dof_position(dof)
        self._dc.set_dof_position(dof, 0)
        self._dc.get_dof_velocity(dof)
        self._dc.set_dof_velocity(dof, 0)
        self._dc.get_dof_properties(dof)
        self._dc.set_dof_properties(dof, _dynamic_control.DofProperties())
        self._dc.set_dof_position_target(dof, 0)
        self._dc.set_dof_velocity_target(dof, 0)
        self._dc.get_dof_position_target(dof)
        self._dc.get_dof_velocity_target(dof)
        self._dc.set_dof_effort(dof, 0)
        self._dc.get_dof_effort(dof)

    async def test_start_stop(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # get articulation handle
        art = self._dc.get_articulation("/Articulation")
        joint = self._dc.find_articulation_joint(art, "RevoluteJoint")
        dof = self._dc.find_articulation_dof(art, "RevoluteJoint")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        self.call_all_articulation_apis(art, joint, dof)
        # make sure handle is still valid after a stop/play
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.call_all_articulation_apis(art, joint, dof)
        # getting this while stopped should fail
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertTrue(dof_states is None)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertTrue(dof_states is not None)

    async def test_delete_joint(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        root_body = self._dc.get_articulation_root_body(art)
        new_pose = self._dc.get_rigid_body_pose(root_body)
        self.assertAlmostEqual(new_pose.p.z, 0, msg=f"new_pose.p.z = {new_pose.p.z}")

        # test to make sure articulation falls when joint is deleted.
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        omni.usd.commands.DeletePrimsCommand(["/Articulation/CenterPivot/FixedJoint"]).do()
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await dc_utils.simulate(0.1, self._dc, art)
        new_pose = self._dc.get_rigid_body_pose(root_body)
        self.assertAlmostEqual(new_pose.p.z, -0.076222, delta=0.02, msg=f"new_pose.p.z = {new_pose.p.z}")
        pass

    async def test_disable_joint(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        root_body = self._dc.get_articulation_root_body(art)
        new_pose = self._dc.get_rigid_body_pose(root_body)
        self.assertAlmostEqual(new_pose.p.z, 0, msg=f"new_pose.p.z = {new_pose.p.z}")

        # test to make sure articulation falls when joint is disabled.
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path("/Articulation/CenterPivot/FixedJoint.physics:jointEnabled"),
            value=False,
            prev=None,
        )

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await dc_utils.simulate(0.1, self._dc, art)
        new_pose = self._dc.get_rigid_body_pose(root_body)
        self.assertAlmostEqual(new_pose.p.z, -0.076222, delta=0.02, msg=f"new_pose.p.z = {new_pose.p.z}")
        pass

    async def test_root_transform(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        root_body = self._dc.get_articulation_root_body(art)
        pivot_body = self._dc.find_articulation_body(art, "CenterPivot")
        new_pose = dc_conversions.create_transform(Gf.Vec3d(0.100, 0.200, 0.030), Gf.Rotation(Gf.Vec3d(0, 0, 1), 0))
        self._dc.set_rigid_body_pose(root_body, new_pose)
        await omni.kit.app.get_app().next_update_async()
        arm_body = self._dc.find_articulation_body(art, "Arm")
        # Check the arm body pose
        self.assertEqual(
            self._dc.get_rigid_body_pose(arm_body), _dynamic_control.Transform((0.60, 0.20, 0.03), (0, 0, 0, 1))
        )
        # Move the body that corresponds to the root, should act the same as above

        new_pose = dc_conversions.create_transform(Gf.Vec3d(-0.100, 0.200, 0.030), Gf.Rotation(Gf.Vec3d(0, 0, 1), 0))
        self._dc.set_rigid_body_pose(pivot_body, new_pose)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(
            self._dc.get_rigid_body_pose(arm_body), _dynamic_control.Transform((0.40, 0.20, 0.03), (0, 0, 0, 1))
        )
        # Rotate the body in place by 45 degrees, x,y of pose should be the same
        new_pose = dc_conversions.create_transform(Gf.Vec3d(0, 0, 0), Gf.Rotation(Gf.Vec3d(0, 0, 1), 45))
        self._dc.set_rigid_body_pose(pivot_body, new_pose)
        await omni.kit.app.get_app().next_update_async()
        new_pose = self._dc.get_rigid_body_pose(arm_body)
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [0.3535535, 0.3535535, 0], atol=1e-5),
            f"{new_pose.p}",
        )

        ### This will fail as expected because its not a root link
        # body = self._dc.find_articulation_body(art, "Arm")
        # new_pose = dc_conversions.create_transform(Gf.Vec3d(10.0, 20.0, 3.0), Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
        # self._dc.set_rigid_body_pose(body, new_pose)
        pass

    async def test_root_velocity(self, gpu=False):

        dc_utils.set_scene_physics_type(gpu)
        self._physics_scene = UsdPhysics.Scene(self._stage.GetPrimAtPath("/physicsScene"))
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        delete_cmd = omni.usd.commands.DeletePrimsCommand(["/Articulation/CenterPivot/FixedJoint"])
        delete_cmd.do()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        root_body = self._dc.get_articulation_root_body(art)
        pivot_body = self._dc.find_articulation_body(art, "CenterPivot")
        self._dc.set_rigid_body_linear_velocity(root_body, (10, 0, 0))
        await dc_utils.simulate(0.1, self._dc, art)
        lin_vel = self._dc.get_rigid_body_linear_velocity(pivot_body)
        self.assertAlmostEqual(lin_vel.x, 10, delta=1e-3, msg=f"lin_vel.x = {lin_vel.x}")
        self._dc.set_rigid_body_linear_velocity(root_body, (0, 0, 0))
        await dc_utils.simulate(0.1, self._dc, art)
        lin_vel = self._dc.get_rigid_body_linear_velocity(pivot_body)
        self.assertAlmostEqual(lin_vel.x, 0, delta=1e-3, msg=f"lin_vel.x = {lin_vel.x}")
        self._dc.set_rigid_body_angular_velocity(root_body, (10, 0, 0))
        await dc_utils.simulate(0.1, self._dc, art)
        ang_vel = self._dc.get_rigid_body_angular_velocity(pivot_body)
        self.assertTrue(np.allclose([ang_vel.x, ang_vel.y, ang_vel.z], [10, 0, 0], atol=1e-5), f"{ang_vel}")
        pass

    async def test_get_articulation_dof_states(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        self._physics_scene = UsdPhysics.Scene(self._stage.GetPrimAtPath("/physicsScene"))
        # set gravity sideways to force articulation to have state
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 1.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # get articulation handle
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        await dc_utils.simulate(0.1, self._dc, art)
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_NONE)
        zeros = np.zeros(2, dtype=np.float32)
        self.assertTrue(np.array_equal(state["pos"], zeros), f'{state["pos"]}')
        self.assertTrue(np.array_equal(state["vel"], zeros), f'{state["vel"]}')
        self.assertTrue(np.array_equal(state["effort"], zeros), f'{state["effort"]}')

        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        self.assertFalse(np.array_equal(state["pos"], zeros), f'{state["pos"]}')
        self.assertTrue(np.array_equal(state["vel"], zeros), f'{state["vel"]}')
        self.assertTrue(np.array_equal(state["effort"], zeros), f'{state["effort"]}')

        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_VEL)
        self.assertTrue(np.array_equal(state["pos"], zeros), f'{state["pos"]}')
        self.assertFalse(np.array_equal(state["vel"], zeros), f'{state["vel"]}')
        self.assertTrue(np.array_equal(state["effort"], zeros), f'{state["effort"]}')

        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_EFFORT)
        self.assertTrue(np.array_equal(state["pos"], zeros), f'{state["pos"]}')
        self.assertTrue(np.array_equal(state["vel"], zeros), f'{state["vel"]}')
        self.assertFalse(np.array_equal(state["effort"], zeros), f'{state["effort"]}')

        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertFalse(np.array_equal(state["pos"], zeros), f'{state["pos"]}')
        self.assertFalse(np.array_equal(state["vel"], zeros), f'{state["vel"]}')
        self.assertFalse(np.array_equal(state["effort"], zeros), f'{state["effort"]}')

    async def test_set_articulation_dof_states(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # get articulation handle
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        slider_body = self._dc.find_articulation_body(art, "Slider")
        await omni.kit.app.get_app().next_update_async()
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)

        # set both dof state and targets for position
        for i in range(num_dofs):
            props[i]["stiffness"] = 1e8
            props[i]["damping"] = 1e8

        self._dc.set_articulation_dof_properties(art, props)
        # Rotate 45 degrees and set prismatic to 100
        new_state = [math.radians(45), 1.00]
        state["pos"] = new_state
        self._dc.set_articulation_dof_states(art, state, _dynamic_control.STATE_POS)
        self._dc.set_articulation_dof_position_targets(art, new_state)
        await omni.kit.app.get_app().next_update_async()
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        # check that the states match
        # print(new_state, state["pos"])
        self.assertTrue(np.allclose(new_state, state["pos"]), f'{new_state}, {state["pos"]}')
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [1.76777, 1.76777, 0], atol=1e-5), f"{new_pose.p}"
        )

        # velocity control test
        for i in range(num_dofs):
            props[i]["stiffness"] = 0
            props[i]["damping"] = 1e15

        self._dc.set_articulation_dof_properties(art, props)
        new_state = [0, -0.10]
        state["vel"] = new_state
        # set both state and target
        self._dc.set_articulation_dof_states(art, state, _dynamic_control.STATE_VEL)
        self._dc.set_articulation_dof_velocity_targets(art, new_state)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()  # need a second step before dof_states are updated
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        g_vel = self._dc.get_rigid_body_linear_velocity(slider_body)
        l_vel = self._dc.get_rigid_body_local_linear_velocity(slider_body)
        self.assertTrue(
            np.allclose([g_vel.x, g_vel.y, g_vel.z], [-0.0707107, -0.0707107, 0], atol=1e-3), f"g_vel {g_vel}"
        )
        self.assertTrue(np.allclose([l_vel.x, l_vel.y, l_vel.z], [-0.10, 0, 0], atol=1e-3), f"l_vel {l_vel}")
        self.assertTrue(np.allclose(new_state, state["vel"], atol=1e-3), f'new_state {new_state} ~= {state["vel"]}')

        # effort control for first joint, second joint is position drive
        props[0]["stiffness"] = 0
        props[0]["damping"] = 0
        props[1]["stiffness"] = 1e15
        props[1]["damping"] = 1e15

        self._dc.set_articulation_dof_properties(art, props)
        # reset state of articulation and apply effort
        state["pos"] = [0, 0]
        state["vel"] = [0, 0]
        state["effort"] = [1e1, 0]
        self._dc.set_articulation_dof_position_targets(art, [0, 0])
        self._dc.set_articulation_dof_velocity_targets(art, [0, 0])
        self._dc.set_articulation_dof_states(art, state, _dynamic_control.STATE_ALL)
        await dc_utils.simulate(1.0, self._dc, art)
        state = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_VEL)
        self.assertAlmostEqual(state["vel"][0], 3.5, delta=1e-2, msg=f'{state["vel"][0]}')

    async def test_get_gravity_effort(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        self._physics_scene = UsdPhysics.Scene(self._stage.GetPrimAtPath("/physicsScene"))
        gravity = -9.81
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 1.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(gravity)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        slider_body = self._dc.find_articulation_body(art, "Slider")
        dof_ptr = self._dc.find_articulation_dof(art, "RevoluteJoint")
        arm_body = self._dc.find_articulation_body(art, "Arm")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)
        for i in range(num_dofs):
            props[i]["driveMode"] = _dynamic_control.DRIVE_FORCE
            props[i]["stiffness"] = 1e10
            props[i]["damping"] = 1e10
            props[i]["maxEffort"] = 1e10
        self._dc.set_articulation_dof_properties(art, props)

        await omni.kit.app.get_app().next_update_async()
        await dc_utils.simulate(1.0, self._dc, art)
        # check both state apis
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_EFFORT)
        dof_state = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
        # compute torque analytically
        fg_slider = self._dc.get_rigid_body_properties(slider_body).mass * gravity
        fg_arm = self._dc.get_rigid_body_properties(arm_body).mass * gravity

        pose_slider = self._dc.get_rigid_body_pose(slider_body)
        pose_arm = self._dc.get_rigid_body_pose(arm_body)

        torque_0 = pose_arm.p.x * fg_arm + pose_slider.p.x * fg_slider
        self.assertAlmostEqual(
            -torque_0, dof_states["effort"][0], delta=6, msg=f'{-torque_0} != {dof_states["effort"][0]}'
        )
        self.assertAlmostEqual(-torque_0, dof_state.effort, delta=6, msg=f"{-torque_0} != {dof_state.effort}")

    async def test_dof_state(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        # get handles
        slider_body = self._dc.find_articulation_body(art, "Slider")
        dof_ptr = self._dc.find_articulation_dof(art, "RevoluteJoint")

        props = self._dc.get_dof_properties(dof_ptr)

        pos_target = math.radians(45)
        vel_target = math.radians(45)
        # configure for position control
        props.damping = 1e8
        props.stiffness = 1e8
        self._dc.set_dof_properties(dof_ptr, props)
        # use set_dof_state api
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(pos_target, 0, 0), _dynamic_control.STATE_ALL)
        self._dc.set_dof_position_target(dof_ptr, pos_target)
        await omni.kit.app.get_app().next_update_async()
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [1.06066, 1.06066, 0], atol=1e-5), f"{new_pose.p}"
        )
        # reset state before next test
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
        self._dc.set_dof_position_target(dof_ptr, 0)
        await omni.kit.app.get_app().next_update_async()

        # use set_dof_position api
        self._dc.set_dof_position(dof_ptr, pos_target)
        self._dc.set_dof_position_target(dof_ptr, pos_target)
        await omni.kit.app.get_app().next_update_async()
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [1.06066, 1.06066, 0], atol=1e-5), f"{new_pose.p}"
        )

        # reset state before next test
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
        self._dc.set_dof_position_target(dof_ptr, 0)
        await omni.kit.app.get_app().next_update_async()

        # velocity control
        props.damping = 1e15
        props.stiffness = 0
        self._dc.set_dof_properties(dof_ptr, props)
        # use set_dof_state api
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, vel_target, 0), _dynamic_control.STATE_ALL)
        self._dc.set_dof_velocity_target(dof_ptr, vel_target)
        await dc_utils.simulate(1.0, self._dc, art)
        state = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        # after one second it should reach this pose
        self.assertAlmostEqual(state.pos, pos_target, delta=1e-4, msg=f"{state.pos} != {pos_target}")
        self.assertAlmostEqual(state.vel, vel_target, delta=1e-4, msg=f"{state.vel} != {vel_target}")
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [1.0607, 1.0607, 0], atol=1e-2), f"{new_pose.p}"
        )
        # reset state before next test
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
        self._dc.set_dof_velocity_target(dof_ptr, 0)
        await omni.kit.app.get_app().next_update_async()
        # use set_dof_velocity api
        self._dc.set_dof_velocity(dof_ptr, vel_target)
        self._dc.set_dof_velocity_target(dof_ptr, vel_target)
        await dc_utils.simulate(1.0, self._dc, art)
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        # after one second it should reach this pose
        self.assertAlmostEqual(state.pos, pos_target, delta=1e-4, msg=f"{state.pos} != {pos_target}")
        self.assertAlmostEqual(state.vel, vel_target, delta=1e-4, msg=f"{state.vel} != {vel_target}")
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [1.0607, 1.0607, 0], atol=1e-2), f"{new_pose.p}"
        )
        # reset state before next test
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
        self._dc.set_dof_velocity_target(dof_ptr, 0)
        await omni.kit.app.get_app().next_update_async()

        # effort control
        props.damping = 0
        props.stiffness = 0
        self._dc.set_dof_properties(dof_ptr, props)
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 1e1), _dynamic_control.STATE_ALL)
        await dc_utils.simulate(1.0, self._dc, art)
        # use get_dof_state api
        state = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        self.assertAlmostEqual(state.pos, 1.8822, delta=1e-3, msg=f"state.pos = {state.pos}")
        self.assertAlmostEqual(state.vel, 3.6385, delta=1e-3, msg=f"state.vel = {state.vel}")
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [-0.46066, 1.4307, 2.34091e-05], atol=1e-2),
            f"{new_pose.p}",
        )
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
        await omni.kit.app.get_app().next_update_async()
        # use set_dof_effort api
        self._dc.set_dof_effort(dof_ptr, 1e1)
        self.assertEqual(self._dc.get_dof_effort(dof_ptr), 1e1)
        await dc_utils.simulate(1.0, self._dc, art)
        state = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        self.assertAlmostEqual(state.pos, 1.8822, delta=1e-3, msg=f"state.pos = {state.pos}")
        self.assertAlmostEqual(state.vel, 3.6385, delta=1e-3, msg=f"state.vel = {state.vel}")
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [-0.46066, 1.4307, 2.34091e-05], atol=1e-2),
            f"{new_pose.p}",
        )
        # reset state before next test
        self._dc.set_dof_state(dof_ptr, _dynamic_control.DofState(0, 0, 0), _dynamic_control.STATE_ALL)
        await omni.kit.app.get_app().next_update_async()
        # use set_articulation_dof_efforts api
        self._dc.set_articulation_dof_efforts(art, [1e1, 0])
        self.assertTrue(
            np.allclose(self._dc.get_articulation_dof_efforts(art), [1e1, 0]),
            f"{self._dc.get_articulation_dof_efforts(art)}",
        )
        await dc_utils.simulate(1.0, self._dc, art)
        state = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
        new_pose = self._dc.get_rigid_body_pose(slider_body)
        self.assertAlmostEqual(state.pos, 1.8822, delta=1e-3, msg=f"state.pos = {state.pos}")
        self.assertAlmostEqual(state.vel, 3.6385, delta=1e-3, msg=f"state.vel = {state.vel}")
        self.assertTrue(
            np.allclose([new_pose.p.x, new_pose.p.y, new_pose.p.z], [-0.46066, 1.4307, 2.34091e-05], atol=1e-2),
            f"new_pose.p = {new_pose.p}",
        )

    # async def test_get_effort(self, gpu=False):
    #     (result, error) = await open_stage_async(
    #         self._assets_root_path + "/Isaac/Robots/Simple/revolute_articulation.usd"
    #     )
    #     self.assertTrue(result)  # Make sure the stage loaded
    #     self._stage = omni.usd.get_context().get_stage()
    #     dc_utils.set_scene_physics_type(gpu)
    #     self._physics_scene = UsdPhysics.Scene(self._stage.GetPrimAtPath("/physicsScene"))
    #     gravity = 9.81
    #     self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    #     self._physics_scene.CreateGravityMagnitudeAttr().Set(gravity)

    #     # sensorAPI = PhysxSchema.PhysxArticulationForceSensorAPI.Apply(self._stage.GetPrimAtPath("/Articulation/Arm"))
    #     # sensorAPI.CreateConstraintSolverForcesEnabledAttr().Set(True)
    #     # sensorAPI.CreateForwardDynamicsForcesEnabledAttr().Set(True)

    #     sensorAPI = PhysxSchema.PhysxArticulationForceSensorAPI.Apply(
    #         self._stage.GetPrimAtPath("/Articulation/CenterPivot")
    #     )
    #     sensorAPI.CreateConstraintSolverForcesEnabledAttr().Set(True)
    #     sensorAPI.CreateForwardDynamicsForcesEnabledAttr().Set(True)

    #     await dc_utils.add_cube(self._stage, "/cube", 10, (90, 0, 20), True, 5)
    #     # Start Simulation and wait
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     art = self._dc.get_articulation("/Articulation")
    #     slider_body = self._dc.find_articulation_body(art, "Arm")
    #     dof_ptr = self._dc.find_articulation_dof(art, "RevoluteJoint")
    #     self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
    #     cube_handle = self._dc.get_rigid_body("/cube")
    #     cube_props = self._dc.get_rigid_body_properties(cube_handle)
    #     cube_props.solver_position_iteration_count = 32
    #     cube_props.solver_velocity_iteration_count = 32
    #     self._dc.set_rigid_body_properties(cube_handle, cube_props)

    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     props = self._dc.get_articulation_dof_properties(art)
    #     num_dofs = self._dc.get_articulation_dof_count(art)
    #     props[0]["driveMode"] = _dynamic_control.DRIVE_FORCE
    #     props[0]["maxEffort"] = 1e10

    #     props[0]["stiffness"] = 1e15
    #     props[0]["damping"] = 1e15

    #     self._dc.set_articulation_dof_properties(art, props)
    #     # change dof target: modifying current state
    #     dof_vel = [math.radians(45)]
    #     # self._dc.set_articulation_dof_velocity_targets(art, dof_vel)
    #     # await dc_utils.simulate(1.0, self._dc, art)
    #     for frame in range(60 * 1):
    #         if art is not None:
    #             self._dc.wake_up_articulation(art)
    #         await omni.kit.app.get_app().next_update_async()
    #         dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_EFFORT)
    #         # print(dof_states["effort"])
    #     # fg_slider = self._dc.get_rigid_body_properties(slider_body).mass * gravity
    #     arm_body = self._dc.find_articulation_body(art, "Arm")
    #     fg_arm = self._dc.get_rigid_body_properties(arm_body).mass * gravity

    #     # pose_slider = self._dc.get_rigid_body_pose(slider_body)
    #     pose_arm = self._dc.get_rigid_body_pose(arm_body)

    #     torque_0 = pose_arm.p.x * fg_arm
    #     print(torque_0)
    #     if cube_handle is not _dynamic_control.INVALID_HANDLE:
    #         pose_cube = self._dc.get_rigid_body_pose(cube_handle)
    #         fg_cube = self._dc.get_rigid_body_properties(cube_handle).mass * gravity
    #         torque_body = fg_cube * pose_cube.p.x
    #         print(torque_body)
    #         print(torque_0 + torque_body)
    #         # self.assertLess(dof_states[0][2], -1000)
    #         # print(dof_states[0][2])
    #     # dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_EFFORT)

    #     # print(dof_states["effort"])
