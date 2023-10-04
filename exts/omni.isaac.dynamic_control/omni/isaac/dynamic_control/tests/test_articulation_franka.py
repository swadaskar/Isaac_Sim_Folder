# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import carb  # carb data types are used as return values, need this
import numpy as np
from pxr import Gf
import asyncio

from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils
from omni.isaac.dynamic_control import conversions as dc_conversions
from .common import get_assets_root_path

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestArticulationFranka(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        prim = self._stage.DefinePrim("/panda", "Xform")
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        prim.GetReferences().AddReference(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")

        dc_utils.set_physics_frequency(60)
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
        obj_type = self._dc.peek_object_type("/panda")
        self.assertEqual(obj_type, _dynamic_control.ObjectType.OBJECT_ARTICULATION)
        # Get handle to articulation and make sure its valud
        art = self._dc.get_articulation("/panda")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        # Use articulation handle to do something and make sure its works
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertTrue(dof_states is not None)
        # check basics for articulation
        num_joints = self._dc.get_articulation_joint_count(art)
        num_dofs = self._dc.get_articulation_dof_count(art)
        num_bodies = self._dc.get_articulation_body_count(art)
        self.assertEqual(num_joints, 11)
        self.assertEqual(num_dofs, 9)
        self.assertEqual(num_bodies, 12)
        # difference between joint and dof
        fixed_joint_ptr = self._dc.find_articulation_joint(art, "panda_hand_joint")
        fixed_dof_ptr = self._dc.find_articulation_dof(art, "panda_hand_joint")
        self.assertNotEqual(fixed_joint_ptr, _dynamic_control.INVALID_HANDLE)
        self.assertEqual(fixed_dof_ptr, _dynamic_control.INVALID_HANDLE)

        # get joint properties
        joint_type = self._dc.get_joint_type(fixed_joint_ptr)
        joint_dof_count = self._dc.get_joint_dof_count(fixed_joint_ptr)  # dof of the joint
        self.assertEqual(joint_type, _dynamic_control.JOINT_FIXED)
        self.assertEqual(joint_dof_count, 0)

        # get dof states
        dof_ptr = self._dc.find_articulation_dof(art, "panda_finger_joint1")
        dof_type = self._dc.get_dof_type(dof_ptr)
        self.assertEqual(dof_type, _dynamic_control.DOF_TRANSLATION)
        dof_state_v1 = self._dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
        # get all dof states for articulation
        dof_idx = self._dc.find_articulation_dof_index(art, "panda_finger_joint1")
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)

        self.assertTrue(dof_states is not None)
        dof_state_v2 = dof_states["pos"][dof_idx]
        # make sure they both match
        self.assertAlmostEqual(dof_state_v1.pos, dof_state_v2, msg=f"{dof_state_v1.pos} += {dof_state_v2}")
        pass

    async def test_teleport(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/panda")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        root_body = self._dc.get_articulation_root_body(art)
        hand_idx = self._dc.find_articulation_body_index(art, "panda_hand")

        # teleport joints to target pose
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        targets = self._dc.get_articulation_dof_position_targets(art)
        dof_states["pos"] = targets
        self._dc.set_articulation_dof_states(art, dof_states, _dynamic_control.STATE_POS)
        await omni.kit.app.get_app().next_update_async()
        body_states = self._dc.get_articulation_body_states(art, _dynamic_control.STATE_POS)

        expected_pos = body_states["pose"]["p"][hand_idx]
        self.assertTrue(
            np.allclose(
                [expected_pos[0], expected_pos[1], expected_pos[2]], [0.36756575, 0.00441444, 0.42769492], atol=1e-5
            ),
            f"[0.36756575, 0.00441444, 0.42769492] != {expected_pos}",
        )

        new_pose = dc_conversions.create_transform(Gf.Vec3d(0.10, 0.10, 0.10), Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))

        self._dc.set_rigid_body_pose(root_body, new_pose)
        await omni.kit.app.get_app().next_update_async()
        body_states = self._dc.get_articulation_body_states(art, _dynamic_control.STATE_POS)

        expected_pos = body_states["pose"]["p"][hand_idx]
        self.assertTrue(
            np.allclose(
                [expected_pos[0], expected_pos[1], expected_pos[2]], [0.09577966, 0.45144385, 0.4985129], atol=1e-5
            ),
            f"[0.09577966, 0.45144385, 0.4985129] != {expected_pos}",
        )

        pass

    async def test_teleport_target(self):
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/panda")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        # turn off gravity because velocity drives will act differently with different damping settings otherwise
        body_count = self._dc.get_articulation_body_count(art)
        for bodyIdx in range(body_count):
            body = self._dc.get_articulation_body(art, bodyIdx)
            self._dc.set_rigid_body_disable_gravity(body, True)

        franka_joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]

        # make change to dynamic_control params
        props = _dynamic_control.DofProperties()
        props.drive_mode = _dynamic_control.DRIVE_FORCE
        props.damping = 1e1
        props.stiffness = 0
        for joint in franka_joint_names:
            self._dc.set_dof_properties(self._dc.find_articulation_dof(art, joint), props)

        await omni.kit.app.get_app().next_update_async()

        # get states with efforts and velocity set to 0
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        pos_targets = self._dc.get_articulation_dof_position_targets(art)
        dof_states["pos"] = pos_targets
        self._dc.set_articulation_dof_states(art, dof_states, _dynamic_control.STATE_ALL)
        await omni.kit.app.get_app().next_update_async()

        # record position
        dof_states1 = np.array(self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL))

        # teleport again from a different position without changing any dc params
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        pos_targets = self._dc.get_articulation_dof_position_targets(art)
        dof_states["pos"] = pos_targets
        self._dc.set_articulation_dof_states(art, dof_states, _dynamic_control.STATE_ALL)
        await omni.kit.app.get_app().next_update_async()

        # record position
        dof_states2 = np.array(self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL))

        # make change to dynamic_control params
        props = _dynamic_control.DofProperties()
        props.drive_mode = _dynamic_control.DRIVE_FORCE
        props.damping = 1e7
        props.stiffness = 0
        for joint in franka_joint_names:
            self._dc.set_dof_properties(self._dc.find_articulation_dof(art, joint), props)

        await omni.kit.app.get_app().next_update_async()

        # teleport again
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        pos_targets = self._dc.get_articulation_dof_position_targets(art)
        dof_states["pos"] = pos_targets
        self._dc.set_articulation_dof_states(art, dof_states, _dynamic_control.STATE_ALL)
        await omni.kit.app.get_app().next_update_async()

        dof_states3 = np.array(self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL))

        # print("dof_states1:\n", dof_states1)
        # print("dof_states2:\n", dof_states2)
        # print("dof_states3:\n", dof_states3)
        for i in range(len(dof_states1)):
            for j in range(3):
                self.assertAlmostEqual(
                    dof_states1[i][j], dof_states2[i][j], delta=1e-4, msg=f"{dof_states1[i][j]} != {dof_states2[i][j]}"
                )
                self.assertAlmostEqual(
                    dof_states1[i][j], dof_states3[i][j], delta=1e-4, msg=f"{dof_states1[i][j]} != {dof_states3[i][j]}"
                )

        pass

    async def test_movement(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/panda")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        # in order for test to pass, self collisions must be disabled
        art_props = _dynamic_control.ArticulationProperties()
        art_props.solver_position_iteration_count = 32
        art_props.solver_velocity_iteration_count = 32
        art_props.enable_self_collisions = False
        self._dc.set_articulation_properties(art, art_props)
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        props = self._dc.get_articulation_dof_properties(art)
        num_dofs = self._dc.get_articulation_dof_count(art)

        # set all joints to velocity mode
        for i in range(num_dofs):
            props["stiffness"][i] = 0
            props["damping"][i] = 1e15
            dof_states["vel"][i] = -10.0
        self._dc.set_articulation_dof_properties(art, props)
        self._dc.set_articulation_dof_states(art, dof_states, _dynamic_control.STATE_VEL)
        self._dc.set_articulation_dof_velocity_targets(art, dof_states["vel"])
        await dc_utils.simulate(1.5, self._dc, art)
        # check that we are at the limits
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_POS)
        for i in range(num_dofs):
            self.assertAlmostEqual(
                dof_states["pos"][i],
                props["lower"][i],
                delta=1e-3,
                msg=f'{dof_states["pos"][i]} += {props["lower"][i]}',
            )
        pass

    async def test_position_franka(self, gpu=False):
        dc_utils.set_scene_physics_type(gpu)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/panda")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        dof_ptr_left = self._dc.find_articulation_dof(art, "panda_finger_joint1")
        dof_ptr_right = self._dc.find_articulation_dof(art, "panda_finger_joint2")

        # set new dof pos target
        new_pos_list = [0.02, 0.0, 0.04]
        for new_pos in new_pos_list:
            for dof_ptr in [dof_ptr_left, dof_ptr_right]:
                self.assertTrue(self._dc.set_dof_position_target(dof_ptr, new_pos))
            await dc_utils.simulate(2.0, self._dc, art)
            for t in [dof_ptr_left, dof_ptr_right]:
                self.assertAlmostEqual(
                    self._dc.get_dof_position(dof_ptr),
                    new_pos,
                    delta=0.01,
                    msg=f"{self._dc.get_dof_position(dof_ptr)} += {new_pos}",
                )
                self.assertAlmostEqual(
                    self._dc.get_dof_position_target(dof_ptr),
                    new_pos,
                    delta=0.01,
                    msg=f"{self._dc.get_dof_position_target(dof_ptr)} += {new_pos}",
                )

    # async def test_masses(self, gpu=False):
    #     dc_utils.set_scene_physics_type(gpu)
    #     self._physics_scene = UsdPhysics.Scene(self._stage.GetPrimAtPath("/physicsScene"))
    #     self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    #     self._physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
    #     # Start Simulation and wait
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     art = self._dc.get_articulation("/panda")
    #     self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

    #     dof_masses = self._dc.get_articulation_dof_masses(art)
    #     dof_props = self._dc.get_articulation_dof_properties(art)
    #     num_dofs = self._dc.get_articulation_dof_count(art)
    #     for i in range(num_dofs):
    #         print(dof_props[i], dof_masses[i])
    #         dof_props[i]["driveMode"] = _dynamic_control.DRIVE_FORCE
    #         dof_props[i]["damping"] = dof_props[i]["damping"] * dof_masses[i]
    #         dof_props[i]["stiffness"] = dof_props[i]["stiffness"] * dof_masses[i]
    #         print(dof_masses[i], dof_props[i]["damping"], dof_props[i]["stiffness"])
    #     self._dc.set_articulation_dof_properties(art, dof_props)
    #     await dc_utils.simulate(5.0)
    #     # TODO: Test each property

    async def test_physics_no_render(self):
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._physx_interface.start_simulation()
        self._physx_interface.force_load_physics_from_usd()
        prim = self._stage.DefinePrim("/panda", "Xform")
        prim.GetReferences().AddReference(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")
        self._physx_interface.force_load_physics_from_usd()
        art = self._dc.get_articulation("/panda")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        # do a zero time step, should not crash
        self._timeline.play()
        omni.physx.acquire_physx_interface().update_simulation(elapsedStep=0, currentTime=0)
        self._timeline.stop()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
