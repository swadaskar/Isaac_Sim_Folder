# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni.kit.test
from pxr import Gf, UsdPhysics, Sdf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils
from omni.isaac.dynamic_control import conversions as dc_conversions
import numpy as np
import asyncio


class TestRigidBody(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.dynamic_control")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

        dc_utils.set_physics_frequency(60)
        self._physics_scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))
        dc_utils.set_scene_physics_type(gpu=False, scene_path="/physicsScene")

        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_pose(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")
        new_pose = _dynamic_control.Transform((1.00, 0, 0), (0, 0, 0, 1))
        self._dc.set_rigid_body_pose(handle, new_pose)
        await dc_utils.simulate(1.0)
        pos = self._dc.get_rigid_body_pose(handle).p
        self.assertAlmostEqual(pos.x, 1.00, delta=0.1)

    async def test_linear_velocity(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")
        self._dc.set_rigid_body_linear_velocity(handle, (1.00, 0, 0))
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_linear_velocity(handle)
        self.assertAlmostEqual(vel.x, 1.00, delta=0.1)

    async def test_angular_velocity(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")
        self._dc.set_rigid_body_angular_velocity(handle, (5, 0, 0))
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        # cube slows down due to angular damping
        self.assertAlmostEqual(vel.x, 4.75, delta=0.1)

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_gravity(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")

        pos = self._dc.get_rigid_body_pose(handle).p

        self._dc.set_rigid_body_disable_gravity(handle, True)
        self._dc.wake_up_rigid_body(handle)
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))
        await dc_utils.simulate(1.0)
        pos = self._dc.get_rigid_body_pose(handle).p

        self.assertAlmostEqual(pos.z, 0.999, delta=0.1)

        self._dc.set_rigid_body_disable_gravity(handle, False)
        self._dc.wake_up_rigid_body(handle)
        await dc_utils.simulate(1.0)
        pos = self._dc.get_rigid_body_pose(handle).p
        self.assertLess(pos.z, 0)

        pass

    async def test_rigid_body_properties(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")
        props = self._dc.get_rigid_body_properties(handle)
        self._dc.set_rigid_body_properties(handle, props)
        await dc_utils.simulate(1.0)
        # TODO: Test each property

    def call_all_rigid_body_apis(self, handle):
        self._dc.get_rigid_body_name(handle)
        self._dc.get_rigid_body_path(handle)
        self._dc.get_rigid_body_parent_joint(handle)
        self._dc.get_rigid_body_child_joint_count(handle)
        self._dc.get_rigid_body_child_joint(handle, 0)
        self._dc.get_rigid_body_child_joint(handle, 100)
        self._dc.get_rigid_body_pose(handle)
        self._dc.set_rigid_body_pose(handle, _dynamic_control.Transform())
        self._dc.set_rigid_body_disable_gravity(handle, True)
        self._dc.set_rigid_body_disable_simulation(handle, False)
        self._dc.get_rigid_body_linear_velocity(handle)
        self._dc.get_rigid_body_local_linear_velocity(handle)
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))
        self._dc.get_rigid_body_angular_velocity(handle)
        self._dc.set_rigid_body_angular_velocity(handle, (0, 0, 0))
        self._dc.apply_body_force(handle, (0, 0, 0), (0, 0, 0), True)
        self._dc.apply_body_force(handle, (0, 0, 0), (0, 0, 0), False)
        self._dc.get_relative_body_poses(handle, [handle])
        self._dc.get_rigid_body_properties(handle)
        self._dc.set_rigid_body_properties(handle, _dynamic_control.RigidBodyProperties())

    async def test_start_stop(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")
        self.call_all_rigid_body_apis(handle)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.call_all_rigid_body_apis(handle)

    # compare values from dc to usd to see if they match
    async def test_update_usd(self, gpu=False):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        prim = await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")

        rigid_prim = UsdPhysics.RigidBodyAPI(prim)
        await dc_utils.simulate(1.0)
        dc_pose = self._dc.get_rigid_body_pose(handle)
        usd_pose = dc_conversions.create_transform_from_mat(omni.usd.utils.get_world_transform_matrix(prim))

        self.assertTrue(
            np.allclose([dc_pose.p.x, dc_pose.p.y, dc_pose.p.z], [usd_pose.p.x, usd_pose.p.y, usd_pose.p.z], atol=1e-2)
        )

        dc_velocity = self._dc.get_rigid_body_linear_velocity(handle)
        usd_velocity = rigid_prim.GetVelocityAttr().Get()

        self.assertTrue(np.allclose([dc_velocity.x, dc_velocity.y, dc_velocity.z], usd_velocity, atol=1e-2))

        rigid_prim.GetVelocityAttr().Set((0, 0, 0))
        await omni.kit.app.get_app().next_update_async()
        dc_velocity = self._dc.get_rigid_body_linear_velocity(handle)
        usd_velocity = rigid_prim.GetVelocityAttr().Get()
        self.assertTrue(np.allclose([dc_velocity.x, dc_velocity.y, dc_velocity.z], usd_velocity, atol=1e-2))

    async def test_physics_no_render(self):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        await dc_utils.add_cube(self._stage, "/cube", 1.00, (0, 0, 1.00))
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._physx_interface.start_simulation()
        self._physx_interface.force_load_physics_from_usd()
        handle = self._dc.get_rigid_body("/cube")
        self.assertNotEqual(handle, _dynamic_control.INVALID_HANDLE)
        self._dc.get_rigid_body_name(handle)
        self._dc.get_rigid_body_path(handle)
        self._dc.get_rigid_body_parent_joint(handle)
        self._dc.get_rigid_body_child_joint_count(handle)
        self._dc.get_rigid_body_child_joint(handle, 0)
        self._dc.get_rigid_body_child_joint(handle, 100)
        self._dc.get_rigid_body_pose(handle)
        self._dc.set_rigid_body_pose(handle, _dynamic_control.Transform())
        self._dc.set_rigid_body_disable_gravity(handle, True)
        self._dc.set_rigid_body_disable_simulation(handle, False)
        self._dc.get_rigid_body_linear_velocity(handle)
        self._dc.get_rigid_body_local_linear_velocity(handle)
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))
        self._dc.get_rigid_body_angular_velocity(handle)
        self._dc.set_rigid_body_angular_velocity(handle, (0, 0, 0))
        self._dc.apply_body_force(handle, (0, 0, 0), (0, 0, 0), True)
        self._dc.apply_body_force(handle, (0, 0, 0), (0, 0, 0), False)
        self._dc.get_relative_body_poses(handle, [handle])
        self._dc.get_rigid_body_properties(handle)
        self._dc.set_rigid_body_properties(handle, _dynamic_control.RigidBodyProperties())
        current_time = 0
        self._physx_interface.update_simulation(elapsedStep=1.0 / 60.0, currentTime=current_time)
        self._physx_interface.update_transformations(
            updateToFastCache=True, updateToUsd=True, updateVelocitiesToUsd=True, outputVelocitiesLocalSpace=False
        )

    async def test_apply_body_force(self):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        prim = await dc_utils.add_cube(self._stage, "/cube", 1.00, (2.00, 0, 1.00), True, 1)
        # make sure that motion is not damped
        physxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physxRigidBodyAPI.CreateLinearDampingAttr(0)
        physxRigidBodyAPI.CreateAngularDampingAttr(0)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")

        pos = self._dc.get_rigid_body_pose(handle).p

        self._dc.set_rigid_body_disable_gravity(handle, True)
        self._dc.wake_up_rigid_body(handle)
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))

        # rotate using local force
        self._dc.apply_body_force(handle, (0, 0, -1), (-2.00, 0, 0), False)
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        self.assertAlmostEqual(vel[1], -0.2, delta=0.001)
        # clear all motion
        await omni.kit.app.get_app().next_update_async()
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))
        self._dc.set_rigid_body_angular_velocity(handle, (0, 0, 0))
        new_pose = _dynamic_control.Transform((2.00, 0, 1.00), (0, 0, 0, 1))
        self._dc.set_rigid_body_pose(handle, new_pose)
        await omni.kit.app.get_app().next_update_async()
        # make sure that we stop moving
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        self.assertAlmostEqual(vel[1], 0.0, delta=0.001)
        await omni.kit.app.get_app().next_update_async()
        # rotate the opposite direction via global force
        self._dc.apply_body_force(handle, (0, 0, 1), (0, 0, 0), True)
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        self.assertAlmostEqual(vel[1], 0.2, delta=0.001)

    async def test_apply_body_torque(self):
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        prim = await dc_utils.add_cube(self._stage, "/cube", 1.00, (2.00, 0, 1.00), True, 1)
        # make sure that motion is not damped
        physxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physxRigidBodyAPI.CreateLinearDampingAttr(0)
        physxRigidBodyAPI.CreateAngularDampingAttr(0)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        handle = self._dc.get_rigid_body("/cube")

        pos = self._dc.get_rigid_body_pose(handle).p

        self._dc.set_rigid_body_disable_gravity(handle, True)
        self._dc.wake_up_rigid_body(handle)
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))

        # rotate using world torque
        self._dc.apply_body_torque(handle, (0, 0, -2.00), True)
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        self.assertAlmostEqual(vel[2], -0.2, delta=0.001)
        print(vel)
        # clear all motion
        await omni.kit.app.get_app().next_update_async()
        self._dc.set_rigid_body_linear_velocity(handle, (0, 0, 0))
        self._dc.set_rigid_body_angular_velocity(handle, (0, 0, 0))
        # flip the rigid body 180 around x so when we apply local torque we rotate the opposite
        new_pose = _dynamic_control.Transform((2.00, 0, 1.00), (1, 0, 0, 0))
        self._dc.set_rigid_body_pose(handle, new_pose)
        await omni.kit.app.get_app().next_update_async()
        # make sure that we stop moving
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        self.assertAlmostEqual(vel[1], 0.0, delta=0.001)
        await omni.kit.app.get_app().next_update_async()
        # shoudl rotate opposite
        self._dc.apply_body_torque(handle, (0, 0, -2.00), False)
        await dc_utils.simulate(1.0)
        vel = self._dc.get_rigid_body_angular_velocity(handle)
        self.assertAlmostEqual(vel[2], 0.2, delta=0.001)
        print(vel)
