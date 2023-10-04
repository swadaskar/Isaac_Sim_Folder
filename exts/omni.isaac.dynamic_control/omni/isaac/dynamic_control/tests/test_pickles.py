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

import carb.tokens
import os
import asyncio
import numpy as np
import carb
import pickle

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.dynamic_control import _dynamic_control

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestPickles(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_pickle_float3(self):

        f3_src = carb.Float3(2.0, -1.5, 13.37)
        f3_bytes = pickle.dumps(f3_src)
        f3_dst = pickle.loads(f3_bytes)
        error_f3 = sum(abs(np.array(f3_src)) - abs(np.array(f3_dst)))
        self.assertAlmostEqual(error_f3, 0)

    async def test_pickle_float4(self):
        f4_src = carb.Float4(2.0, -1.5, 13.37, 42)
        f4_bytes = pickle.dumps(f4_src)
        f4_dst = pickle.loads(f4_bytes)
        error_f4 = sum(abs(np.array(f4_src)) - abs(np.array(f4_dst)))
        self.assertAlmostEqual(error_f4, 0)

    async def test_pickle_transform(self):
        tx_src = _dynamic_control.Transform((0.5, 1.25, -1.0), (0.1, 0.2, 0.3, 0.4))
        tx_bytes = pickle.dumps(tx_src)
        tx_dst = pickle.loads(tx_bytes)
        error_p = sum(abs(np.array(tx_src.p)) - abs(np.array(tx_dst.p)))
        error_r = sum(abs(np.array(tx_src.r)) - abs(np.array(tx_dst.r)))
        self.assertAlmostEqual(error_p, 0)
        self.assertAlmostEqual(error_r, 0)

    async def test_pickle_velocity(self):
        vel_src = _dynamic_control.Velocity((-1.1, -2.2, -3.3), (17, 42, 33))
        vel_bytes = pickle.dumps(vel_src)
        vel_dst = pickle.loads(vel_bytes)
        error_linear = sum(abs(np.array(vel_src.linear)) - abs(np.array(vel_dst.linear)))
        error_angular = sum(abs(np.array(vel_src.angular)) - abs(np.array(vel_dst.angular)))
        self.assertAlmostEqual(error_linear, 0)
        self.assertAlmostEqual(error_angular, 0)

    async def test_pickle_rigid_body_state(self):
        rbs_src = _dynamic_control.RigidBodyState()
        tx_src = _dynamic_control.Transform((0.5, 1.25, -1.0), (0.1, 0.2, 0.3, 0.4))
        vel_src = _dynamic_control.Velocity((-1.1, -2.2, -3.3), (17, 42, 33))
        rbs_src.pose = tx_src
        rbs_src.vel = vel_src
        rbs_bytes = pickle.dumps(rbs_src)
        rbs_dst = pickle.loads(rbs_bytes)
        error_pose_p = sum(abs(np.array(rbs_src.pose.p)) - abs(np.array(rbs_dst.pose.p)))
        error_pose_r = sum(abs(np.array(rbs_src.pose.r)) - abs(np.array(rbs_dst.pose.r)))
        error_vel_linear = sum(abs(np.array(rbs_src.vel.linear)) - abs(np.array(rbs_dst.vel.linear)))
        error_vel_angular = sum(abs(np.array(rbs_src.vel.angular)) - abs(np.array(rbs_dst.vel.angular)))
        self.assertAlmostEqual(error_pose_p, 0)
        self.assertAlmostEqual(error_pose_r, 0)
        self.assertAlmostEqual(error_vel_linear, 0)
        self.assertAlmostEqual(error_vel_angular, 0)

    async def test_pickle_dof_state(self):
        ds_src = _dynamic_control.DofState(2.0, -1.5, 5.5)
        ds_bytes = pickle.dumps(ds_src)
        ds_dst = pickle.loads(ds_bytes)
        error_pos = abs(np.array(ds_src.pos)) - abs(np.array(ds_dst.pos))
        error_vel = abs(np.array(ds_src.vel)) - abs(np.array(ds_dst.vel))
        error_effort = abs(np.array(ds_src.effort)) - abs(np.array(ds_dst.effort))
        self.assertAlmostEqual(error_pos, 0)
        self.assertAlmostEqual(error_vel, 0)
        self.assertAlmostEqual(error_effort, 0)

    async def test_pickle_dof_properties(self):
        dp_src = _dynamic_control.DofProperties()
        dp_src.type = _dynamic_control.DOF_ROTATION
        dp_src.has_limits = True
        dp_src.lower = -3.14
        dp_src.upper = 1.57
        dp_src.drive_mode = _dynamic_control.DRIVE_ACCELERATION
        dp_src.max_velocity = 123.4
        dp_src.max_effort = 1234.5
        dp_src.stiffness = 1e4
        dp_src.damping = 1e3
        dp_bytes = pickle.dumps(dp_src)
        dp_dst = pickle.loads(dp_bytes)
        self.assertEqual(dp_dst.type, dp_src.type)
        self.assertTrue(dp_dst.has_limits)
        self.assertAlmostEqual(dp_dst.lower, dp_src.lower)
        self.assertAlmostEqual(dp_dst.upper, dp_src.upper)
        self.assertEqual(dp_dst.drive_mode, dp_src.drive_mode)
        self.assertAlmostEqual(dp_dst.max_velocity, dp_src.max_velocity)
        self.assertAlmostEqual(dp_dst.max_effort, dp_src.max_effort)
        self.assertAlmostEqual(dp_dst.stiffness, dp_src.stiffness)
        self.assertAlmostEqual(dp_dst.damping, dp_src.damping)

    async def test_pickle_attractor_properties(self):
        ap_src = _dynamic_control.AttractorProperties()
        ap_src.body = 123456789
        ap_src.axes = _dynamic_control.AXIS_ALL
        ap_src.target.p = (-1, -2, -3)
        ap_src.target.r = (1, 2, 3, 4)
        ap_src.offset.p = (-0.1, -0.2, -0.3)
        ap_src.offset.r = (0.1, 0.2, 0.3, 0.4)
        ap_src.stiffness = 1e5
        ap_src.damping = 1e4
        ap_src.force_limit = 1e3
        ap_bytes = pickle.dumps(ap_src)
        ap_dst = pickle.loads(ap_bytes)
        self.assertEqual(ap_dst.body, ap_src.body)
        self.assertEqual(ap_dst.axes, ap_src.axes)

        error_target_p = sum(abs(np.array(ap_src.target.p)) - abs(np.array(ap_dst.target.p)))
        error_target_r = sum(abs(np.array(ap_src.target.r)) - abs(np.array(ap_dst.target.r)))
        error_offset_p = sum(abs(np.array(ap_src.offset.p)) - abs(np.array(ap_dst.offset.p)))
        error_offset_r = sum(abs(np.array(ap_src.offset.r)) - abs(np.array(ap_dst.offset.r)))
        self.assertAlmostEqual(error_target_p, 0)
        self.assertAlmostEqual(error_target_r, 0)
        self.assertAlmostEqual(error_offset_p, 0)
        self.assertAlmostEqual(error_offset_r, 0)
        self.assertAlmostEqual(ap_dst.stiffness, ap_src.stiffness)
        self.assertAlmostEqual(ap_dst.damping, ap_src.damping)
        self.assertAlmostEqual(ap_dst.force_limit, ap_src.force_limit)

    async def test_pickle_articulation_properties(self):
        ap_src = _dynamic_control.ArticulationProperties()
        ap_src.solver_position_iteration_count = 3
        ap_src.solver_velocity_iteration_count = 4
        ap_src.enable_self_collisions = True
        ap_bytes = pickle.dumps(ap_src)
        ap_dst = pickle.loads(ap_bytes)

        self.assertEqual(ap_dst.solver_position_iteration_count, ap_src.solver_position_iteration_count)
        self.assertEqual(ap_dst.solver_velocity_iteration_count, ap_src.solver_velocity_iteration_count)
        self.assertEqual(ap_dst.enable_self_collisions, ap_src.enable_self_collisions)

    async def test_pickle_rigid_body_properties(self):
        rb_src = _dynamic_control.RigidBodyProperties()
        rb_src.mass = 14.0
        rb_src.moment = carb.Float3(1.0, 2.0, 3.0)
        rb_src.max_depeneration_velocity = 2.0
        rb_src.max_contact_impulse = 3.0
        rb_src.solver_position_iteration_count = 4
        rb_src.solver_velocity_iteration_count = 5

        rb_bytes = pickle.dumps(rb_src)
        rb_dst = pickle.loads(rb_bytes)
        self.assertEqual(rb_dst.mass, rb_src.mass)
        error_moment = sum(abs(np.array(rb_dst.moment)) - abs(np.array(rb_src.moment)))
        self.assertAlmostEqual(error_moment, 0)
        self.assertEqual(rb_dst.max_depeneration_velocity, rb_src.max_depeneration_velocity)
        self.assertEqual(rb_dst.max_contact_impulse, rb_src.max_contact_impulse)
        self.assertEqual(rb_dst.solver_position_iteration_count, rb_src.solver_position_iteration_count)
        self.assertEqual(rb_dst.solver_velocity_iteration_count, rb_src.solver_velocity_iteration_count)
