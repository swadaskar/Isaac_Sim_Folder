# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.kit.test
import omni.kit.commands
import omni.timeline
import omni.usd
import omni.physx
import omni.physics.tensors
import omni.graph.core as og
import omni.replicator.isaac as dr
from omni.replicator.isaac import physics_view as physics

from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import create_new_stage_async
from omni.isaac.core import World
from omni.isaac.core.prims import RigidPrimView

import numpy as np


class TestOgnWritePhysicsRigidPrimView(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await create_new_stage_async()
        self._my_world = World(backend="torch")

        await self._my_world.initialize_simulation_context_async()

        await omni.kit.app.get_app().next_update_async()
        self._my_world._physics_context.set_gravity(0)
        await omni.kit.app.get_app().next_update_async()

        self._stage = omni.usd.get_context().get_stage()
        self._controller = og.Controller()
        self._graph = self._controller.create_graph("/World/PushGraph")

        self._rigid_prim_view_node = self._controller.create_node(
            ("rigid_prim_view", self._graph), "omni.replicator.isaac.OgnWritePhysicsRigidPrimView"
        )
        self._distribution_node = self._controller.create_node(
            ("uniform", self._graph), "omni.replicator.core.OgnSampleUniform"
        )
        self._rigid_prim_view_node_prim = self._stage.GetPrimAtPath(self._rigid_prim_view_node.get_prim_path())

        self._iface = omni.timeline.get_timeline_interface()
        self._cube_path = "/World/Cube"
        self._cube = DynamicCuboid(prim_path=self._cube_path)

        await omni.kit.app.get_app().next_update_async()
        self._rb_view = RigidPrimView(prim_paths_expr="/World/Cube", name="cube")
        self._my_world.scene.add(self._rb_view)

        await self._my_world.reset_async()

        self._iface.play()
        dr.physics_view.register_rigid_prim_view(self._rb_view)
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        self._iface.stop()
        self._my_world.clear_instance()
        dr.physics_view._rigid_prim_views = dict()
        dr.physics_view._rigid_prim_views_initial_values = dict()
        await omni.usd.get_context().new_stage_async()

    async def _setup_random_attribute(self, attribute_name, value):
        self._distribution_node.get_attribute("inputs:numSamples").set(1)
        self._distribution_node.get_attribute("inputs:lower").set(value)
        self._distribution_node.get_attribute("inputs:upper").set(value)

        self._rigid_prim_view_node.get_attribute("inputs:prims").set("cube")
        self._rigid_prim_view_node.get_attribute("inputs:attribute").set(attribute_name)
        self._rigid_prim_view_node.get_attribute("inputs:indices").set([0])
        self._rigid_prim_view_node.get_attribute("inputs:operation").set("direct")

        n_elem = physics._rigid_prim_views_initial_values[self._rb_view.name][attribute_name].shape[-1]
        self._rigid_prim_view_node.get_attribute("inputs:values").set_resolved_type(
            og.Type(og.BaseDataType.FLOAT, n_elem, 1)
        )

        self._controller.connect(
            self._distribution_node.get_attribute("outputs:samples"),
            self._rigid_prim_view_node.get_attribute("inputs:values"),
        )
        await self._controller.evaluate(self._graph)

    async def test_randomize_position(self):
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="position", value=value)
        position, _ = self._rb_view.get_world_poses()
        position = position.clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(position, value)))

    async def test_randomize_orientation(self):
        value = [0, np.pi, 0]
        await self._setup_random_attribute(attribute_name="orientation", value=value)
        _, orientation = self._rb_view.get_world_poses()
        orientation = orientation.clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(orientation, [0, 0, 1, 0], atol=1e-04)))

    async def test_randomize_linear_velocity(self):
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="linear_velocity", value=value)
        linear_velocity = self._rb_view.get_linear_velocities().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(linear_velocity, value)))

    async def test_randomize_angular_velocity(self):
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="angular_velocity", value=value)
        angular_velocity = self._rb_view.get_angular_velocities().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(angular_velocity, value)))

    async def test_randomize_forces(self):
        value = [100, 100, 100]
        await self._setup_random_attribute(attribute_name="force", value=value)

    async def test_randomize_masses(self):
        if self._rb_view._device == "cpu":
            value = [100] * self._rb_view.count
            await self._setup_random_attribute(attribute_name="mass", value=value)
            new_value = self._rb_view.get_masses().clone().cpu().numpy()
            self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_inertias(self):
        if self._rb_view._device == "cpu":
            inertias = [0.1, 0.1, 0.1] * self._rb_view.count
            await self._setup_random_attribute(attribute_name="inertia", value=inertias)
            new_value = self._rb_view.get_inertias().clone().cpu().numpy()
            diagonal = new_value[:, [0, 4, 8]]
            self.assertTrue(np.all(np.isclose(diagonal.flatten(), inertias)))

    async def test_randomize_material_properties(self):
        value = [100] * self._rb_view.count * 3 * self._rb_view.num_shapes
        await self._setup_random_attribute(attribute_name="material_properties", value=value)
        new_value = self._rb_view._physics_view.get_material_properties().clone().cpu().numpy()
        print(value, new_value)
        self.assertTrue(np.all(np.isclose(new_value.flatten(), value)))

    async def test_randomize_contact_offsets(self):
        value = [100] * self._rb_view.count * self._rb_view.num_shapes
        await self._setup_random_attribute(attribute_name="contact_offset", value=value)
        new_value = self._rb_view._physics_view.get_contact_offsets().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_rest_offset(self):
        value = [100] * self._rb_view.count * self._rb_view.num_shapes
        await self._setup_random_attribute(attribute_name="rest_offset", value=value)
        new_value = self._rb_view._physics_view.get_rest_offsets().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_value, value)))
