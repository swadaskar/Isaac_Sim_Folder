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

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import create_new_stage_async
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView

import numpy as np
from scipy.spatial.transform import Rotation as R


class TestOgnWritePhysicsArticulationView(omni.kit.test.AsyncTestCase):
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

        self._articulation_view_node = self._controller.create_node(
            ("articulation_view", self._graph), "omni.replicator.isaac.OgnWritePhysicsArticulationView"
        )
        self._distribution_node = self._controller.create_node(
            ("uniform", self._graph), "omni.replicator.core.OgnSampleUniform"
        )
        self._articulation_view_node_prim = self._stage.GetPrimAtPath(self._articulation_view_node.get_prim_path())

        self._iface = omni.timeline.get_timeline_interface()

        # fetch franka asset and add to stage
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        franka_path = "/World/Franka"
        add_reference_to_stage(usd_path=asset_path, prim_path=franka_path)
        self._num_dofs = 7 + 2

        await omni.kit.app.get_app().next_update_async()
        self._articulation_view = ArticulationView(prim_paths_expr="/World/Franka", name="franka")
        self._my_world.scene.add(self._articulation_view)

        await self._my_world.reset_async()

        self._iface.play()
        dr.physics_view.register_articulation_view(self._articulation_view)
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        self._iface.stop()
        self._my_world.clear_instance()
        dr.physics_view._articulation_views = dict()
        dr.physics_view._articulation_views_initial_values = dict()
        await omni.usd.get_context().new_stage_async()

    async def _setup_random_attribute(self, attribute_name, value):
        self._distribution_node.get_attribute("inputs:numSamples").set(1)
        self._distribution_node.get_attribute("inputs:lower").set(value)
        self._distribution_node.get_attribute("inputs:upper").set(value)

        self._articulation_view_node.get_attribute("inputs:prims").set("franka")
        self._articulation_view_node.get_attribute("inputs:attribute").set(attribute_name)
        self._articulation_view_node.get_attribute("inputs:indices").set([0])
        self._articulation_view_node.get_attribute("inputs:operation").set("direct")

        self._controller.connect(
            self._distribution_node.get_attribute("outputs:samples"),
            self._articulation_view_node.get_attribute("inputs:values"),
        )
        await self._controller.evaluate(self._graph)

    async def test_randomize_stiffness(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="stiffness", value=value)
        stiffness, _ = self._articulation_view.get_gains()
        stiffness = stiffness.clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(stiffness, value)))

    async def test_randomize_damping(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="damping", value=value)
        _, damping = self._articulation_view.get_gains()
        damping = damping.clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(damping, value)))

    async def test_randomize_joint_friction(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_friction", value=value)
        dof_friction = self._articulation_view._physics_view.get_dof_friction_coefficients().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(dof_friction, value)))

    async def test_randomize_position(self):
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="position", value=value)
        root_position, _ = self._articulation_view.get_world_poses()
        root_position = root_position.clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(root_position, value)))

    async def test_randomize_orientation(self):
        value = [0, np.pi, 0]
        await self._setup_random_attribute(attribute_name="orientation", value=value)
        _, root_orientation = self._articulation_view.get_world_poses()
        root_orientation = root_orientation.clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(root_orientation, [0, 0, 1, 0], atol=1e-04)))

    async def test_randomize_velocities(self):
        value = [10] * 6
        await self._setup_random_attribute(attribute_name="velocity", value=value)
        velocities = self._articulation_view.get_velocities()
        self.assertTrue(np.all(np.isclose(value, velocities, atol=1e-04)))

    async def test_randomize_joint_positions(self):
        value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        await self._setup_random_attribute(attribute_name="joint_positions", value=value)
        dof_positions = self._articulation_view.get_joint_positions().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(dof_positions, value)))

    async def test_randomize_joint_velocities(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_velocities", value=value)
        dof_velocities = self._articulation_view.get_joint_velocities().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(dof_velocities, value)))

    async def test_randomize_lower_dof_limits(self):
        value = [-10, -20, -30, -40, -50, -60, -70, -80, -90]
        await self._setup_random_attribute(attribute_name="lower_dof_limits", value=value)
        lower_dof_limits = self._articulation_view._physics_view.get_dof_limits()[..., 0].clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(lower_dof_limits, value)))

    async def test_randomize_upper_dof_limits(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="upper_dof_limits", value=value)
        upper_dof_limits = self._articulation_view._physics_view.get_dof_limits()[..., 1].clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(upper_dof_limits, value)))

    async def test_randomize_max_efforts(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="max_efforts", value=value)
        dof_max_forces = self._articulation_view._physics_view.get_dof_max_forces().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(dof_max_forces, value)))

    async def test_randomize_armature(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_armatures", value=value)
        new_values = self._articulation_view._physics_view.get_dof_armatures().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_values, value)))

    async def test_randomize_max_velocities(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_max_velocities", value=value)
        new_values = self._articulation_view._physics_view.get_dof_max_velocities().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_values, value)))

    async def test_randomize_joint_efforts(self):
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_efforts", value=value)

    async def test_randomize_masses(self):
        if self._articulation_view._device == "cpu":
            value = [100] * self._articulation_view.count * self._articulation_view.num_bodies
            await self._setup_random_attribute(attribute_name="body_masses", value=value)
            new_value = self._articulation_view.get_body_masses().clone().cpu().numpy()
            self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_inertias(self):
        if self._articulation_view._device == "cpu":
            inertias = [0.1, 0.1, 0.1] * self._articulation_view.count * self._articulation_view.num_bodies
            await self._setup_random_attribute(attribute_name="body_inertias", value=inertias)
            new_value = self._articulation_view.get_body_inertias().clone().cpu().numpy()
            diagonal = new_value[:, :, [0, 4, 8]]
            self.assertTrue(np.all(np.isclose(diagonal.flatten(), inertias)))

    async def test_randomize_material_properties(self):
        value = [100] * self._articulation_view.count * self._articulation_view.num_shapes * 3
        await self._setup_random_attribute(attribute_name="material_properties", value=value)
        new_value = self._articulation_view._physics_view.get_material_properties().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_value.flatten(), value)))

    async def test_randomize_contact_offsets(self):
        value = [100] * self._articulation_view.count * self._articulation_view.num_shapes
        await self._setup_random_attribute(attribute_name="contact_offset", value=value)
        new_value = self._articulation_view._physics_view.get_contact_offsets().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_rest_offset(self):
        value = [100] * self._articulation_view.count * self._articulation_view.num_shapes
        await self._setup_random_attribute(attribute_name="rest_offset", value=value)
        new_value = self._articulation_view._physics_view.get_rest_offsets().clone().cpu().numpy()
        self.assertTrue(np.all(np.isclose(new_value, value)))
