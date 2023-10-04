# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
from omni.isaac.core.utils.types import DynamicsViewState

import omni.kit.test

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.prims import RigidPrimView

from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats as euler_angles_to_quats_numpy
from omni.isaac.core.utils.torch.rotations import euler_angles_to_quats as euler_angles_to_quats_torch

import carb
import numpy as np
import torch
import asyncio

from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid


default_physics_material = {"static_friction": 1.0, "dynamic_friction": 1.0, "restitution": 0.0}

default_sim_params = {
    ### Per-scene settings
    "use_gpu": False,
    "worker_thread_count": 4,
    "solver_type": 1,  # 0: PGS, 1:TGS
    "bounce_threshold_velocity": 0.2,
    "friction_offset_threshold": 0.04,  # A threshold of contact separation distance used to decide if a contact
    # point will experience friction forces.
    "friction_correlation_distance": 0.025,  # Contact points can be merged into a single friction anchor if the
    # distance between the contacts is smaller than correlation distance.
    # disabling these can be useful for debugging
    "enable_sleeping": True,
    "enable_stabilization": True,
    # GPU buffers
    "gpu_max_rigid_contact_count": 512 * 1024,
    "gpu_max_rigid_patch_count": 80 * 1024,
    "gpu_found_lost_pairs_capacity": 1024,
    "gpu_found_lost_aggregate_pairs_capacity": 1024,
    "gpu_total_aggregate_pairs_capacity": 1024,
    "gpu_max_soft_body_contacts": 1024 * 1024,
    "gpu_max_particle_contacts": 1024 * 1024,
    "gpu_heap_capacity": 64 * 1024 * 1024,
    "gpu_temp_buffer_capacity": 16 * 1024 * 1024,
    "gpu_max_num_partitions": 8,
    ### Per-actor settings ( can override in actor_options )
    "solver_position_iteration_count": 4,
    "solver_velocity_iteration_count": 1,
    "sleep_threshold": 0.0,  # Mass-normalized kinetic energy threshold below which an actor may go to sleep.
    # Allowed range [0, max_float).
    "stabilization_threshold": 0.0,  # Mass-normalized kinetic energy threshold below which an actor may
    # participate in stabilization. Allowed range [0, max_float).
    ### Per-body settings ( can override in actor_options )
    "enable_gyroscopic_forces": False,
    "density": 1000.0,  # density to be used for bodies that do not specify mass or density
    "max_depenetration_velocity": 100.0,
    ### Per-shape settings ( can override in actor_options )
    "contact_offset": 0.02,
    "rest_offset": 0.001,
    "gravity": [0.0, 0.0, 0.0],
    "dt": 1.0 / 60.0,
    "substeps": 1,
    "use_gpu_pipeline": False,
    "add_ground_plane": False,
    "default_physics_material": default_physics_material,
}


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRigidPrimView(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        self._sim_params = default_sim_params
        self._test_cfg = dict()

    async def tearDown(self):
        self._my_world.clear_instance()
        carb.settings.get_settings().set_bool("/physics/suppressReadback", False)

    async def test_rigid_prim_view_gpu_pipeline(self):
        test_configs = {"use_gpu": True, "use_gpu_pipeline": True, "device": "gpu"}
        for backend in ["torch"]:
            test_configs["backend"] = backend

            self._sim_params["use_gpu"] = test_configs["use_gpu"]
            self._sim_params["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
            self._test_cfg["use_gpu"] = test_configs["use_gpu"]
            self._test_cfg["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
            self._test_cfg["backend"] = test_configs["backend"]
            self._test_cfg["device"] = test_configs["device"]

            if backend == "torch":
                self.euler_angles_to_quats = euler_angles_to_quats_torch
                self.isclose = torch.isclose
                if self._test_cfg["device"] == "gpu":
                    self._array_container = torch.cuda.FloatTensor
                    self._device = "cuda:0"

            await self._runner()

    async def test_rigid_prim_view_cpu_pipeline(self):
        test_configs = {"use_gpu_pipeline": False, "device": "cpu"}

        for backend in ["numpy", "torch"]:
            for gpu_sim in [False, True]:
                self._sim_params["use_gpu"] = gpu_sim
                self._sim_params["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
                self._test_cfg["use_gpu"] = gpu_sim
                self._test_cfg["use_gpu_pipeline"] = test_configs["use_gpu_pipeline"]
                self._test_cfg["backend"] = backend
                self._test_cfg["device"] = test_configs["device"]

                if self._test_cfg["backend"] == "numpy":
                    self._array_container = np.array
                    self.euler_angles_to_quats = euler_angles_to_quats_numpy
                    self.isclose = np.isclose
                    self._device = "cpu"
                elif self._test_cfg["backend"] == "torch":
                    self.euler_angles_to_quats = euler_angles_to_quats_torch
                    self.isclose = torch.isclose
                    if self._test_cfg["device"] == "gpu":
                        self._array_container = torch.cuda.FloatTensor
                        self._device = "cuda"
                    else:
                        self._array_container = torch.Tensor
                        self._device = "cpu"

                await self._runner()

    async def _setup_scene(self):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(sim_params=self._sim_params, backend=self._test_cfg["backend"], device=self._device)
        await self._my_world.initialize_simulation_context_async()
        await update_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._my_world._physics_context.set_gravity(0)
        await omni.kit.app.get_app().next_update_async()

        num_cubes = 3
        for i in range(num_cubes):
            DynamicCuboid(
                prim_path=f"/World/Cube_{i+1}", name=f"cube_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=0.0
            )
        await update_stage_async()
        self._cubes_view = RigidPrimView(
            prim_paths_expr="/World/Cube_[1-3]",
            name="cubes_view",
            positions=self._array_container([[0.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, -10.0, 0.0]]),
        )
        self._my_world.scene.add(self._cubes_view)

    async def _setup_contacts_scene(self):
        self.cube_height = 0.51
        self.top_cube_height = self.cube_height + 1.1
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(sim_params=self._sim_params, backend=self._test_cfg["backend"], device=self._device)
        await self._my_world.initialize_simulation_context_async()
        await update_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._my_world._physics_context.set_gravity(-10)
        await omni.kit.app.get_app().next_update_async()
        self._my_world.scene.add_default_ground_plane()

        for i in range(3):
            DynamicCuboid(
                prim_path=f"/World/Box_{i+1}", name=f"box_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=1.0
            )
            DynamicCuboid(
                prim_path=f"/World/TopBox_{i+1}", name=f"top_box_{i}", size=1.0, color=np.array([0.5, 0, 0]), mass=1.0
            )
        # a view to receive contacts between the bottom boxes and top boxes
        self._box_view = RigidPrimView(
            prim_paths_expr="/World/Box_*",
            name="box_view",
            positions=self._array_container(
                [[10.0, 10.0, self.cube_height], [10.0, 20.0, self.cube_height], [10.0, 30.0, self.cube_height]]
            ),
            contact_filter_prim_paths_expr=["/World/TopBox_*"],
        )
        # a view just to manipulate the top boxes
        self._top_box_view = RigidPrimView(
            prim_paths_expr="/World/TopBox_*",
            name="top_box_view",
            positions=self._array_container(
                [
                    [10.0, 10.0, self.top_cube_height],
                    [10.0, 20.0, self.top_cube_height],
                    [10.0, 30.0, self.top_cube_height],
                ]
            ),
            track_contact_forces=True,
        )
        self._my_world.scene.add(self._box_view)
        self._my_world.scene.add(self._top_box_view)

    async def _runner(self):
        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(indexed, self._test_cfg)
            await self._setup_scene()
            await self.com_test()
            await self._setup_scene()
            await self.inertias_test()
            await self._setup_scene()
            await self.world_poses_test()
            await self._setup_scene()
            await self.linear_velocities_test()
            await self._setup_scene()
            await self.angular_velocities_test()
            await self._setup_scene()
            await self.transforms_test()
            await self._setup_scene()
            await self.velocities_test()
            await self._setup_scene()
            await self.apply_forces_test()
            await self._setup_scene()
            await self.enable_disable_physics_test()
            await self._setup_scene()
            await self.enable_disable_gravity_test()
            await self._setup_scene()
            await self.default_state_post_reset_test()
            await self._setup_scene()
            await self.masses_test()
            await self._setup_scene()
            await self.densities_test()
            await self._setup_scene()
            await self.sleep_thresholds_test()
            await self._setup_scene()
            await self.apply_forces_and_torques_at_pos_test(is_global=True, apply_at_pos=False)
            await self._setup_scene()
            await self.apply_forces_and_torques_at_pos_test(is_global=True, apply_at_pos=True)
            await self._setup_scene()
            await self.apply_forces_and_torques_at_pos_test(is_global=False, apply_at_pos=False)
            await self._setup_scene()
            await self.apply_forces_and_torques_at_pos_test(is_global=False, apply_at_pos=True)

            await self._setup_contacts_scene()
            await self.contact_force_test()

        # test USD paths
        if self._device == "cpu":
            for indexed in [False, True]:
                print("USD", indexed, self._test_cfg)
                self._test_cfg["indexed"] = indexed
                await self._setup_scene()
                await self.world_poses_test(usd=True)
                await self._setup_scene()
                await self.local_poses_test(usd=True)
                await self._setup_scene()
                await self.linear_velocities_test(usd=True)
                await self._setup_scene()
                await self.angular_velocities_test(usd=True)
                await self._setup_scene()
                await self.velocities_test(usd=True)
                await self._setup_scene()
                await self.enable_disable_physics_test(usd=True)
                await self._setup_scene()
                await self.enable_disable_gravity_test(usd=True)
                await self._setup_scene()
                await self.masses_test(usd=True)
                await self._setup_scene()
                await self.densities_test(usd=True)
                await self._setup_scene()
                await self.sleep_thresholds_test(usd=True)
                await self._setup_scene()
                await self.default_state_post_reset_test(usd=True)

        self._my_world.clear_instance()

    async def apply_forces_and_torques_at_pos_test(self, is_global, apply_at_pos):
        print(
            "Apply %s forces and torques %s test"
            % ("global" if is_global else "local", "at pos" if apply_at_pos else "at COM")
        )
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]

        new_positions = self._array_container([[20.0, -20.0, 10.0], [30.0, 30.0, 0], [-40, -40, 0]])
        new_orientations = self.euler_angles_to_quats(
            euler_angles=self._array_container([[np.pi / 2, 0, 0], [0, np.pi / 2, 0], [0, 0, np.pi / 2]]),
            device=self._device,
        )  # [y->z, z->x, x->y] rotation
        self._cubes_view.set_world_poses(
            positions=new_positions[indices], orientations=new_orientations[indices], indices=indices
        )

        position = None
        if is_global:
            force = self._array_container([[3000, 0, 0], [-3000, 0, 0], [3000, 0, 0]])[indices]
            torque = self._array_container([[0, 0, 3000], [0, 0, -3000], [0, 0, 3000]])[indices]
            if apply_at_pos:
                position = (self._array_container([[20.0, -19.0, 10.0], [30.0, 31.0, 0], [-40, -39, 0]]))[indices]
        else:
            force = (self._array_container([[3000, 0, 0], [0, 0, 3000], [0, 3000, 0]]))[indices]  # global x forces
            torque = (self._array_container([[0, 3000, 0], [3000, 0, 0], [0, 0, 3000]]))[indices]  # global z torques
            if apply_at_pos:
                position = (self._array_container([[0, 0, -1], [0, -1, 0], [-1, 0, 0]]))[indices]  # cancel the torques

        self._cubes_view.apply_forces_and_torques_at_pos(force, torque, position, indices, is_global)

        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()

        current_linear_velocities = self._cubes_view.get_linear_velocities(indices)
        current_angular_velocities = self._cubes_view.get_angular_velocities(indices)
        # print("linear velocities ", current_linear_velocities )
        # print("angular velocities ", current_angular_velocities )

        if self._test_cfg["backend"] == "torch":
            current_linear_velocities = current_linear_velocities.cpu().numpy()
            current_angular_velocities = current_angular_velocities.cpu().numpy()

        print(current_linear_velocities)
        print(current_angular_velocities)

        # linear velocity test
        self.assertTrue(np.isclose(current_linear_velocities[:, 1:], np.array([[0, 0], [0, 0], [0, 0]])[indices]).all())
        self.assertTrue(np.logical_not(np.isclose(current_linear_velocities[:, 0], np.array([0, 0, 0])[indices])).all())
        # angular velocity test
        if apply_at_pos:
            self.assertTrue(
                np.isclose(
                    current_angular_velocities[:, :], np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices], atol=1e-06
                ).all()
            )
        else:
            self.assertTrue(
                np.isclose(
                    current_angular_velocities[:, :-1],
                    np.array([[0, 0], [0, 0], [0, 0]])[indices].squeeze(),
                    atol=1e-06,
                ).all()
            )
            self.assertTrue(
                np.logical_not(
                    np.isclose(current_angular_velocities[:, -1], np.array([0, 0, 0])[indices].squeeze())
                ).all()
            )

    async def contact_force_test(self):
        print("contact force test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        for i in range(60):
            self._my_world.step_async()
            self._my_world._physics_sim_view.flush()
            await omni.kit.app.get_app().next_update_async()

        states = self._box_view.get_current_dynamic_state()
        top_states = self._top_box_view.get_current_dynamic_state()
        top_net_forces = self._top_box_view.get_net_contact_forces(indices, dt=self._sim_params["dt"])
        net_forces = self._box_view.get_net_contact_forces(indices, dt=self._sim_params["dt"])
        forces_matrix = self._box_view.get_contact_force_matrix(indices, dt=self._sim_params["dt"])
        # print("final forces: \n", net_forces)
        # print("matirx forces: \n", forces_matrix)
        # print("final positions: \n", states.positions)
        # print("final top positions: \n", top_states.positions)
        # print("final linear_velocities: \n", states.linear_velocities)
        # print("final top linear_velocities: \n", top_states.linear_velocities)

        if self._test_cfg["backend"] == "warp":
            # position test
            self.assertTrue(
                self.isclose(
                    states.positions.numpy()[indices],
                    np.array([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices],
                    atol=1.0e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.positions.numpy()[indices],
                    np.array([[10.0, 10, 1.5], [10.0, 20.0, 1.5], [10.0, 30.0, 1.5]])[indices],
                    atol=1.0e-4,
                ).all()
            )
            # velocity test
            self.assertTrue(
                self.isclose(
                    states.linear_velocities.numpy()[indices],
                    np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices],
                    atol=1.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.linear_velocities.numpy()[indices],
                    np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices],
                    atol=2.0e-1,
                ).all()
            )

            # force test for filter 0
            self.assertTrue(
                self.isclose(
                    forces_matrix.numpy()[:, 0, :],
                    np.array([[0, 0, -10], [0, 0, -10], [0, 0, -10]])[indices],
                    atol=2.0e-1,
                ).all()
            )

            # force test : net forces on box
            self.assertTrue(
                self.isclose(
                    net_forces.numpy(), np.array([[0, 0, 10], [0, 0, 10], [0, 0, 10]])[indices], atol=2.0e-1
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_net_forces.numpy(), np.array([[0, 0, 10], [0, 0, 10], [0, 0, 10]])[indices], atol=2.0e-1
                ).all()
            )
        else:
            # position test
            self.assertTrue(
                self.isclose(
                    states.positions[indices],
                    self._array_container([[10.0, 10, 0.5], [10.0, 20.0, 0.5], [10.0, 30.0, 0.5]])[indices].squeeze(),
                    atol=1.0e-4,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.positions[indices],
                    self._array_container([[10.0, 10, 1.5], [10.0, 20.0, 1.5], [10.0, 30.0, 1.5]])[indices].squeeze(),
                    atol=1.0e-4,
                ).all()
            )
            # velocity test
            self.assertTrue(
                self.isclose(
                    states.linear_velocities[indices],
                    self._array_container([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices].squeeze(),
                    atol=1.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_states.linear_velocities[indices],
                    self._array_container([[0, 0, 0], [0, 0, 0], [0, 0, 0]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

            # force test for filter 0
            self.assertTrue(
                self.isclose(
                    forces_matrix[:, 0, :].squeeze(),
                    self._array_container([[0, 0, -10], [0, 0, -10], [0, 0, -10]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

            # force test : net forces on box
            self.assertTrue(
                self.isclose(
                    net_forces.squeeze(),
                    self._array_container([[0, 0, 10], [0, 0, 10], [0, 0, 10]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )
            self.assertTrue(
                self.isclose(
                    top_net_forces.squeeze(),
                    self._array_container([[0, 0, 10], [0, 0, 10], [0, 0, 10]])[indices].squeeze(),
                    atol=2.0e-1,
                ).all()
            )

    async def com_test(self):
        if self._device == "cpu":
            await self._my_world.reset_async()
            await omni.kit.app.get_app().next_update_async()
            indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]

            cur_pos, cur_ori = self._cubes_view.get_coms()
            new_pos = cur_pos[indices] + 0.1
            new_ori = cur_ori[indices]
            self._cubes_view.set_coms(new_pos, new_ori, indices)
            pos, ori = self._cubes_view.get_coms(indices)
            self.assertTrue(self.isclose(new_pos, pos).all())
            self.assertTrue(self.isclose(new_ori, ori).all())

    async def inertias_test(self):
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else [0, 1, 2]

        prev_values = self._cubes_view.get_inertias()
        new_values = prev_values[indices]
        new_values[:, [0, 4, 8]] + 0.01
        self._cubes_view.set_inertias(new_values, indices)
        cur_values = self._cubes_view.get_inertias(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())

        inv_masses = self._cubes_view.get_inv_inertias()
        self.assertTrue(inv_masses.shape == (self._cubes_view.count, 9))

    async def world_poses_test(self, usd=False):
        print("world poses test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        new_positions = self._array_container([[25.0, -20.0, 10.0], [15.0, 10.0, 0.0], [-40.0, -40.0, 0.0]])
        new_orientations = self.euler_angles_to_quats(
            euler_angles=self._array_container([[0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]]),
            device=self._device,
        )
        self._cubes_view.set_world_poses(
            positions=new_positions[indices] if indices else new_positions,
            orientations=new_orientations[indices] if indices else new_orientations,
            indices=indices,
        )
        self._my_world.step_async(0)
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()
        # await asyncio.sleep(5.0)
        current_positions, current_orientations = self._cubes_view.get_world_poses(indices=indices)

        self.assertTrue(self.isclose(current_positions, new_positions[indices], atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, new_orientations[indices], atol=1e-4).all())

        return

    async def local_poses_test(self, usd=False):
        print("local poses test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        new_positions = self._array_container([[25.0, -20.0, 10.0], [15.0, 10.0, 0.0], [-40.0, -40.0, 0.0]])
        new_orientations = self.euler_angles_to_quats(
            euler_angles=self._array_container([[0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]]),
            device=self._device,
        )
        self._cubes_view.set_local_poses(
            translations=new_positions[indices] if indices else new_positions,
            orientations=new_orientations[indices] if indices else new_orientations,
            indices=indices,
        )
        self._my_world.step_async(0)
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()
        # await asyncio.sleep(5.0)
        current_positions, current_orientations = self._cubes_view.get_local_poses(indices=indices)
        self.assertTrue(self.isclose(current_positions, new_positions[indices], atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, new_orientations[indices], atol=1e-4).all())

    async def linear_velocities_test(self, usd=False):
        print("linear velocities test")
        if self._device == "cpu":
            await self._my_world.reset_async()
            await omni.kit.app.get_app().next_update_async()
            if usd:
                await self._my_world.stop_async()
                await omni.kit.app.get_app().next_update_async()
            indices = [1, 2] if self._test_cfg["indexed"] else None

            linear_velocities = self._array_container([[10.0, 0.0, 0.0], [20.0, 0.0, 0.0], [-10, 0, 0]])
            linear_velocities = linear_velocities[indices] if indices else linear_velocities
            self._cubes_view.set_linear_velocities(linear_velocities, indices)
            self._my_world.step_async()
            self._my_world._physics_sim_view.flush()
            await omni.kit.app.get_app().next_update_async()
            await asyncio.sleep(0.1)

            current_linear_velocities = self._cubes_view.get_linear_velocities(indices)
            print(current_linear_velocities)
            print(linear_velocities)
            self.assertTrue(self.isclose(current_linear_velocities, linear_velocities, atol=1e-4).all())

        return

    async def angular_velocities_test(self, usd=False):
        print("angular velocities test")
        if self._device == "cpu":
            await self._my_world.reset_async()
            await omni.kit.app.get_app().next_update_async()
            if usd:
                await self._my_world.stop_async()
                await omni.kit.app.get_app().next_update_async()
            indices = [1, 2] if self._test_cfg["indexed"] else None

            angular_velocities = self._array_container([[20.0, 0, 0], [0, -20, 0], [0, 0, 20]])
            angular_velocities = angular_velocities[indices] if indices else angular_velocities
            self._cubes_view.set_angular_velocities(angular_velocities, indices)
            self._my_world.step_async()
            self._my_world._physics_sim_view.flush()
            await omni.kit.app.get_app().next_update_async()
            current_angular_velocities = self._cubes_view.get_angular_velocities(indices)
            print(current_angular_velocities)
            print(angular_velocities)
            self.assertTrue(self.isclose(current_angular_velocities, angular_velocities, atol=1e-1).all())
        return

    async def masses_test(self, usd=False):
        print("masses test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        masses = self._array_container([10, 20, 30])
        masses = masses[indices] if indices else masses
        self._cubes_view.set_masses(masses, indices)
        current_masses = self._cubes_view.get_masses(indices)
        self.assertTrue(self.isclose(masses, current_masses, atol=1e-4).all())
        return

    async def densities_test(self, usd=False):
        print("densities test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        densities = self._array_container([10, 20, 30])
        densities = densities[indices] if indices else densities
        self._cubes_view.set_densities(densities, indices)
        current_densities = self._cubes_view.get_densities(indices)
        self.assertTrue(self.isclose(densities, current_densities, atol=1e-4).all())
        return

    async def sleep_thresholds_test(self, usd=False):
        print("sleep thresholds test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        thresholds = self._array_container([0.0, 0.2, 0.1])
        thresholds = thresholds[indices] if indices else thresholds
        self._cubes_view.set_sleep_thresholds(thresholds, indices)
        current_thresholds = self._cubes_view.get_sleep_thresholds(indices)
        self.assertTrue(self.isclose(thresholds, current_thresholds, atol=1e-4).all())
        return

    async def enable_disable_physics_test(self, usd=False):
        print("enable/disable physics test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        self._cubes_view.enable_rigid_body_physics(indices)
        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()
        self._cubes_view.disable_rigid_body_physics(indices)

        return

    async def enable_disable_gravity_test(self, usd=False):
        print("enable/disable gravity test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        self._cubes_view.enable_gravities(indices)
        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()
        self._cubes_view.disable_gravities(indices)
        return

    async def default_state_post_reset_test(self, usd=False):
        print("default state post test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        positions = self._array_container([[10.0, 10.0, 0.0], [0.0, 10.0, 0.0], [0.0, -10.0, 0.0]])
        orientations = self.euler_angles_to_quats(
            self._array_container([[0.0, np.pi / 4.0, 0], [0, 0, np.pi / 4.0], [0, 0, -np.pi / 8.0]]),
            device=self._device,
        )
        linear_velocities = self._array_container([[0.0, 10, 0], [10, 0, 0], [-10, 0, 0]])
        angular_velocities = self._array_container([[0.0, 360, 0], [360, 0, 0], [-360, 0, 0]])

        desired_default_state = DynamicsViewState(
            positions=positions[indices] if indices else positions,
            orientations=orientations[indices] if indices else orientations,
            linear_velocities=linear_velocities[indices] if indices else linear_velocities,
            angular_velocities=angular_velocities[indices] if indices else angular_velocities,
        )
        self._cubes_view.set_default_state(
            positions=desired_default_state.positions,
            orientations=desired_default_state.orientations,
            linear_velocities=desired_default_state.linear_velocities,
            angular_velocities=desired_default_state.angular_velocities,
            indices=indices,
        )
        default_state = self._cubes_view.get_default_state()
        self.assertTrue(
            self.isclose(desired_default_state.positions, default_state.positions[indices], atol=1e-4).all()
        )
        self.assertTrue(
            self.isclose(desired_default_state.orientations, default_state.orientations[indices], atol=1e-4).all()
        )
        self.assertTrue(
            self.isclose(
                desired_default_state.linear_velocities, default_state.linear_velocities[indices], atol=1e-4
            ).all()
        )
        self.assertTrue(
            self.isclose(
                desired_default_state.angular_velocities, default_state.angular_velocities[indices], atol=1e-4
            ).all()
        )

        if not self._test_cfg["indexed"]:
            # resets to default state
            self._cubes_view.post_reset()
            current_state = self._cubes_view.get_current_dynamic_state()
            self.assertTrue(self.isclose(desired_default_state.positions, current_state.positions, atol=1e-4).all())
            self.assertTrue(
                self.isclose(desired_default_state.orientations, current_state.orientations, atol=1e-4).all()
            )
            self.assertTrue(
                self.isclose(desired_default_state.linear_velocities, current_state.linear_velocities, atol=1e-4).all()
            )
            self.assertTrue(
                self.isclose(
                    desired_default_state.angular_velocities, current_state.angular_velocities, atol=1e-4
                ).all()
            )

    async def transforms_test(self):
        print("transforms test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        num_indices = 2 if self._test_cfg["indexed"] else 3

        new_positions = self._array_container([[25.0, -20.0, 10.0], [15.0, 10.0, 0.0], [-45.0, -40.0, 0.0]])
        new_orientations = self._array_container([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
        new_positions = new_positions[indices] if indices else new_positions
        new_orientations = new_orientations[indices] if indices else new_orientations
        self._cubes_view.set_world_poses(positions=new_positions, orientations=new_orientations, indices=indices)
        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()
        current_positions, current_orientations = self._cubes_view.get_world_poses(indices=indices)
        self.assertTrue(self.isclose(current_positions, new_positions, atol=1e-4).all())
        self.assertTrue(self.isclose(current_orientations, new_orientations, atol=1e-4).all())
        return

    async def velocities_test(self, usd=False):
        print("velocities test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        if usd:
            await self._my_world.stop_async()
            await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        velocities = self._array_container(
            [[10.0, 0.0, 0.0, 20.0, 0.0, 0.0], [20.0, 0.0, 0.0, 0.0, -20.0, 0.0], [-10.0, 0.0, 0.0, 0.0, 0.0, 20.0]]
        )
        velocities = velocities[indices] if indices else velocities
        self._cubes_view.set_velocities(velocities, indices)
        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()
        current_velocities = self._cubes_view.get_velocities(indices)
        self.assertTrue(self.isclose(current_velocities, velocities, atol=1e-1).all())
        return

    async def apply_forces_test(self):
        print("apply forces test")
        await self._my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None

        new_positions = self._array_container([[20.0, -20.0, 10.0], [30.0, 30.0, 0], [-40, -40, 0]])
        new_orientations = self.euler_angles_to_quats(
            euler_angles=self._array_container([[0.0, 0, 0], [0, 0, 0], [0, 0, 0]]), device=self._device
        )
        self._cubes_view.set_world_poses(
            positions=new_positions[indices] if indices else new_positions,
            orientations=new_orientations[indices] if indices else new_orientations,
            indices=indices,
        )

        forces = self._array_container([[3000, 0, 0], [-3000, 0, 0], [3000, 0, 0]])
        forces = forces[indices] if indices else forces
        self._cubes_view.apply_forces(forces, indices)
        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()

        current_linear_velocities = self._cubes_view.get_linear_velocities(indices)

        if self._test_cfg["backend"] == "torch":
            current_linear_velocities = current_linear_velocities.cpu().numpy()

        self.assertTrue(
            np.isclose(current_linear_velocities[:, 1:], np.array([[0, 0], [0, 0], [0, 0]])[indices], atol=1e-4).all()
        )

        if self._test_cfg["backend"] == "numpy":
            self.assertTrue(
                np.logical_not(np.isclose(current_linear_velocities[:, 0], np.array([0, 0, 0])[indices])).all()
            )
        elif self._test_cfg["backend"] == "torch":
            self.assertTrue(
                np.logical_not(np.isclose(current_linear_velocities[:, 0], np.array([0, 0, 0])[indices])).all()
            )
