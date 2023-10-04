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
import omni.kit.test

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.torch.rotations import euler_angles_to_quats
import numpy as np
from omni.isaac.core.utils.stage import create_new_stage_async, add_reference_to_stage, update_stage_async
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
import torch
import carb
import omni.physx as _physx

INDEXED = [True, False]
USD_PATH = [True, False]
BACKEND = ["torch", "numpy"]


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestArticulationView(omni.kit.test.AsyncTestCase):
    async def setUpWorld(self, backend="torch", device="cpu"):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(stage_units_in_meters=1.0, backend=backend, device=device)
        await self._my_world.initialize_simulation_context_async()
        await update_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._my_world.scene.add_default_ground_plane()
        self._my_world._physics_context.set_gravity(0)
        await omni.kit.app.get_app().next_update_async()

    async def add_frankas(self, backend, enable_dof_force_sensors=False):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
        positions = [[0, 0, 0], [0, 2, 0.0]]
        if backend == "numpy":
            positions = np.array(positions)
        elif backend == "torch":
            positions = torch.tensor(positions)
        elif backend == "warp":
            positions = wp.array(positions, device="cpu", dtype=wp.float32)
        self._frankas_view = ArticulationView(
            prim_paths_expr="/World/Franka_[1-2]",
            name="frankas_view",
            positions=positions,
            enable_dof_force_sensors=enable_dof_force_sensors,
        )
        self._my_world.scene.add(self._frankas_view)
        await self._my_world.reset_async()

    async def add_humanoids(self, backend):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Humanoid_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Humanoid_2")
        positions = [[0, 0, 1.5], [0, 5, 1.5]]
        if backend == "numpy":
            positions = np.array(positions)
        elif backend == "torch":
            positions = torch.tensor(positions)
        elif backend == "warp":
            positions = wp.array(positions, device="cpu", dtype=wp.float32)
        humanoids_view = ArticulationView(
            prim_paths_expr="/World/Humanoid_[1-2]", name="humanoids_view", positions=positions
        )
        self._humanoids_view = ArticulationView(prim_paths_expr="/World/Humanoid_[1-2]/torso", name="humanoids_view")
        self._my_world.scene.add(self._humanoids_view)
        await self._my_world.reset_async()

    async def add_cartpoles(self, backend):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Cartpole/cartpole.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Cartpole_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Cartpole_2")
        positions = [[0, 0, 2.0], [0, 5, 2.0]]
        if backend == "numpy":
            positions = np.array(positions)
        elif backend == "torch":
            positions = torch.tensor(positions)
        elif backend == "warp":
            positions = wp.array(positions, device="cpu", dtype=wp.float32)
        self._cartpoles_view = ArticulationView(
            prim_paths_expr="/World/Cartpole_[1-2]", name="cartpole_view", positions=positions
        )
        self._my_world.scene.add(self._cartpoles_view)
        await self._my_world.reset_async()

    async def add_shadow_hands(self, backend, device="cpu"):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/ShadowHand/shadow_hand_instanceable.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/ShadowHand_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/ShadowHand_2")
        positions = [[0, 0, 0.2], [0, 1, 0.2]]
        if backend == "numpy":
            positions = np.array(positions)
        elif backend == "torch":
            positions = torch.tensor(positions, device=device)
        elif backend == "warp":
            positions = wp.array(positions, device=device, dtype=wp.float32)
        self._hands_view = ArticulationView(
            prim_paths_expr="/World/ShadowHand_[1-2]", name="hands_view", positions=positions
        )
        self._my_world.scene.add(self._hands_view)
        await self._my_world.reset_async()

    async def tearDown(self):
        self._my_world.clear_instance()
        carb.settings.get_settings().set_bool("/physics/suppressReadback", False)
        await create_new_stage_async()

    async def _step(self):
        self._my_world.step_async()
        self._my_world._physics_sim_view.flush()
        await omni.kit.app.get_app().next_update_async()

    async def test_world_poses_torch(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                for device in ["cpu"] if usd else ["cpu", "cuda:0"]:
                    print("index:", indexed, "usd:", usd, "device:", device)
                    await self.setUpWorld(backend="torch", device=device)
                    await self.add_frankas(backend="torch")
                    if usd:
                        await self._my_world.stop_async()
                    if indexed:
                        gt_positions = torch.tensor([[100.0, 100.0, 0]], device=device)
                        gt_orientations = euler_angles_to_quats(torch.tensor([[0, 0, -np.pi / 2.0]], device=device))
                        self._frankas_view.set_world_poses(
                            positions=gt_positions, orientations=gt_orientations, indices=[1]
                        )
                        await self._step()
                        new_positions, new_orientations = self._frankas_view.get_world_poses(indices=[1])
                    else:
                        gt_positions = torch.tensor([[10.0, 10.0, 0], [100.0, 100.0, 0]], device=device)
                        gt_orientations = euler_angles_to_quats(
                            torch.tensor([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]], device=device)
                        )
                        self._frankas_view.set_world_poses(positions=gt_positions, orientations=gt_orientations)
                        await self._step()
                        new_positions, new_orientations = self._frankas_view.get_world_poses()
                    self.assertTrue(
                        np.isclose(new_positions.cpu().numpy(), gt_positions.cpu().numpy(), atol=1e-05).all()
                    )
                    self.assertTrue(
                        np.logical_or(
                            np.isclose(new_orientations.cpu().numpy(), gt_orientations.cpu().numpy(), atol=1e-05).all(
                                axis=1
                            ),
                            np.isclose(new_orientations.cpu().numpy(), -gt_orientations.cpu().numpy(), atol=1e-05).all(
                                axis=1
                            ),
                        ).all()
                    )
                    self._my_world.clear_instance()

    async def test_world_poses_numpy(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                await self.setUpWorld(backend="numpy")
                await self.add_frankas(backend="numpy")
                if usd:
                    await self._my_world.stop_async()
                if indexed:
                    gt_positions = np.array([[100.0, 100.0, 0]])
                    gt_orientations = euler_angles_to_quats(torch.tensor([[0, 0, -np.pi / 2.0]])).numpy()
                    self._frankas_view.set_world_poses(
                        positions=gt_positions, orientations=gt_orientations, indices=[1]
                    )
                    new_positions, new_orientations = self._frankas_view.get_world_poses(indices=[1])
                else:
                    gt_positions = np.array([[10.0, 10.0, 0], [100.0, 100.0, 0]])
                    gt_orientations = euler_angles_to_quats(
                        torch.tensor([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0]])
                    ).numpy()
                    self._frankas_view.set_world_poses(positions=gt_positions, orientations=gt_orientations)
                    new_positions, new_orientations = self._frankas_view.get_world_poses()
                self.assertTrue(np.isclose(new_positions, gt_positions, atol=1e-05).all())
                self.assertTrue(
                    np.logical_or(
                        np.isclose(new_orientations, gt_orientations, atol=1e-05).all(axis=1),
                        np.isclose(new_orientations, -gt_orientations, atol=1e-05).all(axis=1),
                    ).all()
                )
                self._my_world.clear_instance()

    async def test_velocities_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_humanoids(backend="torch")
                if indexed:
                    gt_v1 = torch.tensor([[0.0, 0.0, 0.1, 0.0, 0.0, 0.0]], device=device)
                    self._humanoids_view.set_velocities(gt_v1, indices=[1])
                    print(gt_v1)
                    await self._step()
                    new_v1 = self._humanoids_view.get_velocities(indices=[1])
                    print(new_v1)
                else:
                    gt_v1 = torch.tensor([[0.0, 0.0, 0.1, 0, 0, 0], [0.0, 0.0, 0.2, 0, 0, 0]], device=device)
                    self._humanoids_view.set_velocities(gt_v1)
                    await self._step()
                    new_v1 = self._humanoids_view.get_velocities()
                self.assertTrue(np.isclose(new_v1.cpu().numpy(), gt_v1.cpu().numpy(), atol=1e-05).all())
                self._my_world.clear_instance()

    async def test_velocities_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_humanoids(backend="numpy")
            if indexed:
                gt_v1 = np.array([[1.0, 2.0, 3.0, 0, 0, 0]])
                self._humanoids_view.set_velocities(gt_v1, indices=[1])
                new_v1 = self._humanoids_view.get_velocities(indices=[1])
            else:
                gt_v1 = np.array([[1.0, 2.0, 3.0, 0, 0, 0], [0.1, 0.2, 0.3, 0, 0, 0]])
                self._humanoids_view.set_velocities(gt_v1)
                new_v1 = self._humanoids_view.get_velocities()
            self.assertTrue(np.isclose(new_v1, gt_v1, atol=1e-05).all())
            self._my_world.clear_instance()

    async def test_linear_velocities_torch(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="torch")
            await self.add_frankas(backend="torch")
            if indexed:
                gt_v1 = torch.tensor([[1.0, 2.0, 3.0]])
                self._frankas_view.set_linear_velocities(gt_v1, indices=[1])
                new_v1 = self._frankas_view.get_linear_velocities(indices=[1])
            else:
                gt_v1 = torch.tensor([[1.0, 2.0, 3.0], [0.1, 0.2, 0.3]])
                self._frankas_view.set_linear_velocities(gt_v1)
                new_v1 = self._frankas_view.get_linear_velocities()
            self.assertTrue(np.isclose(new_v1.cpu().numpy(), gt_v1.cpu().numpy(), atol=1e-05).all())
            self._my_world.clear_instance()

    async def test_linear_velocities_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                gt_v1 = np.array([[1.0, 2.0, 3.0]])
                self._frankas_view.set_linear_velocities(gt_v1, indices=[1])
                new_v1 = self._frankas_view.get_linear_velocities(indices=[1])
            else:
                gt_v1 = np.array([[1.0, 2.0, 3.0], [0.1, 0.2, 0.3]])
                self._frankas_view.set_linear_velocities(gt_v1)
                new_v1 = self._frankas_view.get_linear_velocities()
            self.assertTrue(np.isclose(new_v1, gt_v1, atol=1e-05).all())
            self._my_world.clear_instance()

    async def test_angular_velocities_torch(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="torch")
            await self.add_frankas(backend="torch")
            if indexed:
                gt_v1 = torch.tensor([[1.0, 2.0, 3.0]])
                self._frankas_view.set_angular_velocities(gt_v1, indices=[1])
                new_v1 = self._frankas_view.get_angular_velocities(indices=[1])
            else:
                gt_v1 = torch.tensor([[1.0, 2.0, 3.0], [0.1, 0.2, 0.3]])
                self._frankas_view.set_angular_velocities(gt_v1)
                new_v1 = self._frankas_view.get_angular_velocities()
            self.assertTrue(np.isclose(new_v1.cpu().numpy(), gt_v1.cpu().numpy(), atol=1e-05).all())
            self._my_world.clear_instance()

    async def test_angular_velocities_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                gt_v1 = np.array([[1.0, 2.0, 3.0]])
                self._frankas_view.set_angular_velocities(gt_v1, indices=[1])
                new_v1 = self._frankas_view.get_angular_velocities(indices=[1])
            else:
                gt_v1 = np.array([[1.0, 2.0, 3.0], [0.1, 0.2, 0.3]])
                self._frankas_view.set_angular_velocities(gt_v1)
                new_v1 = self._frankas_view.get_angular_velocities()
            self.assertTrue(np.isclose(new_v1, gt_v1, atol=1e-05).all())
            self._my_world.clear_instance()

    async def test_friction_coefficients_torch(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                for device in ["cpu", "cuda:0"]:
                    print("index:", indexed, "usd:", usd, "device:", device)
                    await self.setUpWorld(backend="torch", device=device)
                    await self.add_frankas(backend="torch")
                    if usd:
                        await self._my_world.stop_async()
                    cur_friction = self._frankas_view.get_friction_coefficients()
                    if indexed:
                        new_friction = cur_friction
                        new_friction[1, 2] = cur_friction[1, 2] + 0.5
                        self._frankas_view.set_friction_coefficients(
                            new_friction[1, 2].unsqueeze(0).unsqueeze(0), indices=[1], joint_indices=[2]
                        )
                    else:
                        new_friction = cur_friction + 0.5
                        self._frankas_view.set_friction_coefficients(new_friction)
                    friction = self._frankas_view.get_friction_coefficients()
                    self.assertTrue(np.isclose(new_friction.cpu().numpy(), friction.cpu().numpy(), atol=1e-05).all())

                    self._my_world.clear_instance()

    async def test_friction_coefficients_numpy(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                await self.setUpWorld(backend="numpy", device="cpu")
                await self.add_frankas(backend="numpy")
                if usd:
                    await self._my_world.stop_async()
                cur_friction = self._frankas_view.get_friction_coefficients()
                if indexed:
                    new_friction = cur_friction
                    new_friction[1, 2] = cur_friction[1, 2] + 0.5
                    self._frankas_view.set_friction_coefficients(
                        np.array([[new_friction[1, 2]]]), indices=[1], joint_indices=[2]
                    )
                else:
                    new_friction = cur_friction + 0.5
                    self._frankas_view.set_friction_coefficients(new_friction)
                friction = self._frankas_view.get_friction_coefficients()
                self.assertTrue(np.isclose(new_friction, friction, atol=1e-05).all())

    async def test_armatures_torch(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                for device in ["cpu", "cuda:0"]:
                    print("index:", indexed, "usd:", usd, "device:", device)
                    await self.setUpWorld(backend="torch", device=device)
                    await self.add_frankas(backend="torch")
                    if usd:
                        await self._my_world.stop_async()
                    cur_value = self._frankas_view.get_armatures()
                    if indexed:
                        new_value = cur_value
                        new_value[1, 2] = cur_value[1, 2] + 0.5
                        self._frankas_view.set_armatures(
                            new_value[1, 2].unsqueeze(0).unsqueeze(0), indices=[1], joint_indices=[2]
                        )
                    else:
                        new_value = cur_value + 0.5
                        self._frankas_view.set_armatures(new_value)
                    value = self._frankas_view.get_armatures()
                    self.assertTrue(np.isclose(new_value.cpu().numpy(), value.cpu().numpy(), atol=1e-05).all())
                    self._my_world.clear_instance()

    async def test_armatures_numpy(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                await self.setUpWorld(backend="numpy", device="cpu")
                await self.add_frankas(backend="numpy")
                if usd:
                    await self._my_world.stop_async()
                cur_value = self._frankas_view.get_armatures()
                if indexed:
                    new_value = cur_value
                    new_value[1, 2] = cur_value[1, 2] + 0.5
                    self._frankas_view.set_armatures(np.array([[new_value[1, 2]]]), indices=[1], joint_indices=[2])
                else:
                    new_value = cur_value + 0.5
                    self._frankas_view.set_armatures(new_value)
                value = self._frankas_view.get_armatures()
                self.assertTrue(np.isclose(new_value, value, atol=1e-05).all())

    async def test_physics_callback(self):
        for backend in BACKEND:
            for indexed in INDEXED:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "device:", device)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)

                    def step_callback_1(step_size):
                        a = self._frankas_view.get_joint_positions()

                    physx_subs = _physx.get_physx_interface().subscribe_physics_step_events(step_callback_1)
                    # self._my_world.add_physics_callback(callback_name="sim_step", callback_fn=step_callback_1)
                    await self._my_world.reset_async()
                    await update_stage_async()
                    await update_stage_async()
                    await self._my_world.reset_async()
                    physx_subs = None
                    self._my_world.clear_instance()

    async def test_local_pose_torch(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                for device in ["cpu"] if usd else ["cpu", "cuda:0"]:
                    print("index:", indexed, "usd:", usd, "device:", device)
                    await self.setUpWorld(backend="torch", device=device)
                    await self.add_frankas(backend="torch")
                    if usd:
                        await self._my_world.stop_async()
                    cur_trans, cur_ori = self._frankas_view.get_local_poses()
                    if indexed:
                        new_trans = torch.tensor([[0, 1.0, 0]], device=device)
                        new_ori = torch.tensor([[np.sqrt(2) / 2, 0, 0, np.sqrt(2) / 2]], device=device)
                        self._frankas_view.set_local_poses(new_trans, new_ori, indices=[1])
                        await self._step()
                        trans, rot = self._frankas_view.get_local_poses(indices=[1])
                    else:
                        new_trans = torch.tensor([[0, 1.0, 0], [0, 2.0, 0]], device=device)
                        new_ori = torch.tensor(
                            [[np.sqrt(2) / 2, 0, 0, np.sqrt(2) / 2], [np.sqrt(2) / 2, 0, 0, np.sqrt(2) / 2]],
                            device=device,
                        )
                        self._frankas_view.set_local_poses(new_trans, new_ori)
                        await self._step()
                        trans, rot = self._frankas_view.get_local_poses()
                    self.assertTrue(np.isclose(new_trans.cpu().numpy(), trans.cpu().numpy(), atol=1e-05).all())
                    self.assertTrue(np.isclose(new_ori.cpu().numpy(), rot.cpu().numpy(), atol=1e-05).all())
                    self._my_world.clear_instance()

    async def test_local_pose_numpy(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                await self.setUpWorld(backend="numpy", device="cpu")
                await self.add_frankas(backend="numpy")
                if usd:
                    await self._my_world.stop_async()
                cur_trans, cur_ori = self._frankas_view.get_local_poses()
                if indexed:
                    new_trans = np.array([[0, 1.0, 0]])
                    new_ori = np.array([[np.sqrt(2) / 2, 0, 0, np.sqrt(2) / 2]])
                    self._frankas_view.set_local_poses(new_trans, new_ori, indices=[1])
                    trans, rot = self._frankas_view.get_local_poses(indices=[1])
                else:
                    new_trans = np.array([[0, 1.0, 0], [0, 2.0, 0]])
                    new_ori = np.array([[np.sqrt(2) / 2, 0, 0, np.sqrt(2) / 2], [np.sqrt(2) / 2, 0, 0, np.sqrt(2) / 2]])
                    self._frankas_view.set_local_poses(new_trans, new_ori)
                    trans, rot = self._frankas_view.get_local_poses()
                self.assertTrue(np.isclose(new_trans, trans, atol=1e-05).all())
                self.assertTrue(np.isclose(new_ori, rot, atol=1e-05).all())
                self._my_world.clear_instance()

    async def test_effort_modes(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "device:", device, "backend:", backend)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)
                    values = self._frankas_view.get_effort_modes()
                    if indexed:
                        self._frankas_view.set_effort_modes("force", indices=[1], joint_indices=[1, 3, 5])
                        values[1][1:6:2] = ["force"] * 3
                    else:
                        self._frankas_view.set_effort_modes("force")
                        values = [["force"] * len(values[0])] * len(values)
                    new_values = self._frankas_view.get_effort_modes()
                    self.assertTrue(values == new_values)

    async def test_gains_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                for usd in USD_PATH:
                    print("index:", indexed, "device:", device, "usd:", usd)
                    await self.setUpWorld(backend="torch", device=device)
                    await self.add_frankas(backend="torch")
                    if usd:
                        await self._my_world.stop_async()
                    if indexed:
                        old_kps, old_kds = self._frankas_view.get_gains(indices=[1], joint_indices=[1, 2])
                        new_kps = torch.tensor([[100.0, 100.0]], device=device)
                        self._frankas_view.set_gains(kps=new_kps, indices=[1], joint_indices=[1, 2])
                        kps, kds = self._frankas_view.get_gains(indices=[1], joint_indices=[1, 2])
                    else:
                        old_kps, old_kds = self._frankas_view.get_gains()
                        new_kps = torch.tensor(
                            [
                                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                            ],
                            device=device,
                        )
                        self._frankas_view.set_gains(kps=new_kps)
                        kps, kds = self._frankas_view.get_gains()
                    self.assertTrue(np.isclose(new_kps.cpu().numpy(), kps.cpu().numpy(), atol=1e-05).all())
                    self.assertTrue(np.isclose(old_kds.cpu().numpy(), kds.cpu().numpy(), atol=1e-05).all())

    async def test_gains_numpy(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                print("index:", indexed, "usd:", usd)
                await self.setUpWorld(backend="numpy")
                await self.add_frankas(backend="numpy")
                if usd:
                    await self._my_world.stop_async()
                if indexed:
                    old_kps, old_kds = self._frankas_view.get_gains(indices=[1], joint_indices=[1, 2])
                    new_kps = np.array([[100.0, 100.0]])
                    self._frankas_view.set_gains(kps=new_kps, indices=[1], joint_indices=[1, 2])
                    kps, kds = self._frankas_view.get_gains(indices=[1], joint_indices=[1, 2])
                else:
                    old_kps, old_kds = self._frankas_view.get_gains()
                    new_kps = np.array(
                        [
                            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                        ]
                    )
                    self._frankas_view.set_gains(kps=new_kps)
                    kps, kds = self._frankas_view.get_gains()
                self.assertTrue(np.isclose(new_kps, kps, atol=1e-05).all())
                self.assertTrue(np.isclose(old_kds, kds, atol=1e-05).all())

    async def test_switch_control_mode(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "device:", device, "backend:", backend)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)
                    self._frankas_view.switch_control_mode(mode="velocity")
                    kps, kds = self._frankas_view.get_gains()
                    if backend == "torch":
                        self.assertTrue(not np.any(kps.cpu().numpy()))
                        self.assertTrue(np.any(kds.cpu().numpy()))
                    elif backend == "numpy":
                        self.assertTrue(not np.any(kps))
                        self.assertTrue(np.any(kds))
                    else:
                        self.assertTrue(not np.any(kps.numpy()))
                        self.assertTrue(np.any(kds.numpy()))

    async def test_switch_dof_control_mode(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "device:", device, "backend:", backend)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)
                    if indexed:
                        self._frankas_view.switch_dof_control_mode(mode="velocity", dof_index=1, indices=[1])
                        kps, kds = self._frankas_view.get_gains(joint_indices=[1], indices=[1])
                    else:
                        self._frankas_view.switch_dof_control_mode(mode="velocity", dof_index=1)
                        kps, kds = self._frankas_view.get_gains(joint_indices=[1])
                    if backend == "torch":
                        self.assertTrue(not np.any(kps.cpu().numpy()))
                        self.assertTrue(np.any(kds.cpu().numpy()))
                    elif backend == "numpy":
                        self.assertTrue(not np.any(kps))
                        self.assertTrue(np.any(kds))
                    else:
                        self.assertTrue(not np.any(kps.numpy()))
                        self.assertTrue(np.any(kds.numpy()))

    async def test_max_efforts_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                for usd in USD_PATH:
                    print("index:", indexed, "device:", device, "usd:", usd)
                    await self.setUpWorld(backend="torch", device=device)
                    await self.add_frankas(backend="torch")
                    if usd:
                        await self._my_world.stop_async()
                    if indexed:
                        old_efforts = self._frankas_view.get_max_efforts(indices=[1], joint_indices=[1, 2])
                        new_efforts = torch.tensor([[100.0, 100.0]], device=device)
                        self._frankas_view.set_max_efforts(new_efforts, indices=[1], joint_indices=[1, 2])
                        efforts = self._frankas_view.get_max_efforts(indices=[1], joint_indices=[1, 2])
                    else:
                        old_efforts = self._frankas_view.get_max_efforts()
                        new_efforts = torch.tensor(
                            [
                                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                            ],
                            device=device,
                        )
                        self._frankas_view.set_max_efforts(new_efforts)
                        efforts = self._frankas_view.get_max_efforts()
                    self.assertTrue(np.isclose(new_efforts.cpu().numpy(), efforts.cpu().numpy(), atol=1e-05).all())

    async def test_max_efforts_numpy(self):
        for indexed in INDEXED:
            for usd in USD_PATH:
                print("index:", indexed, "usd:", usd)
                await self.setUpWorld(backend="numpy")
                await self.add_frankas(backend="numpy")
                if usd:
                    await self._my_world.stop_async()
                if indexed:
                    old_efforts = self._frankas_view.get_max_efforts(indices=[1], joint_indices=[1, 2])
                    new_efforts = np.array([[100.0, 100.0]])
                    self._frankas_view.set_max_efforts(new_efforts, indices=[1], joint_indices=[1, 2])
                    efforts = self._frankas_view.get_max_efforts(indices=[1], joint_indices=[1, 2])
                else:
                    old_efforts = self._frankas_view.get_max_efforts()
                    new_efforts = np.array(
                        [
                            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                        ]
                    )
                    self._frankas_view.set_max_efforts(new_efforts)
                    efforts = self._frankas_view.get_max_efforts()
                self.assertTrue(np.isclose(new_efforts, efforts, atol=1e-05).all())

    async def test_physics_properties_torch(self):
        for device in ["cpu", "cuda:0"]:
            await self.setUpWorld(backend="torch", device=device)
            await self.add_frankas(backend="torch")
            self._frankas_view.set_effort_modes("force")
            stiffness_tensor = torch.tensor(
                [
                    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                ],
                device=device,
            )
            damping_tensor = torch.tensor(
                [
                    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                ],
                device=device,
            )
            max_efforts_tensor = torch.tensor(
                [
                    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
                ],
                device=device,
            )
            self._frankas_view.set_gains(stiffness_tensor, damping_tensor)
            self._frankas_view.switch_control_mode("velocity", joint_indices=list(range(7)))
            self._frankas_view.switch_control_mode("position", joint_indices=[7, 8])
            self._frankas_view.set_max_efforts(max_efforts_tensor)

    async def test_physics_properties_numpy(self):
        await self.setUpWorld(backend="numpy")
        await self.add_frankas(backend="numpy")
        self._frankas_view.set_effort_modes("force")
        stiffness_tensor = np.array(
            [
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
            ]
        )
        damping_tensor = np.array(
            [
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
            ]
        )
        max_efforts_tensor = np.array(
            [
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0],
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 400.0],
            ]
        )
        self._frankas_view.set_gains(stiffness_tensor, damping_tensor)
        self._frankas_view.switch_control_mode("velocity", joint_indices=list(range(7)))
        self._frankas_view.switch_control_mode("position", joint_indices=[7, 8])
        self._frankas_view.set_max_efforts(max_efforts_tensor)

    async def test_initializing_views(self):
        for backend in BACKEND:
            for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                await self.setUpWorld(backend=backend, device=device)
                await self.add_frankas(backend=backend)
                robots = ArticulationView(prim_paths_expr="/World/Franka_[1-2]")
                robots.initialize()
                # right-finger
                self.left_fingers = RigidPrimView(prim_paths_expr="/World/Franka_[1-2]/panda_leftfinger")
                self.left_fingers.initialize()
                # # left-finger
                self.right_fingers = RigidPrimView(prim_paths_expr="/World/Franka_[1-2]/panda_rightfinger")
                self.right_fingers.initialize()

    async def test_physics_handles_none(self):
        for backend in BACKEND:
            for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                await self.setUpWorld(backend=backend, device=device)
                await self.add_frankas(backend=backend)
                robots = ArticulationView(prim_paths_expr="/World/Franka_[1-2]")
                robots.initialize()
                # right-finger
                self.assertTrue(robots.get_joint_positions() is not None)
                await self._my_world.stop_async()
                self.assertTrue(robots.get_joint_positions() is None)
                self.assertTrue(not robots.is_physics_handle_valid())
                self.assertTrue(robots.get_world_poses() is not None)
                await self._my_world.play_async()
                self.assertTrue(not robots.is_physics_handle_valid())
                robots.initialize()
                self.assertTrue(robots.is_physics_handle_valid())
                self.assertTrue(robots.get_joint_positions() is not None)

    async def test_position_targets_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                cur_value = self._frankas_view.get_applied_actions().joint_positions
                if indexed:
                    new_value = cur_value
                    new_value[1, 2] = cur_value[1, 2] + 0.5
                    self._frankas_view.set_joint_position_targets(new_value[1, 2], indices=[1], joint_indices=[2])
                else:
                    new_value = cur_value + 0.5
                    self._frankas_view.set_joint_position_targets(new_value)
                await self._step()
                value = self._frankas_view.get_applied_actions().joint_positions
                self.assertTrue(np.isclose(new_value.cpu().numpy(), value.cpu().numpy(), atol=1e-05).all())

                self._my_world.clear_instance()

    async def test_position_targets_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy", device="cpu")
            await self.add_frankas(backend="numpy")
            cur_value = self._frankas_view.get_applied_actions().joint_positions
            if indexed:
                new_value = cur_value
                new_value[1, 2] = cur_value[1, 2] + 0.5
                self._frankas_view.set_joint_position_targets(new_value[1, 2], indices=[1], joint_indices=[2])
            else:
                new_value = cur_value + 0.5
                self._frankas_view.set_joint_position_targets(new_value)
            await self._step()
            value = self._frankas_view.get_applied_actions().joint_positions
            self.assertTrue(np.isclose(new_value, value, atol=1e-05).all())

    async def test_velocity_targets_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                cur_value = self._frankas_view.get_applied_actions().joint_velocities
                if indexed:
                    new_value = cur_value
                    new_value[1, 2] = cur_value[1, 2] + 0.5
                    self._frankas_view.set_joint_velocity_targets(new_value[1, 2], indices=[1], joint_indices=[2])
                else:
                    new_value = cur_value + 0.5
                    self._frankas_view.set_joint_velocity_targets(new_value)
                await self._step()
                value = self._frankas_view.get_applied_actions().joint_velocities
                self.assertTrue(np.isclose(new_value.cpu().numpy(), value.cpu().numpy(), atol=1e-05).all())
                self._my_world.clear_instance()

    async def test_velocity_targets_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy", device="cpu")
            await self.add_frankas(backend="numpy")
            cur_value = self._frankas_view.get_applied_actions().joint_velocities
            if indexed:
                new_value = cur_value
                new_value[1, 2] = cur_value[1, 2] + 0.5
                self._frankas_view.set_joint_velocity_targets(new_value[1, 2], indices=[1], joint_indices=[2])
            else:
                new_value = cur_value + 0.5
                self._frankas_view.set_joint_velocity_targets(new_value)
            value = self._frankas_view.get_applied_actions().joint_velocities
            self.assertTrue(np.isclose(new_value, value, atol=1e-05).all())

    async def test_joint_velocities_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_cartpoles(backend="torch")
                cur_value = self._cartpoles_view.get_joint_velocities()
                if indexed:
                    new_value = torch.tensor([[0.1]], device=device)
                    self._cartpoles_view.set_joint_velocities(new_value, indices=[1], joint_indices=[0])
                    await self._step()
                    value = self._cartpoles_view.get_joint_velocities(indices=[1], joint_indices=[0])
                else:
                    new_value = torch.tensor([[0.1, 0.1], [0.1, 0.1]], device=device)
                    self._cartpoles_view.set_joint_velocities(new_value)
                    await self._step()
                    value = self._cartpoles_view.get_joint_velocities()
                self.assertTrue(np.isclose(new_value.cpu().numpy(), value.cpu().numpy(), atol=1e-03).all())

                self._my_world.clear_instance()

    async def test_joint_velocities_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy", device="cpu")
            await self.add_cartpoles(backend="numpy")
            cur_value = self._cartpoles_view.get_joint_velocities()
            if indexed:
                new_value = np.array([[0.1]])
                self._cartpoles_view.set_joint_velocities(new_value, indices=[1], joint_indices=[0])
                value = self._cartpoles_view.get_joint_velocities(indices=[1], joint_indices=[0])
            else:
                new_value = np.array([[0.1, 0.1], [0.1, 0.1]])
                self._cartpoles_view.set_joint_velocities(new_value)
                value = self._cartpoles_view.get_joint_velocities()
            self.assertTrue(np.isclose(new_value, value, atol=1e-03).all())

    async def test_joint_positions_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                cur_value = self._frankas_view.get_joint_positions()
                if indexed:
                    new_value = torch.tensor([[0.02, 0.02]], device=device)
                    self._frankas_view.set_joint_positions(new_value, indices=[1], joint_indices=[7, 8])
                    await self._step()
                    value = self._frankas_view.get_joint_positions(indices=[1], joint_indices=[7, 8])
                else:
                    new_value = torch.tensor(
                        [
                            [0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.02, 0.02],
                            [0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.02, 0.02],
                        ],
                        device=device,
                    )
                    self._frankas_view.set_joint_positions(new_value)
                    await self._step()
                    value = self._frankas_view.get_joint_positions()
                self.assertTrue(np.isclose(new_value.cpu().numpy(), value.cpu().numpy(), atol=1e-05).all())

                self._my_world.clear_instance()

    async def test_joint_positions_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy", device="cpu")
            await self.add_frankas(backend="numpy")
            cur_value = self._frankas_view.get_joint_positions()
            if indexed:
                new_value = np.array([[0.02, 0.02]])
                self._frankas_view.set_joint_positions(new_value, indices=[1], joint_indices=[7, 8])
                value = self._frankas_view.get_joint_positions(indices=[1], joint_indices=[7, 8])
            else:
                new_value = np.array(
                    [
                        [0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.02, 0.02],
                        [0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.02, 0.02],
                    ]
                )
                self._frankas_view.set_joint_positions(new_value)
                value = self._frankas_view.get_joint_positions()
            self.assertTrue(np.isclose(new_value, value, atol=1e-05).all())

    async def test_joint_efforts_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                cur_value = torch.zeros((self._frankas_view.count, self._frankas_view._num_dof), device=device)
                if indexed:
                    new_value = cur_value.clone()
                    new_value[1, 2] = cur_value[1, 2] + 0.5
                    self._frankas_view.set_joint_efforts(new_value[1, 2], indices=[1], joint_indices=[2])
                else:
                    new_value = cur_value + 0.5
                    self._frankas_view.set_joint_efforts(new_value)
                await omni.kit.app.get_app().next_update_async()
                self._my_world.clear_instance()

    async def test_joint_efforts_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy", device="cpu")
            await self.add_frankas(backend="numpy")
            cur_value = np.zeros((self._frankas_view.count, self._frankas_view._num_dof))
            if indexed:
                new_value = cur_value.copy()
                new_value[1, 2] = cur_value[1, 2] + 0.5
                self._frankas_view.set_joint_efforts(new_value[1, 2], indices=[1], joint_indices=[2])
            else:
                new_value = cur_value + 0.5
                self._frankas_view.set_joint_efforts(new_value)

    async def test_body_indices(self):
        await self.setUpWorld(backend="numpy", device="cpu")
        await self.add_frankas(backend="numpy")
        await self._my_world.reset_async()
        # ground-truth values
        body_names = {"panda_link0": 0, "panda_rightfinger": 11}
        # test
        for name, value in body_names.items():
            self.assertEqual(self._frankas_view.get_body_index(name), value)

    async def test_efforts(self):
        await self.setUpWorld(backend="numpy", device="cpu")
        await self.add_frankas(backend="numpy")
        await self._my_world.reset_async()
        current_forces = self._frankas_view.get_applied_joint_efforts()
        self.assertTrue(current_forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))
        new_forces = current_forces + 100
        self._frankas_view.set_joint_efforts(new_forces)
        current_forces = self._frankas_view.get_applied_joint_efforts()
        self.assertTrue(current_forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))
        self.assertTrue(np.isclose(current_forces, new_forces).all())
        self.assertTrue(np.isclose(current_forces, self._frankas_view.get_applied_actions().joint_efforts).all())
        self._my_world.clear_instance()

        await self.setUpWorld(backend="torch", device="cuda:0")
        await self.add_frankas(backend="torch")
        await self._my_world.reset_async()
        current_forces = self._frankas_view.get_applied_joint_efforts()
        self.assertTrue(current_forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))
        new_forces = current_forces + 100
        self._frankas_view.set_joint_efforts(new_forces)
        current_forces = self._frankas_view.get_applied_joint_efforts()
        self.assertTrue(current_forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))
        self.assertTrue(torch.isclose(current_forces, new_forces).all())
        self.assertTrue(torch.isclose(current_forces, self._frankas_view.get_applied_actions().joint_efforts).all())

    async def test_jacobians(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "backend:", backend, "device:", device)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)
                    await self._my_world.reset_async()
                    jacobian_shape = self._frankas_view.get_jacobian_shape()
                    if indexed:
                        jacobians = self._frankas_view.get_jacobians(indices=[1])
                    else:
                        jacobians = self._frankas_view.get_jacobians()
                    if backend == "warp":
                        jacobians = jacobians.numpy()
                    elif backend == "torch":
                        jacobians = jacobians.cpu().numpy()
                    self.assertTrue(tuple(jacobians[0].shape) == jacobian_shape)
                    self.assertTrue(jacobians.shape[0] == 1 if indexed else self._frankas_view.count)
                    is_nan = np.where(np.isnan(jacobians))
                    for i in is_nan:
                        self.assertTrue(len(i) == 0)
                    self._my_world.clear_instance()

    async def test_mass_matrices(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "backend:", backend, "device:", device)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)
                    await self._my_world.reset_async()
                    mass_matrix_shape = self._frankas_view.get_mass_matrix_shape()
                    if indexed:
                        mass_matrices = self._frankas_view.get_mass_matrices(indices=[1])
                    else:
                        mass_matrices = self._frankas_view.get_mass_matrices()
                    if backend == "warp":
                        mass_matrices = mass_matrices.numpy()
                    elif backend == "torch":
                        mass_matrices = mass_matrices.cpu().numpy()
                    self.assertTrue(tuple(mass_matrices[0].shape) == mass_matrix_shape)
                    self.assertTrue(mass_matrices.shape[0] == 1 if indexed else self._frankas_view.count)
                    is_nan = np.where(np.isnan(mass_matrices))
                    for i in is_nan:
                        self.assertTrue(len(i) == 0)

    async def test_coriolis_centrifugal(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "backend:", backend, "device:", device)
                    await self.setUpWorld(backend=backend, device=device)
                    await self.add_frankas(backend=backend)
                    await self._my_world.reset_async()
                    if indexed:
                        forces = self._frankas_view.get_coriolis_and_centrifugal_forces(
                            indices=[1], joint_indices=[1, 2]
                        )
                        self.assertTrue(forces.shape == (1, 2))
                    else:
                        forces = self._frankas_view.get_coriolis_and_centrifugal_forces()
                        self.assertTrue(forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))
                    self._my_world.clear_instance()

    async def test_generalized_gravity(self):
        for indexed in INDEXED:
            for backend in BACKEND:
                for device in ["cpu", "cuda:0"] if backend != "numpy" else ["cpu"]:
                    print("index:", indexed, "backend:", backend, "device:", device)
                    await self.setUpWorld(backend=backend, device=device)
                    self._my_world.get_physics_context().set_gravity(0.0)
                    await self.add_frankas(backend=backend)
                    await self._my_world.reset_async()
                    if indexed:
                        forces = self._frankas_view.get_generalized_gravity_forces(indices=[1], joint_indices=[1, 2])
                        self.assertTrue(forces.shape == (1, 2))
                    else:
                        forces = self._frankas_view.get_generalized_gravity_forces()
                        self.assertTrue(forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))

                    if backend == "torch":
                        forces = forces.cpu().numpy()
                    elif backend == "warp":
                        forces = forces.numpy()
                    self.assertTrue(np.count_nonzero(forces) == 0)

                    self._my_world.clear_instance()
                    await self.setUpWorld(backend=backend, device=device)
                    self._my_world.get_physics_context().set_gravity(-9.81)
                    await self.add_frankas(backend=backend)
                    await self._my_world.reset_async()
                    if indexed:
                        forces = self._frankas_view.get_generalized_gravity_forces(indices=[1], joint_indices=[1, 2])
                        self.assertTrue(forces.shape == (1, 2))
                        if backend == "torch":
                            forces = forces.cpu().numpy()
                        elif backend == "warp":
                            forces = forces.numpy()
                        self.assertTrue(np.count_nonzero(forces) == 2)
                    else:
                        forces = self._frankas_view.get_generalized_gravity_forces()
                        self.assertTrue(forces.shape == (self._frankas_view.count, self._frankas_view.num_dof))
                        if backend == "torch":
                            forces = forces.cpu().numpy()
                        elif backend == "warp":
                            forces = forces.numpy()
                        self.assertTrue(
                            np.count_nonzero(forces) == self._frankas_view.count * self._frankas_view.num_dof
                        )

                    self._my_world.clear_instance()

    async def test_masses_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    new_values = torch.tensor([100.0, 200.0, 300.0], device=device)
                    self._frankas_view.set_body_masses(new_values, indices=[1], body_indices=[1, 3, 5])
                    values = self._frankas_view.get_body_masses(indices=[1], body_indices=[1, 3, 5])

                    inv_masses = self._frankas_view.get_body_inv_masses(indices=[1], body_indices=[1, 3, 5])
                    self.assertTrue(inv_masses.shape == (1, 3))
                else:
                    new_values = (
                        torch.zeros((self._frankas_view.count, self._frankas_view._num_bodies), device=device) + 100.0
                    )
                    self._frankas_view.set_body_masses(new_values)
                    values = self._frankas_view.get_body_masses()

                    inv_masses = self._frankas_view.get_body_inv_masses()
                    self.assertTrue(inv_masses.shape == (self._frankas_view.count, self._frankas_view._num_bodies))
                self.assertTrue(np.allclose(values.cpu().numpy(), new_values.cpu().numpy()))

    async def test_masses_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                new_values = np.array([100.0, 200.0, 300.0])
                self._frankas_view.set_body_masses(new_values, indices=[1], body_indices=[1, 3, 5])
                values = self._frankas_view.get_body_masses(indices=[1], body_indices=[1, 3, 5])

                inv_masses = self._frankas_view.get_body_inv_masses(indices=[1], body_indices=[1, 3, 5])
                self.assertTrue(inv_masses.shape == (1, 3))
            else:
                new_values = np.zeros((self._frankas_view.count, self._frankas_view._num_bodies)) + 100.0
                self._frankas_view.set_body_masses(new_values)
                values = self._frankas_view.get_body_masses()

                inv_masses = self._frankas_view.get_body_inv_masses()
                self.assertTrue(inv_masses.shape == (self._frankas_view.count, self._frankas_view._num_bodies))
            self.assertTrue(np.allclose(values, new_values, atol=1e-05))

    async def test_com_torch(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="torch")
            await self.add_frankas(backend="torch")
            if indexed:
                cur_pos, cur_ori = self._frankas_view.get_body_coms(indices=[1], body_indices=[1, 3, 5])
                new_pos = cur_pos + 0.1
                self._frankas_view.set_body_coms(new_pos, cur_ori, indices=[1], body_indices=[1, 3, 5])
                pos, ori = self._frankas_view.get_body_coms(indices=[1], body_indices=[1, 3, 5])
            else:
                cur_pos, cur_ori = self._frankas_view.get_body_coms()
                new_pos = cur_pos + 0.1
                self._frankas_view.set_body_coms(new_pos, cur_ori)
                pos, ori = self._frankas_view.get_body_coms()

            self.assertTrue(np.allclose(new_pos.cpu().numpy(), pos.cpu().numpy(), atol=1e-05))
            self.assertTrue(np.allclose(cur_ori.cpu().numpy(), ori.cpu().numpy(), atol=1e-05))

    async def test_com_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                cur_pos, cur_ori = self._frankas_view.get_body_coms(indices=[1], body_indices=[1, 3, 5])
                new_pos = cur_pos + 0.1
                self._frankas_view.set_body_coms(new_pos, cur_ori, indices=[1], body_indices=[1, 3, 5])
                pos, ori = self._frankas_view.get_body_coms(indices=[1], body_indices=[1, 3, 5])
            else:
                cur_pos, cur_ori = self._frankas_view.get_body_coms()
                new_pos = cur_pos + 0.1
                self._frankas_view.set_body_coms(new_pos, cur_ori)
                pos, ori = self._frankas_view.get_body_coms()
            self.assertTrue(np.allclose(new_pos, pos, atol=1e-05))
            self.assertTrue(np.allclose(cur_ori, ori, atol=1e-05))

    async def test_inertia_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    cur_value = self._frankas_view.get_body_inertias(indices=[1], body_indices=[1, 3, 5])
                    offset = torch.zeros_like(cur_value)
                    offset[:, :, [0, 4, 8]] += 0.1
                    new_value = cur_value + offset
                    self._frankas_view.set_body_inertias(new_value, indices=[1], body_indices=[1, 3, 5])
                    value = self._frankas_view.get_body_inertias(indices=[1], body_indices=[1, 3, 5])
                    inv_masses = self._frankas_view.get_body_inv_inertias(indices=[1], body_indices=[1, 3, 5])
                    self.assertTrue(inv_masses.shape == (1, 3, 9))
                else:
                    cur_value = self._frankas_view.get_body_inertias()
                    offset = torch.zeros_like(cur_value)
                    offset[:, :, [0, 4, 8]] += 0.1
                    new_value = cur_value + offset
                    self._frankas_view.set_body_inertias(new_value)
                    value = self._frankas_view.get_body_inertias()
                    inv_masses = self._frankas_view.get_body_inv_inertias()
                    self.assertTrue(inv_masses.shape == (self._frankas_view.count, self._frankas_view.num_bodies, 9))
                self.assertTrue(np.allclose(new_value.cpu().numpy(), value.cpu().numpy(), atol=1e-05))

    async def test_inertia_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                cur_value = self._frankas_view.get_body_inertias(indices=[1], body_indices=[1, 3, 5])
                offset = np.zeros_like(cur_value)
                offset[:, :, [0, 4, 8]] += 0.1
                new_value = cur_value + offset
                self._frankas_view.set_body_inertias(new_value, indices=[1], body_indices=[1, 3, 5])
                value = self._frankas_view.get_body_inertias(indices=[1], body_indices=[1, 3, 5])

                inv_masses = self._frankas_view.get_body_inv_inertias(indices=[1], body_indices=[1, 3, 5])
                self.assertTrue(inv_masses.shape == (1, 3, 9))
            else:
                cur_value = self._frankas_view.get_body_inertias()
                offset = np.zeros_like(cur_value)
                offset[:, :, [0, 4, 8]] += 0.1
                new_value = cur_value + offset
                self._frankas_view.set_body_inertias(new_value)
                value = self._frankas_view.get_body_inertias()

                inv_masses = self._frankas_view.get_body_inv_inertias()
                self.assertTrue(inv_masses.shape == (self._frankas_view.count, self._frankas_view.num_bodies, 9))
            self.assertTrue(np.allclose(new_value, value, atol=1e-05))

    async def test_fixed_tendon_properties_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_shadow_hands(backend="torch", device=device)

                if indexed:
                    new_stiffness = torch.tensor([0.1, 0.2, 0.3, 0.4], device=device)
                    new_dampings = torch.tensor([0.1, 0.2, 0.3, 0.4], device=device)
                    new_limit_stiffness = torch.tensor([0.1, 0.2, 0.3, 0.4], device=device)
                    new_limits = torch.tensor([[0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [0.7, 0.8]], device=device)
                    new_rest_lengths = torch.tensor([0.1, 0.2, 0.3, 0.4], device=device)
                    new_offsets = torch.tensor([0.1, 0.2, 0.3, 0.4], device=device)

                    self._hands_view.set_fixed_tendon_properties(
                        stiffnesses=new_stiffness,
                        dampings=new_dampings,
                        limit_stiffnesses=new_limit_stiffness,
                        limits=new_limits,
                        rest_lengths=new_rest_lengths,
                        offsets=new_offsets,
                        indices=[1],
                    )

                    if device == "cpu":
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_stiffnesses(indices=[1]).cpu().numpy(),
                                new_stiffness.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_dampings(indices=[1]).cpu().numpy(),
                                new_dampings.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_limit_stiffnesses(indices=[1]).cpu().numpy(),
                                new_limit_stiffness.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_limits(indices=[1]).cpu().numpy(),
                                new_limits.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_rest_lengths(indices=[1]).cpu().numpy(),
                                new_rest_lengths.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_offsets(indices=[1]).cpu().numpy(),
                                new_offsets.cpu().numpy(),
                                atol=1e-05,
                            )
                        )

                else:
                    new_stiffness = torch.tensor([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]], device=device)
                    new_dampings = torch.tensor([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]], device=device)
                    new_limit_stiffness = torch.tensor([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]], device=device)
                    new_limits = torch.tensor(
                        [
                            [[0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [0.7, 0.8]],
                            [[0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [0.7, 0.8]],
                        ],
                        device=device,
                    )
                    new_rest_lengths = torch.tensor([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]], device=device)
                    new_offsets = torch.tensor([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]], device=device)

                    self._hands_view.set_fixed_tendon_properties(
                        stiffnesses=new_stiffness,
                        dampings=new_dampings,
                        limit_stiffnesses=new_limit_stiffness,
                        limits=new_limits,
                        rest_lengths=new_rest_lengths,
                        offsets=new_offsets,
                    )

                    if device == "cpu":
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_stiffnesses().cpu().numpy(),
                                new_stiffness.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_dampings().cpu().numpy(),
                                new_dampings.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_limit_stiffnesses().cpu().numpy(),
                                new_limit_stiffness.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_limits().cpu().numpy(),
                                new_limits.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_rest_lengths().cpu().numpy(),
                                new_rest_lengths.cpu().numpy(),
                                atol=1e-05,
                            )
                        )
                        self.assertTrue(
                            np.allclose(
                                self._hands_view.get_fixed_tendon_offsets().cpu().numpy(),
                                new_offsets.cpu().numpy(),
                                atol=1e-05,
                            )
                        )

    async def test_fixed_tendon_properties_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_shadow_hands(backend="numpy")

            if indexed:
                new_stiffness = np.array([0.1, 0.2, 0.3, 0.4])
                new_dampings = np.array([0.1, 0.2, 0.3, 0.4])
                new_limit_stiffness = np.array([0.1, 0.2, 0.3, 0.4])
                new_limits = np.array([[0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [0.7, 0.8]])
                new_rest_lengths = np.array([0.1, 0.2, 0.3, 0.4])
                new_offsets = np.array([0.1, 0.2, 0.3, 0.4])

                self._hands_view.set_fixed_tendon_properties(
                    stiffnesses=new_stiffness,
                    dampings=new_dampings,
                    limit_stiffnesses=new_limit_stiffness,
                    limits=new_limits,
                    rest_lengths=new_rest_lengths,
                    offsets=new_offsets,
                    indices=[1],
                )

                self.assertTrue(
                    np.allclose(self._hands_view.get_fixed_tendon_stiffnesses(indices=[1]), new_stiffness, atol=1e-05)
                )
                self.assertTrue(
                    np.allclose(self._hands_view.get_fixed_tendon_dampings(indices=[1]), new_dampings, atol=1e-05)
                )
                self.assertTrue(
                    np.allclose(
                        self._hands_view.get_fixed_tendon_limit_stiffnesses(indices=[1]),
                        new_limit_stiffness,
                        atol=1e-05,
                    )
                )
                self.assertTrue(
                    np.allclose(self._hands_view.get_fixed_tendon_limits(indices=[1]), new_limits, atol=1e-05)
                )
                self.assertTrue(
                    np.allclose(
                        self._hands_view.get_fixed_tendon_rest_lengths(indices=[1]), new_rest_lengths, atol=1e-05
                    )
                )
                self.assertTrue(
                    np.allclose(self._hands_view.get_fixed_tendon_offsets(indices=[1]), new_offsets, atol=1e-05)
                )

            else:
                new_stiffness = np.array([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]])
                new_dampings = np.array([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]])
                new_limit_stiffness = np.array([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]])
                new_limits = np.array(
                    [[[0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [0.7, 0.8]], [[0.1, 0.2], [0.3, 0.4], [0.5, 0.6], [0.7, 0.8]]]
                )
                new_rest_lengths = np.array([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]])
                new_offsets = np.array([[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]])

                self._hands_view.set_fixed_tendon_properties(
                    stiffnesses=new_stiffness,
                    dampings=new_dampings,
                    limit_stiffnesses=new_limit_stiffness,
                    limits=new_limits,
                    rest_lengths=new_rest_lengths,
                    offsets=new_offsets,
                )

                self.assertTrue(np.allclose(self._hands_view.get_fixed_tendon_stiffnesses(), new_stiffness, atol=1e-05))
                self.assertTrue(np.allclose(self._hands_view.get_fixed_tendon_dampings(), new_dampings, atol=1e-05))
                self.assertTrue(
                    np.allclose(self._hands_view.get_fixed_tendon_limit_stiffnesses(), new_limit_stiffness, atol=1e-05)
                )
                self.assertTrue(np.allclose(self._hands_view.get_fixed_tendon_limits(), new_limits, atol=1e-05))
                self.assertTrue(
                    np.allclose(self._hands_view.get_fixed_tendon_rest_lengths(), new_rest_lengths, atol=1e-05)
                )
                self.assertTrue(np.allclose(self._hands_view.get_fixed_tendon_offsets(), new_offsets, atol=1e-05))

    async def test_position_iteration_count_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    new_values = torch.tensor([4], device=device)
                    self._frankas_view.set_solver_position_iteration_counts(new_values, indices=[1])
                    values = self._frankas_view.get_solver_position_iteration_counts(indices=[1])
                else:
                    new_values = torch.tensor([4, 4], device=device)
                    self._frankas_view.set_solver_position_iteration_counts(new_values)
                    values = self._frankas_view.get_solver_position_iteration_counts()
                self.assertTrue(np.allclose(values.cpu().numpy(), new_values.cpu().numpy()))

    async def test_position_iteration_count_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                new_values = np.array([4])
                self._frankas_view.set_solver_position_iteration_counts(new_values, indices=[1])
                values = self._frankas_view.get_solver_position_iteration_counts(indices=[1])
            else:
                new_values = np.array([4, 4])
                self._frankas_view.set_solver_position_iteration_counts(new_values)
                values = self._frankas_view.get_solver_position_iteration_counts()
            self.assertTrue(np.allclose(values, new_values, atol=1e-05))

    async def test_velocity_iteration_count_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    new_values = torch.tensor([4], device=device)
                    self._frankas_view.set_solver_velocity_iteration_counts(new_values, indices=[1])
                    values = self._frankas_view.get_solver_velocity_iteration_counts(indices=[1])
                else:
                    new_values = torch.tensor([4, 4], device=device)
                    self._frankas_view.set_solver_velocity_iteration_counts(new_values)
                    values = self._frankas_view.get_solver_velocity_iteration_counts()
                self.assertTrue(np.allclose(values.cpu().numpy(), new_values.cpu().numpy()))

    async def test_velocity_iteration_count_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                new_values = np.array([4])
                self._frankas_view.set_solver_velocity_iteration_counts(new_values, indices=[1])
                values = self._frankas_view.get_solver_velocity_iteration_counts(indices=[1])
            else:
                new_values = np.array([4, 4])
                self._frankas_view.set_solver_velocity_iteration_counts(new_values)
                values = self._frankas_view.get_solver_velocity_iteration_counts()
            self.assertTrue(np.allclose(values, new_values, atol=1e-05))

    async def test_stabilization_thresholds_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    new_values = torch.tensor([0.01], device=device)
                    self._frankas_view.set_stabilization_thresholds(new_values, indices=[1])
                    values = self._frankas_view.get_stabilization_thresholds(indices=[1])
                else:
                    new_values = torch.tensor([0.01, 0.02], device=device)
                    self._frankas_view.set_stabilization_thresholds(new_values)
                    values = self._frankas_view.get_stabilization_thresholds()
                self.assertTrue(np.allclose(values.cpu().numpy(), new_values.cpu().numpy()))

    async def test_stabilization_thresholds_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                new_values = np.array([0.01])
                self._frankas_view.set_stabilization_thresholds(new_values, indices=[1])
                values = self._frankas_view.get_stabilization_thresholds(indices=[1])
            else:
                new_values = np.array([0.01, 0.02])
                self._frankas_view.set_stabilization_thresholds(new_values)
                values = self._frankas_view.get_stabilization_thresholds()
            self.assertTrue(np.allclose(values, new_values, atol=1e-05))

    async def test_sleep_thresholds_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    new_values = torch.tensor([0.01], device=device)
                    self._frankas_view.set_sleep_thresholds(new_values, indices=[1])
                    values = self._frankas_view.get_sleep_thresholds(indices=[1])
                else:
                    new_values = torch.tensor([0.01, 0.02], device=device)
                    self._frankas_view.set_sleep_thresholds(new_values)
                    values = self._frankas_view.get_sleep_thresholds()
                self.assertTrue(np.allclose(values.cpu().numpy(), new_values.cpu().numpy()))

    async def test_sleep_thresholds_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                new_values = np.array([0.01])
                self._frankas_view.set_sleep_thresholds(new_values, indices=[1])
                values = self._frankas_view.get_sleep_thresholds(indices=[1])
            else:
                new_values = np.array([0.01, 0.02])
                self._frankas_view.set_sleep_thresholds(new_values)
                values = self._frankas_view.get_sleep_thresholds()
            self.assertTrue(np.allclose(values, new_values, atol=1e-05))

    async def test_enabled_self_collisions_torch(self):
        for indexed in INDEXED:
            for device in ["cpu", "cuda:0"]:
                print("index:", indexed, "device:", device)
                await self.setUpWorld(backend="torch", device=device)
                await self.add_frankas(backend="torch")
                if indexed:
                    new_values = torch.tensor([True], device=device)
                    self._frankas_view.set_enabled_self_collisions(new_values, indices=[1])
                    values = self._frankas_view.get_enabled_self_collisions(indices=[1])
                else:
                    new_values = torch.tensor([False, True], device=device)
                    self._frankas_view.set_enabled_self_collisions(new_values)
                    values = self._frankas_view.get_enabled_self_collisions()
                self.assertTrue(np.allclose(values.cpu().numpy(), new_values.cpu().numpy()))

    async def test_enabled_self_collisions_numpy(self):
        for indexed in INDEXED:
            await self.setUpWorld(backend="numpy")
            await self.add_frankas(backend="numpy")
            if indexed:
                new_values = np.array([True])
                self._frankas_view.set_enabled_self_collisions(new_values, indices=[1])
                values = self._frankas_view.get_enabled_self_collisions(indices=[1])
            else:
                new_values = np.array([False, True])
                self._frankas_view.set_enabled_self_collisions(new_values)
                values = self._frankas_view.get_enabled_self_collisions()
            self.assertTrue(np.allclose(values, new_values, atol=1e-05))
