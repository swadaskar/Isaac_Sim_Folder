# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.physics_context.physics_context import PhysicsContext
from omni.isaac.core.materials.physics_material import PhysicsMaterial
from omni.isaac.franka.franka import Franka
from pxr import Gf, Usd, UsdPhysics, PhysxSchema, UsdShade
from .nut_bolt_controller import NutBoltController

import numpy as np

# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


class FrankaNutAndBolt(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        # SCENE GEOMETRY
        # env (group) spacing:
        self._env_spacing = 2.0

        # franka
        self._stool_height = 0.15
        self._franka_position = np.array([0.269, 0.1778, 0.0])  # Gf.Vec3f(0.269, 0.1778, 0.0)
        # table and vibra table:
        self._table_position = np.array([0.5, 0.0, 0.0])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._table_scale = 0.01
        self._tooling_plate_offset = np.array([0.0, 0.0, 0.0])
        self._vibra_table_position_offset = np.array([0.157, -0.1524, 0.0])
        self._vibra_top_offset = np.array([0.0, 0.0, 0.15])
        self._vibra_table_top_to_collider_offset = np.array([0.05, 2.5, -0.59]) * 0.01
        # xyz relative to the vibra table where the nut should be picked up
        self._vibra_table_nut_pickup_pos_offset = np.array([0.124, 0.24, 0.158])

        # nut
        self._nut_height = 0.016
        self._nut_spiral_center_vibra_offset = np.array([-0.04, -0.17, 0.01])
        # randomize initial nut and bolt positions
        self._nut_radius = 0.055
        self._nut_height_delta = 0.03
        self._nut_dist_delta = 0.03
        self._mass_nut = 0.065

        # pipe and bolt parameters
        self._bolt_length = 0.1
        self._bolt_radius = 0.11
        self._pipe_pos_on_table = np.array([0.2032, 0.381, 0.0])
        self._bolt_z_offset_to_pipe = 0.08
        self._gripper_to_nut_offset = np.array([0.0, 0.0, 0.005])
        self._top_of_bolt = (
            np.array([0.0, 0.0, self._bolt_length + (self._nut_height / 2)]) + self._gripper_to_nut_offset
        )

        # randomization
        self._randomize_nut_positions = True
        self._nut_position_noise_minmax = 0.005
        self._rng_seed = 8

        # states
        self._reset_hydra_instancing_on_shutdown = False
        self._time = 0.0
        self._fsm_time = 0.0

        # some global sim options:
        self._time_steps_per_second = 240  # 4.167ms aprx
        self._fsm_update_rate = 60
        self._solverPositionIterations = 4
        self._solverVelocityIterations = 1
        self._solver_type = "TGS"
        self._ik_damping = 0.1

        # setup asset paths:
        self.nucleus_server = get_assets_root_path()
        self.asset_folder = self.nucleus_server + "/Isaac/Samples/Examples/FrankaNutBolt/"
        self.asset_paths = {
            "shop_table": self.asset_folder + "SubUSDs/Shop_Table/Shop_Table.usd",
            "tooling_plate": self.asset_folder + "SubUSDs/Tooling_Plate/Tooling_Plate.usd",
            "nut": self.asset_folder + "SubUSDs/Nut/M20_Nut_Tight_R256_Franka_SI.usd",
            "bolt": self.asset_folder + "SubUSDs/Bolt/M20_Bolt_Tight_R512_Franka_SI.usd",
            "vibra_table_top": self.asset_folder + "SubUSDs/VibrationTable_Top/VibrationTable_Top.usd",
            "vibra_table_bot": self.asset_folder + "SubUSDs/VibrationTable_Base/VibrationTable_Base.usd",
            "vibra_table_collision": self.asset_folder + "SubUSDs/VibrationTable_Top_collision.usd",
            "vibra_table_clamps": self.asset_folder + "SubUSDs/Clamps/Clamps.usd",
            "pipe": self.asset_folder + "SubUSDs/Pipe/Pipe.usd",
        }

        self._num_bolts = 6
        self._num_nuts = 6
        self._sim_dt = 1.0 / self._time_steps_per_second
        self._fsm_update_dt = 1.0 / self._fsm_update_rate

        return

    def setup_scene(self):
        world = self.get_world()

        world.scene.add_default_ground_plane()

        world.scene.add(XFormPrim(prim_path="/World/collisionGroups", name="collision_groups_xform"))
        self._setup_simulation()

        # add_table_assets
        add_reference_to_stage(usd_path=self.asset_paths["shop_table"], prim_path="/World/env/table") # gives asset ref path
        world.scene.add(GeometryPrim(prim_path="/World/env/table", name=f"table_ref_geom", collision=True)) # declares in the world

        add_reference_to_stage(usd_path=self.asset_paths["tooling_plate"], prim_path="/World/env/tooling_plate")
        world.scene.add(GeometryPrim(prim_path="/World/env/tooling_plate", name=f"tooling_plate_geom", collision=True))

        add_reference_to_stage(usd_path=self.asset_paths["pipe"], prim_path="/World/env/pipe")
        world.scene.add(GeometryPrim(prim_path="/World/env/pipe", name=f"pipe_geom", collision=True))

        # add_vibra_table_assets
        add_reference_to_stage(usd_path=self.asset_paths["vibra_table_bot"], prim_path="/World/env/vibra_table_bottom")
        world.scene.add(GeometryPrim(prim_path="/World/env/vibra_table_bottom", name=f"vibra_table_bottom_geom"))

        add_reference_to_stage(
            usd_path=self.asset_paths["vibra_table_clamps"], prim_path="/World/env/vibra_table_clamps"
        )
        world.scene.add(
            GeometryPrim(prim_path="/World/env/vibra_table_clamps", name=f"vibra_table_clamps_geom", collision=True)
        )

        world.scene.add(XFormPrim(prim_path="/World/env/vibra_table", name=f"vibra_table_xform"))
        add_reference_to_stage(usd_path=self.asset_paths["vibra_table_top"], prim_path="/World/env/vibra_table/visual")
        add_reference_to_stage(
            usd_path=self.asset_paths["vibra_table_collision"], prim_path="/World/env/vibra_table/collision"
        )

        world.scene.add(XFormPrim(prim_path="/World/env/vibra_table/visual", name=f"vibra_table_visual_xform"))
        world.scene.add(
            GeometryPrim(
                prim_path="/World/env/vibra_table/collision", name=f"vibra_table_collision_ref_geom", collision=True
            )
        )

        # add_nuts_bolts_assets
        for bolt in range(self._num_bolts):
            add_reference_to_stage(usd_path=self.asset_paths["bolt"], prim_path=f"/World/env/bolt{bolt}")
            world.scene.add(GeometryPrim(prim_path=f"/World/env/bolt{bolt}", name=f"bolt{bolt}_geom"))
        for nut in range(self._num_nuts):
            add_reference_to_stage(usd_path=self.asset_paths["nut"], prim_path=f"/World/env/nut{nut}")
            world.scene.add(GeometryPrim(prim_path=f"/World/env/nut{nut}", name=f"nut{nut}_geom"))

        # add_franka_assets
        world.scene.add(Franka(prim_path="/World/env/franka", name=f"franka"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._rng = np.random.default_rng(self._rng_seed)

        self._world.scene.enable_bounding_boxes_computations()
        await self._setup_materials()
        # next four functions are for setting up the right positions and orientations for all assets
        await self._add_table()
        await self._add_vibra_table()
        await self._add_nuts_and_bolt(add_debug_nut=self._num_nuts == 2)
        await self._add_franka()
        self._controller = NutBoltController(name="nut_bolt_controller", franka=self._franka)
        self._franka.gripper.open()
        self._rbApi2 = UsdPhysics.RigidBodyAPI.Apply(self._vibra_table_xform.prim.GetPrim())
        self._world.add_physics_callback(f"sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        if self._controller.is_paused():
            if self._controller._i >= min(self._num_nuts, self._num_bolts):
                self._rbApi2.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                return
            self._controller.reset(self._franka)

        if self._controller._i < min(self._num_nuts, self._num_bolts):
            initial_position = self._vibra_table_nut_pickup_pos_offset + self._vibra_table_position
            self._bolt_geom = self._world.scene.get_object(f"bolt{self._controller._i}_geom")

            finger_pos = self._franka.get_joint_positions()[-2:]
            positive_x_offset = finger_pos[1] - finger_pos[0]

            bolt_position, _ = self._bolt_geom.get_world_pose()
            placing_position = bolt_position + self._top_of_bolt
            _vibra_table_transforms = self._controller.forward(
                initial_picking_position=initial_position,
                bolt_top=placing_position,
                gripper_to_nut_offset=self._gripper_to_nut_offset,
                x_offset=positive_x_offset,
            )

        self._rbApi2.CreateVelocityAttr().Set(
            Gf.Vec3f(_vibra_table_transforms[0], _vibra_table_transforms[1], _vibra_table_transforms[2])
        )
        return

    async def _setup_materials(self):
        self._bolt_physics_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/BoltMaterial",
            name="bolt_material_physics",
            static_friction=0.2,
            dynamic_friction=0.2,
        )
        self._nut_physics_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/NutMaterial",
            name="nut_material_physics",
            static_friction=0.2,
            dynamic_friction=0.2,
        )
        self._vibra_table_physics_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/VibraTableMaterial",
            name="vibra_table_material_physics",
            static_friction=0.3,
            dynamic_friction=0.3,
        )
        self._franka_finger_physics_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/FrankaFingerMaterial",
            name="franka_finger_material_physics",
            static_friction=0.7,
            dynamic_friction=0.7,
        )
        await self._world.reset_async()

    async def _add_table(self):
        ##shop_table
        self._table_ref_geom = self._world.scene.get_object(f"table_ref_geom")
        self._table_ref_geom.set_local_scale(np.array([self._table_scale]))
        self._table_ref_geom.set_world_pose(position=self._table_position)
        self._table_ref_geom.set_default_state(position=self._table_position)
        lb = self._world.scene.compute_object_AABB(name=f"table_ref_geom")
        zmin = lb[0][2]
        zmax = lb[1][2]
        self._table_position[2] = -zmin
        self._table_height = zmax
        self._table_ref_geom.set_collision_approximation("none")
        self._convexIncludeRel.AddTarget(self._table_ref_geom.prim_path)

        ##tooling_plate
        self._tooling_plate_geom = self._world.scene.get_object(f"tooling_plate_geom")
        self._tooling_plate_geom.set_local_scale(np.array([self._table_scale]))
        lb = self._world.scene.compute_object_AABB(name=f"tooling_plate_geom")
        zmin = lb[0][2]
        zmax = lb[1][2]
        tooling_transform = self._tooling_plate_offset
        tooling_transform[2] = -zmin + self._table_height
        tooling_transform = tooling_transform + self._table_position
        self._tooling_plate_geom.set_world_pose(position=tooling_transform)
        self._tooling_plate_geom.set_default_state(position=tooling_transform)
        self._tooling_plate_geom.set_collision_approximation("boundingCube")
        self._table_height += zmax - zmin
        self._convexIncludeRel.AddTarget(self._tooling_plate_geom.prim_path)

        ##pipe
        self._pipe_geom = self._world.scene.get_object(f"pipe_geom")
        self._pipe_geom.set_local_scale(np.array([self._table_scale]))
        lb = self._world.scene.compute_object_AABB(name=f"pipe_geom")
        zmin = lb[0][2]
        zmax = lb[1][2]
        self._pipe_height = zmax - zmin
        pipe_transform = self._pipe_pos_on_table
        pipe_transform[2] = -zmin + self._table_height
        pipe_transform = pipe_transform + self._table_position
        self._pipe_geom.set_world_pose(position=pipe_transform, orientation=np.array([0, 0, 0, 1]))
        self._pipe_geom.set_default_state(position=pipe_transform, orientation=np.array([0, 0, 0, 1]))
        self._pipe_geom.set_collision_approximation("none")
        self._convexIncludeRel.AddTarget(self._pipe_geom.prim_path)
        await self._world.reset_async()

    async def _add_vibra_table(self):
        self._vibra_table_bottom_geom = self._world.scene.get_object(f"vibra_table_bottom_geom")
        self._vibra_table_bottom_geom.set_local_scale(np.array([self._table_scale]))
        lb = self._world.scene.compute_object_AABB(name=f"vibra_table_bottom_geom")
        zmin = lb[0][2]
        bot_part_pos = self._vibra_table_position_offset
        bot_part_pos[2] = -zmin + self._table_height
        bot_part_pos = bot_part_pos + self._table_position
        self._vibra_table_bottom_geom.set_world_pose(position=bot_part_pos)
        self._vibra_table_bottom_geom.set_default_state(position=bot_part_pos)
        self._vibra_table_bottom_geom.set_collision_approximation("none")
        self._convexIncludeRel.AddTarget(self._vibra_table_bottom_geom.prim_path)

        # clamps
        self._vibra_table_clamps_geom = self._world.scene.get_object(f"vibra_table_clamps_geom")
        self._vibra_table_clamps_geom.set_collision_approximation("none")
        self._convexIncludeRel.AddTarget(self._vibra_table_clamps_geom.prim_path)

        # vibra_table
        self._vibra_table_xform = self._world.scene.get_object(f"vibra_table_xform")
        self._vibra_table_position = bot_part_pos
        vibra_kinematic_prim = self._vibra_table_xform.prim
        rbApi = UsdPhysics.RigidBodyAPI.Apply(vibra_kinematic_prim.GetPrim())
        rbApi.CreateRigidBodyEnabledAttr(True)
        rbApi.CreateKinematicEnabledAttr(True)

        # visual
        self._vibra_table_visual_xform = self._world.scene.get_object(f"vibra_table_visual_xform")
        self._vibra_table_visual_xform.set_world_pose(position=self._vibra_top_offset)
        self._vibra_table_visual_xform.set_default_state(position=self._vibra_top_offset)
        self._vibra_table_visual_xform.set_local_scale(np.array([self._table_scale]))

        # collision
        self._vibra_table_collision_ref_geom = self._world.scene.get_object(f"vibra_table_collision_ref_geom")
        offset = self._vibra_top_offset + self._vibra_table_top_to_collider_offset
        self._vibra_table_collision_ref_geom.set_local_scale(np.array([1.0]))
        self._vibra_table_collision_ref_geom.set_world_pose(position=offset)
        self._vibra_table_collision_ref_geom.set_default_state(position=offset)
        self._vibra_table_collision_ref_geom.apply_physics_material(self._vibra_table_physics_material)
        self._convexIncludeRel.AddTarget(self._vibra_table_collision_ref_geom.prim_path)
        self._vibra_table_collision_ref_geom.set_collision_approximation("convexHull")
        vibra_kinematic_prim.SetInstanceable(True)
        self._vibra_table_xform.set_world_pose(position=self._vibra_table_position, orientation=np.array([0, 0, 0, 1]))
        self._vibra_table_xform.set_default_state(
            position=self._vibra_table_position, orientation=np.array([0, 0, 0, 1])
        )
        self._vibra_table_visual_xform.set_default_state(
            position=self._vibra_table_visual_xform.get_world_pose()[0],
            orientation=self._vibra_table_visual_xform.get_world_pose()[1],
        )
        self._vibra_table_collision_ref_geom.set_default_state(
            position=self._vibra_table_collision_ref_geom.get_world_pose()[0],
            orientation=self._vibra_table_collision_ref_geom.get_world_pose()[1],
        )
        await self._world.reset_async()

    async def _add_nuts_and_bolt(self, add_debug_nut=False):
        angle_delta = np.pi * 2.0 / self._num_bolts
        for j in range(self._num_bolts):
            self._bolt_geom = self._world.scene.get_object(f"bolt{j}_geom")
            self._bolt_geom.prim.SetInstanceable(True)
            bolt_pos = np.array(self._pipe_pos_on_table) + self._table_position
            bolt_pos[0] += np.cos(j * angle_delta) * self._bolt_radius
            bolt_pos[1] += np.sin(j * angle_delta) * self._bolt_radius
            bolt_pos[2] = self._bolt_z_offset_to_pipe + self._table_height
            self._bolt_geom.set_world_pose(position=bolt_pos)
            self._bolt_geom.set_default_state(position=bolt_pos)
            self._boltMeshIncludeRel.AddTarget(self._bolt_geom.prim_path)
            self._bolt_geom.apply_physics_material(self._bolt_physics_material)

        await self._generate_nut_initial_poses()
        for nut_idx in range(self._num_nuts):
            nut_pos = self._nut_init_poses[nut_idx, :3].copy()
            if add_debug_nut and nut_idx == 0:
                nut_pos[0] = 0.78
                nut_pos[1] = self._vibra_table_nut_pickup_pos_offset[1] + self._vibra_table_position[1]  # 0.0264
            if add_debug_nut and nut_idx == 1:
                nut_pos[0] = 0.78
                nut_pos[1] = 0.0264 - 0.04

            self._nut_geom = self._world.scene.get_object(f"nut{nut_idx}_geom")
            self._nut_geom.prim.SetInstanceable(True)
            self._nut_geom.set_world_pose(position=np.array(nut_pos.tolist()))
            self._nut_geom.set_default_state(position=np.array(nut_pos.tolist()))
            physxRBAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(self._nut_geom.prim)
            physxRBAPI.CreateSolverPositionIterationCountAttr().Set(self._solverPositionIterations)
            physxRBAPI.CreateSolverVelocityIterationCountAttr().Set(self._solverVelocityIterations)
            self._nut_geom.apply_physics_material(self._nut_physics_material)
            self._convexIncludeRel.AddTarget(self._nut_geom.prim_path + "/M20_Nut_Tight_Convex")
            self._nutMeshIncludeRel.AddTarget(self._nut_geom.prim_path + "/M20_Nut_Tight_SDF")
            rbApi3 = UsdPhysics.RigidBodyAPI.Apply(self._nut_geom.prim.GetPrim())
            rbApi3.CreateRigidBodyEnabledAttr(True)
            physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(self._nut_geom.prim.GetPrim())
            physxAPI.CreateSleepThresholdAttr().Set(0.0)
            massAPI = UsdPhysics.MassAPI.Apply(self._nut_geom.prim.GetPrim())
            massAPI.CreateMassAttr().Set(self._mass_nut)
        await self._world.reset_async()

    async def _generate_nut_initial_poses(self):
        self._nut_init_poses = np.zeros((self._num_nuts, 7), dtype=np.float32)
        self._nut_init_poses[:, -1] = 1  # quat to identity
        nut_spiral_center = self._vibra_table_position + self._nut_spiral_center_vibra_offset
        nut_spiral_center += self._vibra_top_offset
        for nut_idx in range(self._num_nuts):
            self._nut_init_poses[nut_idx, :3] = np.array(nut_spiral_center)
            self._nut_init_poses[nut_idx, 0] += self._nut_radius * np.sin(
                np.pi / 3.0 * nut_idx
            ) + self._nut_dist_delta * (nut_idx // 6)
            self._nut_init_poses[nut_idx, 1] += self._nut_radius * np.cos(
                np.pi / 3.0 * nut_idx
            ) + self._nut_dist_delta * (nut_idx // 6)
            self._nut_init_poses[nut_idx, 2] += self._nut_height_delta * (nut_idx // 6)
            if self._randomize_nut_positions:
                self._nut_init_poses[nut_idx, 0] += self._rng.uniform(
                    -self._nut_position_noise_minmax, self._nut_position_noise_minmax
                )
                self._nut_init_poses[nut_idx, 1] += self._rng.uniform(
                    -self._nut_position_noise_minmax, self._nut_position_noise_minmax
                )
        await self._world.reset_async()

    async def _add_franka(self):
        self._franka = self._world.scene.get_object(f"franka")
        franka_pos = np.array(self._franka_position)
        franka_pos[2] = franka_pos[2] + self._table_height
        self._franka.set_world_pose(position=franka_pos)
        self._franka.set_default_state(position=franka_pos)
        self._franka.gripper.open()

        self._convexIncludeRel.AddTarget(self._franka.prim_path + "/panda_leftfinger")
        self._convexIncludeRel.AddTarget(self._franka.prim_path + "/panda_rightfinger")

        franka_left_finger = self._world.stage.GetPrimAtPath(
            "/World/env/franka/panda_leftfinger/geometry/panda_leftfinger"
        )
        x = UsdShade.MaterialBindingAPI.Apply(franka_left_finger)
        x.Bind(
            self._franka_finger_physics_material.material,
            bindingStrength="weakerThanDescendants",
            materialPurpose="physics",
        )
        franka_right_finger = self._world.stage.GetPrimAtPath(
            "/World/env/franka/panda_rightfinger/geometry/panda_rightfinger"
        )
        x2 = UsdShade.MaterialBindingAPI.Apply(franka_right_finger)
        x2.Bind(
            self._franka_finger_physics_material.material,
            bindingStrength="weakerThanDescendants",
            materialPurpose="physics",
        )
        await self._world.reset_async()

    def _setup_simulation(self):
        self._scene = PhysicsContext()
        self._scene.set_solver_type(self._solver_type)
        self._scene.set_broadphase_type("GPU")
        self._scene.enable_gpu_dynamics(flag=True)
        self._scene.set_friction_offset_threshold(0.01)
        self._scene.set_friction_correlation_distance(0.0005)
        self._scene.set_gpu_total_aggregate_pairs_capacity(10 * 1024)
        self._scene.set_gpu_found_lost_pairs_capacity(10 * 1024)
        self._scene.set_gpu_heap_capacity(64 * 1024 * 1024)
        self._scene.set_gpu_found_lost_aggregate_pairs_capacity(10 * 1024)

        self._meshCollisionGroup = UsdPhysics.CollisionGroup.Define(
            self._world.scene.stage, "/World/collisionGroups/meshColliders"
        )
        collectionAPI = Usd.CollectionAPI.ApplyCollection(self._meshCollisionGroup.GetPrim(), "colliders")
        self._nutMeshIncludeRel = collectionAPI.CreateIncludesRel()
        self._convexCollisionGroup = UsdPhysics.CollisionGroup.Define(
            self._world.scene.stage, "/World/collisionGroups/convexColliders"
        )
        collectionAPI = Usd.CollectionAPI.ApplyCollection(self._convexCollisionGroup.GetPrim(), "colliders")
        self._convexIncludeRel = collectionAPI.CreateIncludesRel()
        self._boltCollisionGroup = UsdPhysics.CollisionGroup.Define(
            self._world.scene.stage, "/World/collisionGroups/boltColliders"
        )
        collectionAPI = Usd.CollectionAPI.ApplyCollection(self._boltCollisionGroup.GetPrim(), "colliders")
        self._boltMeshIncludeRel = collectionAPI.CreateIncludesRel()

        # invert group logic so only groups that filter each-other will collide:
        self._scene.set_invert_collision_group_filter(True)

        # # the SDF mesh collider nuts should only collide with the bolts
        filteredRel = self._meshCollisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget("/World/collisionGroups/boltColliders")

        # # the convex hull nuts should collide with each other, the vibra table, and the grippers
        filteredRel = self._convexCollisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget("/World/collisionGroups/convexColliders")

        # # the SDF mesh bolt only collides with the SDF mesh nut colliders
        filteredRel = self._boltCollisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget("/World/collisionGroups/meshColliders")

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        self._controller._vibraSM.reset()
        self._controller._vibraSM._i = 0
        self._controller.reset(franka=self._franka)
        self._controller._i = self._controller._vibraSM._i
        self._franka.gripper.open()
        self._controller._vibraSM.stop_feed_after_delay(delay_sec=5.0)
        await self._world.play_async()
        return

    def world_cleanup(self):
        self._controller = None
        return
