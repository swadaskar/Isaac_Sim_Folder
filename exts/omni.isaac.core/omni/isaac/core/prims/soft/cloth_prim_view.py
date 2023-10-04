# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import torch
import numpy as np
from typing import Optional, Tuple, Union

# omniverse
import carb
import omni.kit.app
from pxr import Vt, UsdPhysics, PhysxSchema

# isaac-core
from omni.isaac.core.prims import XFormPrimView
from omni.isaac.core.utils.types import XFormPrimViewState
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.materials import ParticleMaterial


class ClothPrimView(XFormPrimView):
    """The view class for cloth prims."""

    def __init__(
        self,
        prim_paths_expr: str,
        particle_systems: Union[np.ndarray, torch.Tensor] = None,
        particle_materials: Optional[Union[np.ndarray, torch.Tensor]] = None,
        name: str = "cloth_prim_view",
        reset_xform_properties: bool = True,
        positions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor]] = None,
        particle_masses: Optional[Union[np.ndarray, torch.Tensor]] = None,
        pressures: Optional[Union[np.ndarray, torch.Tensor]] = None,
        particle_groups: Optional[Union[np.ndarray, torch.Tensor]] = None,
        self_collisions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        self_collision_filters: Optional[Union[np.ndarray, torch.Tensor]] = None,
        stretch_stiffnesses: Optional[Union[np.ndarray, torch.Tensor]] = None,
        bend_stiffnesses: Optional[Union[np.ndarray, torch.Tensor]] = None,
        shear_stiffnesses: Optional[Union[np.ndarray, torch.Tensor]] = None,
        spring_dampings: Optional[Union[np.ndarray, torch.Tensor]] = None,
    ):
        """
        Provides high level functions to deal with cloths (1 or more cloths) 
        as well as its attributes/ properties. This object wraps all matching cloths found at the regex provided at the prim_paths_expr.
        This object wraps all matching Cloth Prims found at the regex provided at the prim_paths_expr.

        Note: - if the prim does not already have a rigid body api applied to it before init, it will apply it.
        Args:
            prim_paths_expr(str): Prim paths regex to encapsulate all prims that match it.
            name(str): Shortname to be used as a key by Scene class.
            positions: (Union[np.ndarray, torch.Tensor], optional): Default positions in the world frame of the prim. shape is (N, 3).
            translations: (Union[np.ndarray, torch.Tensor], optional): Default translations in the local frame of the
                                                                        prims (with respect to its parent prims). shape is (N, 3).
            orientations: (Union[np.ndarray, torch.Tensor], optional): Default quaternion orientations in the world/
                                                                        local frame of the prim (depends if translation or position is specified).
                                                                        quaternion is scalar-first (w, x, y, z). shape is (N, 4).
            scales: (Union[np.ndarray, torch.Tensor], optional): Local scales to be applied to the prim's dimensions. shape is (N, 3).
            visibilities: (Union[np.ndarray, torch.Tensor], optional): Set to false for an invisible prim in the stage while rendering. shape is (N,).
            particle_masses (Union[np.ndarray, torch.Tensor], optional): particle masses to be applied to each prim.
            pressures (Union[np.ndarray, torch.Tensor], optional): pressures to be applied to each prim. if > 0, a particle 
                                                                cloth has an additional pressure constraint that provides
                                                                inflatable (i.e. balloon-like) dynamics. The pressure 
                                                                times the rest volume defines the volume the inflatable 
                                                                tries to match. Pressure only works well for closed or 
                                                                approximately closed meshes, range: [0, inf), units: dimensionless
            particle_groups (Union[np.ndarray, torch.Tensor], optional): group Id of the particles of each prim, range: [0, 2^20)
            self_collisions (Union[np.ndarray, torch.Tensor], optional): enable self collision of the particles of each prim. 
            self_collision_filters (Union[np.ndarray, torch.Tensor], optional): whether the simulation should filter 
                                                                                particle-particle collisions based on the 
                                                                                rest position distances of each prim. shape is (N,).
            stretch_stiffnesses (Union[np.ndarray, torch.Tensor], optional): represents the stretch spring stiffnesses for 
                                                                            linear springs placed between particles to counteract 
                                                                            stretching, shape is (N,). range: [0, inf), units: 
                                                                            force / distance = mass / second / second
            bend_stiffnesses (Union[np.ndarray, torch.Tensor], optional): represents the spring bend stiffnesses for linear
                                                                         springs placed in a way to counteract bending,  shape is (N,).
                                                                         range: [0, inf), units: force / distance = mass / second / second
            shear_stiffnesses (Union[np.ndarray, torch.Tensor], optional): represents the shear stiffnesses for linear 
                                                                            springs placed in a way to counteract shear,  shape is (N,). 
                                                                            range: [0, inf), units: force / distance = mass / second / second
            spring_dampings (Union[np.ndarray, torch.Tensor], optional): damping on cloth spring constraints. Applies to all constraints
                                                                        parameterized by stiffness attributes, range: [0, inf),  shape is (N,).
                                                                        units: force * second / distance = mass / second
        """

        self._physics_view = None
        self._device = None
        self._name = name
        XFormPrimView.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
        )
        self._cloth_auto_apis = [None] * self._count
        self._cloth_apis = [None] * self._count
        self._mass_apis = [None] * self._count

        if particle_masses is not None:
            self.set_particle_masses(particle_masses)
        if pressures is not None:
            self.set_pressures(pressures)
        if particle_groups is not None:
            self.set_particle_groups(particle_groups)
        if self_collision_filters is not None:
            self.set_self_collision_filters(self_collision_filters)
        if self_collisions is not None:
            self.set_self_collisions(self_collisions)
        if stretch_stiffnesses is not None:
            if len(stretch_stiffnesses.tolist()) == self._count:
                self.set_cloths_stretch_stiffnesses(stretch_stiffnesses)
            else:
                self.set_stretch_stiffnesses(stretch_stiffnesses)
        if bend_stiffnesses is not None:
            if len(bend_stiffnesses.tolist()) == self._count:
                self.set_cloths_bend_stiffnesses(bend_stiffnesses)
            else:
                self.set_bend_stiffnesses(bend_stiffnesses)
        if shear_stiffnesses is not None:
            if len(shear_stiffnesses.tolist()) == self._count:
                self.set_cloths_shear_stiffnesses(shear_stiffnesses)
            else:
                self.set_shear_stiffnesses(shear_stiffnesses)
        if spring_dampings is not None:
            if len(spring_dampings.tolist()) == self._count:
                self.set_cloths_dampings(spring_dampings)
            else:
                self.set_spring_dampings(spring_dampings)

        timeline = omni.timeline.get_timeline_interface()
        self._invalidate_physics_handle_event = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._invalidate_physics_handle_callback
        )

    """
    Properties.
    """

    @property
    def count(self) -> int:
        """
        Returns:
            int: cloth counts.
        """
        return self._count

    @property
    def max_springs_per_cloth(self) -> int:
        """
        Returns:
            int: maximum number of springs per cloth.
        """
        return self._max_springs_per_cloth

    @property
    def max_particles_per_cloth(self) -> int:
        """
        Returns:
            int: maximum number of particles per cloth.
        """
        return self._max_particles_per_cloth

    def is_physics_handle_valid(self) -> bool:
        """
        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a rigid body view in physX.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """

        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info("initializing view for {}".format(self._name))
        self._physics_sim_view = physics_sim_view
        self._physics_view = self._physics_sim_view.create_particle_cloth_view(
            self._regex_prim_paths.replace(".*", "*")
        )
        self._count = self._physics_view.count
        self._max_springs_per_cloth = self._physics_view.max_springs_per_cloth
        self._max_particles_per_cloth = self._physics_view.max_particles_per_cloth
        carb.log_info("Cloth Prim View Device: {}".format(self._device))
        return

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
        return

    def _apply_cloth_auto_api(self, index):
        if self._cloth_auto_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxAutoParticleClothAPI):
                cloth_api = PhysxSchema.PhysxAutoParticleClothAPI(self._prims[index])
            else:
                cloth_api = PhysxSchema.PhysxAutoParticleClothAPI.Apply(self._prims[index])
            self._cloth_auto_apis[index] = cloth_api

    def _apply_cloth_api(self, index):
        if self._cloth_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxParticleClothAPI):
                cloth_api = PhysxSchema.PhysxParticleClothAPI(self._prims[index])
            else:
                cloth_api = PhysxSchema.PhysxParticleClothAPI.Apply(self._prims[index])
            self._cloth_apis[index] = cloth_api

    def set_world_positions(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle world positions for the cloths indicated by the indices.

        Args:
            positions (Union[np.ndarray, torch.Tensor]): particle positions with the shape 
                                                                                (M, max_particles_per_cloth, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_positions = self._backend_utils.move_to_gpu(positions)
            current_positions = self.get_world_positions(clone=False)
            current_positions[indices] = new_positions
            self._physics_view.set_positions(current_positions, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_cloth_auto_api(i.tolist())
                points = self._prims[i.tolist()].GetAttribute("points").Get()
                if points is None:
                    raise Exception(f"The prim {self.name} does not have points attribute.")
                self._prims[i.tolist()].GetAttribute("points").Set(positions[idx_count].tolist())
                idx_count += 1
        return

    def get_world_positions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the particle world positions for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: position tensor with shape (M, max_particles_per_cloth, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            positions = self._physics_view.get_positions()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return positions[indices].reshape(len(indices), -1, 3)
            else:
                return self._backend_utils.clone_tensor(
                    positions[indices].reshape(len(indices), -1, 3), device=self._device
                )
        else:
            positions = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_particles_per_cloth, 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_cloth_auto_api(i.tolist())
                points = self._prims[i.tolist()].GetAttribute("points").Get()
                if points is None:
                    raise Exception(f"The prim {self.name} does not have points attribute.")
                positions[write_idx] = self._backend_utils.create_tensor_from_list(
                    points, dtype="float32", device=self._device
                ).view(self.max_particles_per_cloth, 3)
                write_idx += 1
            return positions

    def set_velocities(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle velocities for the cloths indicated by the indices.

        Args:
            velocities (Union[np.ndarray, torch.Tensor]): particle velocities with the shape 
                                                                                (M, max_particles_per_cloth, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_velocities = self._backend_utils.move_to_gpu(velocities)
            current_velocities = self.get_velocities(clone=False)
            current_velocities[indices] = new_velocities
            self._physics_view.set_velocities(current_velocities, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_cloth_auto_api(i.tolist())
                point_velocities = self._prims[i.tolist()].GetAttribute("velocities").Get()
                if point_velocities is None:
                    raise Exception(f"The prim {self.name} does not have velocities attribute.")
                self._prims[i.tolist()].GetAttribute("velocities").Set(velocities[idx_count].tolist())
                idx_count += 1

    def get_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the particle velocities for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: velocity tensor with shape (M, max_particles_per_cloth, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            velocities = self._physics_view.get_velocities()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return velocities[indices].reshape(len(indices), -1, 3)
            else:
                return self._backend_utils.clone_tensor(
                    velocities[indices].reshape(len(indices), -1, 3), device=self._device
                )
        else:
            velocities = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_particles_per_cloth, 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_cloth_auto_api(i.tolist())
                point_velocities = self._prims[i.tolist()].GetAttribute("velocities").Get()
                if point_velocities is None:
                    raise Exception(f"The prim {self.name} does not have velocities attribute.")
                velocities[write_idx] = self._backend_utils.create_tensor_from_list(
                    point_velocities, dtype="float32", device=self._device
                ).view(self.max_particles_per_cloth, 3)
                write_idx += 1
            return velocities

    def set_particle_masses(
        self,
        masses: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle masses for the cloths indicated by the indices.

        Args:
            masses (Union[np.ndarray, torch.Tensor]): cloth particle masses with the shape 
                                                                                (M, max_particles_per_cloth, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_masses = self._backend_utils.move_to_gpu(masses)
            current_masses = self.get_masses(clone=False)
            current_masses[indices] = new_masses
            self._physics_view.set_masses(current_masses, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if self._mass_apis[i.tolist()] is None:
                    if self._prims[i.tolist()].HasAPI(UsdPhysics.MassAPI):
                        mass_api = UsdPhysics.MassAPI(self._prims[i.tolist()])
                    else:
                        mass_api = UsdPhysics.MassAPI.Apply(self._prims[i.tolist()])
                    self._mass_apis[i.tolist()] = mass_api
                mass_api.GetMassAttr().Set(sum(masses[idx_count].tolist()))
                idx_count += 1

    def get_particle_masses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the particle masses for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: mass tensor with shape (M, max_particles_per_cloth)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            masses = self._physics_view.get_masses()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return masses[indices]
            else:
                return self._backend_utils.clone_tensor(masses[indices], device=self._device)
        else:
            values = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_particles_per_cloth], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                if "physics:mass" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"physics:mass is not defined on the cloth prim: {self.name}.")
                else:
                    values[write_idx, :] = (
                        self._mass_apis[i.tolist()].GetMassAttr().Get() / self.max_particles_per_cloth
                    )
                write_idx += 1
            return values

    def set_stretch_stiffnesses(
        self,
        stiffness: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the spring stretch stiffness values for springs within the cloths indicated by the indices.

        Args:
            stiffness (Union[np.ndarray, torch.Tensor]): cloth spring stiffness with the shape  (M, max_springs_per_cloth).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_stiffnesses = self._backend_utils.move_to_gpu(stiffness)
            current_stiffnesses = self.get_stretch_stiffnesses(clone=False)
            current_stiffnesses[indices] = new_stiffnesses
            self._physics_view.set_spring_stiffness(current_stiffnesses, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if stiffness[idx_count].any() < 0:
                    carb.log_error("The range of stiffness is [0. inf). Incorrect value for index ", idx_count)
                self._apply_cloth_api(i.tolist())
                if "physxParticle:springStiffnesses" not in self._prims[i.tolist()].GetPropertyNames():
                    self._cloth_apis[i.tolist()].CreateSpringStiffnessesAttr().Set(
                        Vt.FloatArray(stiffness[idx_count].tolist())
                    )
                else:
                    self._cloth_apis[i.tolist()].GetSpringStiffnessesAttr().Set(
                        Vt.FloatArray(stiffness[idx_count].tolist())
                    )
                idx_count += 1

    def get_stretch_stiffnesses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the spring stretch stiffness for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: stiffness tensor with shape (M, max_springs_per_cloth)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            stiffness = self._physics_view.get_spring_stiffness()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return stiffness[indices]
            else:
                return self._backend_utils.clone_tensor(stiffness[indices], device=self._device)
        else:
            stiffnesses = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_springs_per_cloth], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_cloth_api(i.tolist())
                if "physxParticle:springStiffnesses" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"Stretch stiffness is not defined on the cloth prim: {self.name}.")
                else:
                    stiffnesses[write_idx] = self._backend_utils.create_tensor_from_list(
                        self._cloth_apis[i.tolist()].GetSpringStiffnessesAttr().Get(), dtype="float32"
                    )
                write_idx += 1
            return stiffnesses

    def set_spring_dampings(
        self,
        damping: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the spring damping for the cloths indicated by the indices.

        Args:
            damping (Union[np.ndarray, torch.Tensor]): cloth spring damping with the shape 
                                                                            (M, max_springs_per_cloth).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_damping = self._backend_utils.move_to_gpu(damping)
            current_damping = self.get_spring_dampings(clone=False)
            current_damping[indices] = new_damping
            self._physics_view.set_spring_damping(current_damping, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if damping[idx_count].any() < 0:
                    carb.log_error("The range of damping is [0. inf). Incorrect value for index ", idx_count)
                self._apply_cloth_api(i.tolist())
                if "physxParticle:springDampings" not in self._prims[i.tolist()].GetPropertyNames():
                    self._cloth_apis[i.tolist()].CreateSpringDampingsAttr().Set(
                        Vt.FloatArray(damping[idx_count].tolist())
                    )
                else:
                    self._cloth_apis[i.tolist()].GetSpringDampingsAttr().Set(Vt.FloatArray(damping[idx_count].tolist()))
                idx_count += 1

    def get_spring_dampings(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the spring damping for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: damping tensor with shape (M, max_springs_per_cloth)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            damping = self._physics_view.get_spring_damping()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return damping[indices]
            else:
                return self._backend_utils.clone_tensor(damping[indices], device=self._device)
        else:
            dampings = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_springs_per_cloth], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_cloth_api(i.tolist())
                if "physxParticle:springDampings" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"Stretch damping is not defined on the cloth prim: {self.name}.")
                else:
                    dampings[write_idx] = self._backend_utils.create_tensor_from_list(
                        self._cloth_apis[i.tolist()].GetSpringDampingsAttr().Get(), dtype="float32"
                    )
                write_idx += 1
            return dampings

    def set_pressures(
        self,
        pressures: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the pressures of the cloths indicated by the indices.

        Args:
            pressures (Union[np.ndarray, torch.Tensor]): cloths pressure with shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:pressure" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_apis[i.tolist()].CreatePressureAttr().Set(pressures[idx_count].tolist())
            else:
                self._cloth_apis[i.tolist()].GetPressureAttr().Set(pressures[idx_count].tolist())
            idx_count += 1

    def set_self_collision_filters(
        self,
        self_collision_filters: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the self collision filters for the cloths indicated by the indices.

        Args:
            self_collision_filters (Union[np.ndarray, torch.Tensor]): self collision filters with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:selfCollisionFilter" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_apis[i.tolist()].CreateSelfCollisionFilterAttr().Set(
                    self_collision_filters[idx_count].tolist()
                )
            else:
                self._cloth_apis[i.tolist()].GetSelfCollisionFilterAttr().Set(
                    self_collision_filters[idx_count].tolist()
                )
            idx_count += 1

    def set_self_collisions(
        self,
        self_collisions: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the self collision flags for the cloths indicated by the indices.

        Args:
            self_collisions (Union[np.ndarray, torch.Tensor]): self collision flag with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:selfCollision" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_apis[i.tolist()].CreateSelfCollisionAttr().Set(self_collisions[idx_count].tolist())
            else:
                self._cloth_apis[i.tolist()].GetSelfCollisionAttr().Set(self_collisions[idx_count].tolist())
            idx_count += 1

    def set_particle_groups(
        self,
        particle_groups: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle group of the cloths indicated by the indices.

        Args:
            particle_groups (Union[np.ndarray, torch.Tensor]): particle group with shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:particleGroup" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_apis[i.tolist()].CreateParticleGroupAttr().Set(particle_groups[idx_count].tolist())
            else:
                self._cloth_apis[i.tolist()].GetParticleGroupAttr().Set(particle_groups[idx_count].tolist())
            idx_count += 1

    def set_cloths_dampings(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets a single value of damping to all the springs within cloths indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): cloth spring damping with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error("The range of damping is [0. inf). Incorrect value for index ", idx_count)
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springDamping" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_auto_apis[i.tolist()].CreateSpringDampingAttr().Set(values[idx_count].tolist())
            else:
                self._cloth_auto_apis[i.tolist()].GetSpringDampingAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def set_cloths_stretch_stiffnesses(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets a single value of stretch stiffnesses to all the springs within cloths indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): cloth spring stretch stiffness values with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error("The range of stretch stiffness is [0. inf). Incorrect value for index ", idx_count)
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springStretchStiffness" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_auto_apis[i.tolist()].CreateSpringStretchStiffnessAttr().Set(values[idx_count].tolist())
            else:
                self._cloth_auto_apis[i.tolist()].GetSpringStretchStiffnessAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_cloths_bend_stiffnesses(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets a single value of bend stiffnesses to all the springs within cloths indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): cloth spring bend stiffness values with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error("The range of bend stiffness is [0. inf). Incorrect value for index ", idx_count)
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springBendStiffness" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_auto_apis[i.tolist()].CreateSpringBendStiffnessAttr().Set(values[idx_count].tolist())
            else:
                self._cloth_auto_apis[i.tolist()].GetSpringBendStiffnessAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_cloths_shear_stiffnesses(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets a single value of shear stiffnesses to all the springs within cloths indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): cloth spring shear stiffness values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error("The range of shear stiffness is [0. inf). Incorrect value for index ", idx_count)
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springShearStiffness" not in self._prims[i.tolist()].GetPropertyNames():
                self._cloth_auto_apis[i.tolist()].CreateSpringShearStiffnessAttr().Set(values[idx_count].tolist())
            else:
                self._cloth_auto_apis[i.tolist()].GetSpringShearStiffnessAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def get_cloths_dampings(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the value of damping set for all the springs within cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: damping tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springDamping" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"damping is not defined on the cloth prim: {self.name}.")
            else:
                values[write_idx] = self._cloth_auto_apis[i.tolist()].GetSpringDampingAttr().Get()
            write_idx += 1
        return values

    def get_cloths_stretch_stiffnesses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the value of stretch stiffness set to all the springs within cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: stretch stiffness tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springStretchStiffness" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"Stretch stiffness is not defined on the cloth prim: {self.name}.")
            else:
                values[write_idx] = self._cloth_auto_apis[i.tolist()].GetSpringStretchStiffnessAttr().Get()
            write_idx += 1
        return values

    def get_cloths_bend_stiffnesses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the value of bend stiffness set to all the springs within cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: bend stiffness tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springBendStiffness" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"bend stiffness is not defined on the cloth prim: {self.name}.")
            else:
                values[write_idx] = self._cloth_auto_apis[i.tolist()].GetSpringBendStiffnessAttr().Get()
            write_idx += 1
        return values

    def get_cloths_shear_stiffnesses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the value of shear stiffness set to all the springs within cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: shear stiffness tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_cloth_auto_api(i.tolist())
            if "physxAutoParticleCloth:springShearStiffness" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"shear stiffness is not defined on the cloth prim: {self.name}.")
            else:
                values[write_idx] = self._cloth_auto_apis[i.tolist()].GetSpringShearStiffnessAttr().Get()
            write_idx += 1
        return values

    def get_self_collision_filters(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the self collision filters for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the self collision filters tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        self_collision_filters = self._backend_utils.create_zeros_tensor(
            [indices.shape[0]], dtype="bool", device=self._device
        )
        write_idx = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:selfCollisionFilter" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"selfCollisionFilter is not defined on the cloth prim: {self.name}.")
            else:
                self_collision_filters[write_idx] = self._cloth_apis[i.tolist()].GetSelfCollisionFilterAttr().Get()
            write_idx += 1
        return self_collision_filters

    def get_self_collisions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the self collision for the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the self collision tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        self_collisions = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:selfCollision" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"selfCollision is not defined on the cloth prim: {self.name}.")
            else:
                self_collisions[write_idx] = self._cloth_apis[i.tolist()].GetSelfCollisionAttr().Get()
            write_idx += 1
        return self_collisions

    def get_pressures(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the pressures of the cloths indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: cloths pressure with shape (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        pressures = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:pressure" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"pressure is not defined on the cloth prim: {self.name}.")
            else:
                pressures[write_idx] = self._cloth_apis[i.tolist()].GetPressureAttr().Get()
            write_idx += 1
        return pressures

    def get_particle_groups(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the particle groups of the cloths indicated by the indices.
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which cloth prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: particle groups with shape (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        particle_groups = self._backend_utils.create_zeros_tensor(
            [indices.shape[0]], dtype="int32", device=self._device
        )
        write_idx = 0
        for i in indices:
            self._apply_cloth_api(i.tolist())
            if "physxParticle:particleGroup" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"particleGroup is not defined on the cloth prim: {self.name}.")
            else:
                particle_groups[write_idx] = self._cloth_apis[i.tolist()].GetParticleGroupAttr().Get()
            write_idx += 1
        return particle_groups
