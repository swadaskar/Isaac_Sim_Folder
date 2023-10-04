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
from pxr import PhysxSchema

# omniverse
import carb
import omni.kit.app

# isaac-core
import omni.kit.app
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.utils.prims import get_prim_at_path, find_matching_prim_paths, is_prim_path_valid


class ParticleMaterialView:
    """The view class to deal with particleMaterial prims.
        Provides high level functions to deal with particle material (1 or more particle materials) 
        as well as its attributes/ properties. This object wraps all matching materials found at the regex provided at the prim_paths_expr.
        This object wraps all matching materials Prims found at the regex provided at the prim_paths_expr.
    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "particle_material_view",
        frictions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        particle_friction_scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        dampings: Optional[Union[np.ndarray, torch.Tensor]] = None,
        viscosities: Optional[Union[np.ndarray, torch.Tensor]] = None,
        vorticity_confinements: Optional[Union[np.ndarray, torch.Tensor]] = None,
        surface_tensions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        cohesions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        adhesions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        particle_adhesion_scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        adhesion_offset_scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        gravity_scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        lifts: Optional[Union[np.ndarray, torch.Tensor]] = None,
        drags: Optional[Union[np.ndarray, torch.Tensor]] = None,
    ):
        """
        Args:
            prim_paths_expr(str): Prim paths regex to encapsulate all prims that match it.
            name(str): Shortname to be used as a key by Scene class.
            frictions (Union[np.ndarray, torch.Tensor], optional): The friction coefficient tensor, shape is (N, ).
            particle_friction_scales (Union[np.ndarray, torch.Tensor], optional): The coefficient that scales friction for
                solid particle-particle interactions, shape is (N, ).
            dampings (Union[np.ndarray, torch.Tensor], optional): The global velocity damping tensor, shape is (N, ).
            viscosities (Union[np.ndarray, torch.Tensor], optional): The viscosity tensor of fluid particles, shape is (N, ).
            vorticity_confinements (Union[np.ndarray, torch.Tensor], optional): The vorticity confinement tensor for fluid particles, shape is (N, ).
            surface_tensions (Union[np.ndarray, torch.Tensor], optional): The surface tension tensor, shape is (N, ).
            cohesions (Union[np.ndarray, torch.Tensor], optional): The cohesion tensor for interaction between fluid particles, shape is (N, ).
            adhesions (Union[np.ndarray, torch.Tensor], optional): The adhesion tensor for interaction between particles (solid or fluid),
                and rigid or deformable objects, shape is (N, ).
            particle_adhesion_scales (Union[np.ndarray, torch.Tensor], optional): The coefficient tensor that scales adhesion for solid
                particle-particle interations, shape is (N, ).
            adhesion_offset_scales (Union[np.ndarray, torch.Tensor], optional): The offset scale tensor defines at which adhesion ceases
                to take effect, shape is (N, ).
            gravity_scales (Union[np.ndarray, torch.Tensor], optional): The gravitational acceleration scaling tensor. It can be used
                to approximate lighter-than-air inflatables, shape is (N, ).
            lifts (Union[np.ndarray, torch.Tensor], optional): The lift coefficient tensor for cloth and inflatable particle objects, shape is (N, ).
            drags (Union[np.ndarray, torch.Tensor], optional): The drag coefficient tensor for cloth and inflatable particle objects, shape is (N, ).
        """
        self._name = name
        self._physics_view = None
        self._prim_paths = find_matching_prim_paths(prim_paths_expr)
        if len(self._prim_paths) == 0:
            raise Exception(
                "Prim path expression {} is invalid, a prim matching the expression needs to created before wrapping it as view".format(
                    prim_paths_expr
                )
            )
        self._count = len(self._prim_paths)
        self._prims = []
        self._regex_prim_paths = prim_paths_expr
        for prim_path in self._prim_paths:
            self._prims.append(get_prim_at_path(prim_path))

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._backend_utils = np_utils

        # NOTE: particleSystemView is only supported on the host
        self._device = "cpu"
        self._material_apis = [None] * self._count

        # set properties
        if frictions is not None:
            self.set_frictions(frictions)
        if particle_friction_scales is not None:
            self.set_particle_friction_scales(particle_friction_scales)
        if dampings is not None:
            self.set_dampings(dampings)
        if viscosities is not None:
            self.set_viscosities(viscosities)
        if vorticity_confinements is not None:
            self.set_vorticity_confinements(vorticity_confinements)
        if surface_tensions is not None:
            self.set_surface_tensions(surface_tensions)
        if cohesions is not None:
            self.set_cohesions(cohesions)
        if adhesions is not None:
            self.set_adhesions(adhesions)
        if particle_adhesion_scales is not None:
            self.set_particle_adhesion_scales(particle_adhesion_scales)
        if adhesion_offset_scales is not None:
            self.set_adhesion_offset_scales(adhesion_offset_scales)
        if gravity_scales is not None:
            self.set_gravity_scales(gravity_scales)
        if lifts is not None:
            self.set_lifts(lifts)
        if drags is not None:
            self.set_drags(drags)

    """
    Properties.
    """

    @property
    def count(self) -> int:
        """
        Returns:
            int: number of rigid shapes for the prims in the view.
        """
        return self._count

    @property
    def name(self) -> str:
        """
        Returns:
            str: name given to the view when instantiating it.
        """
        return self._name

    def _apply_material_api(self, index):
        if self._material_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxPBDMaterialAPI):
                material_api = PhysxSchema.PhysxPBDMaterialAPI(self._prims[index])
            else:
                material_api = PhysxSchema.PhysxPBDMaterialAPI.Apply(self._prims[index])
            self._material_apis[index] = material_api

    def is_physics_handle_valid(self) -> bool:
        """
        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def is_valid(self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None) -> bool:
        """
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            bool: True if all prim paths specified in the view correspond to a valid prim in stage. False otherwise.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = True
        for index in indices:
            result = result and is_prim_path_valid(self._prim_paths[index.tolist()])
        return result

    def post_reset(self) -> None:
        """Resets the particles to their initial states.
        """
        # TODO:
        return

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
        self._physics_view = self._physics_sim_view.create_particle_material_view(
            self._regex_prim_paths.replace(".*", "*")
        )
        self._count = self._physics_view.count
        carb.log_info("Particle material View Device: {}".format(self._device))
        return

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
        return

    def set_frictions(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the friction for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material friction tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_frictions(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_friction(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:friction" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateFrictionAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetFrictionAttr().Set(values[idx_count].tolist())

                idx_count += 1

    def get_frictions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the friction of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
           Union[np.ndarray, torch.Tensor]: friction tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            current_values = self._physics_view.get_friction()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:friction" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"friction is not defined on the cloth prim: {self.name}.")
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetFrictionAttr().Get()
                write_idx += 1
            return result

    def set_dampings(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the dampings for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material damping tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_dampings(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_damping(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:damping" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateDampingAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetDampingAttr().Set(values[idx_count].tolist())

                idx_count += 1

    def get_dampings(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the dampings of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: dampings tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            current_values = self._physics_view.get_damping()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:damping" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"damping is not defined on the cloth prim: {self.name}.")
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetDampingAttr().Get()
                write_idx += 1
            return result

    def set_gravity_scales(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the gravity scale for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material gravity scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_gravity_scales(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_gravity_scale(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:gravityScale" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateGravityScaleAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetGravityScaleAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_gravity_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the gravity scale of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: gravity scale tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            current_values = self._physics_view.get_gravity_scale()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:gravityScale" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"gravityScale is not defined on the cloth prim: {self.name}.")
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetGravityScaleAttr().Get()
                write_idx += 1
            return result

    def set_lifts(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the lifts for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material lift tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_lifts(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_lift(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:lift" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateLiftAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetLiftAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_lifts(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the lifts of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: lift tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            current_values = self._physics_view.get_lift()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:lift" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"lift is not defined on the cloth prim: {self.name}.")
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetLiftAttr().Get()
                write_idx += 1
            return result

    def set_drags(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the drags for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material drag tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_drags(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_drag(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:drag" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateDragAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetDragAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_drags(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the drags of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: drag tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            current_values = self._physics_view.get_drag()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:drag" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"drag is not defined on the cloth prim: {self.name}.")
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetDragAttr().Get()
                write_idx += 1
            return result

    def set_viscosities(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle viscosity for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle viscosity scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:viscosity" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateViscosityAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetViscosityAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_viscosities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the viscosity of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: viscosity tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:viscosity" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"viscosity is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetViscosityAttr().Get()
            write_idx += 1
        return result

    def set_cohesions(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle cohesion for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle cohesion scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:cohesion" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateCohesionAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetCohesionAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_cohesions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the cohesion of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: cohesion tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:cohesion" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"cohesion is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetCohesionAttr().Get()
            write_idx += 1
        return result

    def set_adhesions(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle adhesion for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle adhesion scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesion" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateAdhesionAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetAdhesionAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_adhesions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the adhesion of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: adhesion tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesion" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"adhesion is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetAdhesionAttr().Get()
            write_idx += 1
        return result

    def set_particle_adhesion_scales(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle adhesion for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle adhesion scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleAdhesionScale" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateParticleAdhesionScaleAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetParticleAdhesionScaleAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_particle_adhesion_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the adhesion scale of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: adhesion scale tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleAdhesionScale" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"particleAdhesionScale is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetParticleAdhesionScaleAttr().Get()
            write_idx += 1
        return result

    def set_adhesion_offset_scales(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the adhesion offset scale for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material adhesion offset scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesionOffsetScale" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateAdhesionOffsetScaleAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetAdhesionOffsetScaleAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_adhesion_offset_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the adhesion offset scale of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: adhesion offset scale tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesionOffsetScale" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"adhesionOffsetScale is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetAdhesionOffsetScaleAttr().Get()
            write_idx += 1
        return result

    def set_surface_tensions(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle surface tension for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle surface tension scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:surfaceTension" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateSurfaceTensionAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetSurfaceTensionAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_surface_tensions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the surface tension of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: surface tension tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:surfaceTension" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"surface tension is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetSurfaceTensionAttr().Get()
            write_idx += 1
        return result

    def set_vorticity_confinements(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the vorticity confinement for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle vorticity confinement scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:vorticityConfinement" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreatetVorticityConfinementAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetVorticityConfinementAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_vorticity_confinements(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the vorticity confinement of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: vorticity confinement tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:vorticityConfinement" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"vorticityConfinement is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetVorticityConfinementAttr().Get()
            write_idx += 1
        return result

    def set_particle_friction_scales(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the particle friction scale for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material particle friction scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleFrictionScale" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateParticleFrictionScaleAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetParticleFrictionScaleAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_particle_friction_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the particle friction scale of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: particle friction scale tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleFrictionScale" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"particleFrictionScale is not defined on the cloth prim: {self.name}.")
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetParticleFrictionScaleAttr().Get()
            write_idx += 1
        return result
