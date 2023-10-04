# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from typing import Optional, Sequence, Tuple, Union, List

# omniverse
from pxr import PhysxSchema, Usd, UsdShade
import carb

# isaac-core
import omni.kit.app
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.utils.prims import get_prim_at_path, find_matching_prim_paths, is_prim_path_valid
from omni.isaac.core.materials.particle_material import ParticleMaterial

import numpy as np
import torch


class ParticleSystemView:
    """Provides high level functions to deal with particle systems (1 or more particle systems) as well as its attributes/ properties.
    This object wraps all matching particle systems found at the regex provided at the prim_paths_expr.
    Note: not all the attributes of the PhysxSchema.PhysxParticleSystem is currently controlled with this view class
    Tensor API support will be added in the future to extend the functionality of this class to applications beyond cloth.
    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "particle_system_view",
        particle_systems_enabled: Optional[Union[np.ndarray, torch.Tensor]] = None,
        simulation_owners: Optional[Sequence[str]] = None,
        contact_offsets: Optional[Union[np.ndarray, torch.Tensor]] = None,
        rest_offsets: Optional[Union[np.ndarray, torch.Tensor]] = None,
        particle_contact_offsets: Optional[Union[np.ndarray, torch.Tensor]] = None,
        solid_rest_offsets: Optional[Union[np.ndarray, torch.Tensor]] = None,
        fluid_rest_offsets: Optional[Union[np.ndarray, torch.Tensor]] = None,
        enable_ccds: Optional[Union[np.ndarray, torch.Tensor]] = None,
        solver_position_iteration_counts: Optional[Union[np.ndarray, torch.Tensor]] = None,
        max_depenetration_velocities: Optional[Union[np.ndarray, torch.Tensor]] = None,
        winds: Optional[Union[np.ndarray, torch.Tensor]] = None,
        max_neighborhoods: Optional[int] = None,
        max_velocities: Optional[Union[np.ndarray, torch.Tensor]] = None,
        global_self_collisions_enabled: Optional[Union[np.ndarray, torch.Tensor]] = None,
    ):
        """ high level functions to deal with one or more particleSystems.
            
            Args:
                prim_paths_expr(str): Prim paths regex to encapsulate all prims that match it.
                name(str): Shortname to be used as a key by Scene class.
                particle_systems_enabled (Optional[Union[np.ndarray, torch.Tensor]], optional): Whether to enable or disable the particle system.
                simulation_owners (Optional[Sequence[str]], optional): Single PhysicsScene that simulates this particle system.
                contact_offsets (Optional[Union[np.ndarray, torch.Tensor]], optional): Contact offset used for collisions with non-particle
                    objects such as rigid or deformable bodies.
                rest_offsets (Optional[Union[np.ndarray, torch.Tensor]], optional): Rest offset used for collisions with non-particle objects
                    such as rigid or deformable bodies.
                particle_contact_offsets (Optional[Union[np.ndarray, torch.Tensor]], optional): Contact offset used for interactions
                    between particles. Must be larger than solid and fluid rest offsets.
                solid_rest_offsets (Optional[Union[np.ndarray, torch.Tensor]], optional): Rest offset used for solid-solid or solid-fluid
                    particle interactions. Must be smaller than particle contact offset.
                fluid_rest_offsets (Optional[Union[np.ndarray, torch.Tensor]], optional): Rest offset used for fluid-fluid particle interactions.
                    Must be smaller than particle contact offset.
                enable_ccds (Optional[Union[np.ndarray, torch.Tensor]], optional): Enable continuous collision detection for particles to help
                    avoid tunneling effects.
                solver_position_iteration_counts (Optional[Union[np.ndarray, torch.Tensor]], optional): Number of solver iterations for position.
                max_depenetration_velocities (Optional[Union[np.ndarray, torch.Tensor]], optional): The maximum velocity permitted to be introduced
                    by the solver to depenetrate intersecting particles.
                winds (Optional[Union[np.ndarray, torch.Tensor]], optional):The wind applied to the current particle system.
                max_neighborhoods (Optional[int], optional): The particle neighborhood size.
                max_velocities (Optional[Union[np.ndarray, torch.Tensor]], optional): Maximum particle velocity.
                global_self_collisions_enabled (Optional[Union[np.ndarray, torch.Tensor]], optional): If True, self collisions follow
                    particle-object-specific settings. If False, all particle self collisions are disabled, regardless
                    of any other settings. Improves performance if self collisions are not needed.
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

        self._applied_particle_materials = [None] * self._count
        self._binding_apis = [None] * self._count

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._backend_utils = np_utils

        # TODO: particleSystemView is currently supported only on the host
        self._device = "cpu"

        # set properties
        if particle_systems_enabled is not None:
            self.set_particle_systems_enabled(particle_systems_enabled)
        if simulation_owners is not None:
            self.set_simulation_owners(simulation_owners)
        if contact_offsets is not None:
            self.set_contact_offsets(contact_offsets)
        if rest_offsets is not None:
            self.set_rest_offsets(rest_offsets)
        if particle_contact_offsets is not None:
            self.set_particle_contact_offsets(particle_contact_offsets)
        if solid_rest_offsets is not None:
            self.set_solid_rest_offsets(solid_rest_offsets)
        if fluid_rest_offsets is not None:
            self.set_fluid_rest_offsets(fluid_rest_offsets)
        if enable_ccds is not None:
            self.set_enable_ccds(enable_ccds)
        if solver_position_iteration_counts is not None:
            self.set_solver_position_iteration_counts(solver_position_iteration_counts)
        if max_depenetration_velocities is not None:
            self.set_max_depenetration_velocities(max_depenetration_velocities)
        if winds is not None:
            self.set_wind(winds)
        if max_neighborhoods is not None:
            self.set_max_neighborhoods(max_neighborhoods)
        if max_velocities is not None:
            self.set_max_velocities(max_velocities)
        if global_self_collisions_enabled is not None:
            self.set_global_self_collisions_enabled(global_self_collisions_enabled)

        timeline = omni.timeline.get_timeline_interface()
        self._invalidate_physics_handle_event = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._invalidate_physics_handle_callback
        )

    def _apply_material_binding_api(self, index):
        if self._binding_apis[index] is None:
            if self._prims[index].HasAPI(UsdShade.MaterialBindingAPI):
                self._binding_apis[index] = UsdShade.MaterialBindingAPI(self._prims[index])
            else:
                self._binding_apis[index] = UsdShade.MaterialBindingAPI.Apply(self._prims[index])

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

    def is_physics_handle_valid(self) -> bool:
        """
        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a Particle System View.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info("initializing view for {}".format(self._name))
        self._physics_sim_view = physics_sim_view
        self._physics_view = physics_sim_view.create_particle_system_view(self._regex_prim_paths.replace(".*", "*"))
        self._count = self._physics_view.count
        carb.log_info("Particle System View Device: {}".format(self._device))
        return

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
        return

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

    def apply_particle_materials(
        self,
        particle_materials: Union[ParticleMaterial, List[ParticleMaterial]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """ Used to apply particle material to prims in the view.

            Args:
                particle_materials (Union[ParticleMaterial, List[ParticleMaterial]]): particle materials to be applied to prims in the view. 
                                                                                    Note: if a physics material is not defined, 
                                                                                    the defaults will be used from PhysX.
                                                                                    If a list is provided then its size has to be equal 
                                                                                    the view's size or indices size. 
                                                                                    If one material is provided it will be applied to all prims in the view.
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            Raises:
                Exception: length of physics materials != length of prims indexed
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if isinstance(particle_materials, list):
            if indices.shape[0] != len(particle_materials):
                raise Exception("length of particle materials != length of prims indexed")
        read_idx = 0
        for i in indices:
            self._apply_material_binding_api(i.tolist())
            material = particle_materials[read_idx] if isinstance(particle_materials, list) else particle_materials
            self._binding_apis[i.tolist()].Bind(
                material.material, bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics"
            )
            self._applied_particle_materials[i.tolist()] = material
            read_idx += 1

    def get_applied_particle_materials(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> List[ParticleMaterial]:
        """ Gets the applied particle material to prims in the view.

            Args:
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

            Returns:
                List[ParticleMaterial]: the current applied particle materials for prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = [None] * indices.shape[0]
        write_idx = 0
        for i in indices:
            self._apply_material_binding_api(i.tolist())
            if self._applied_particle_materials[i.tolist()] is not None:
                result[write_idx] = self._applied_particle_materials[i.tolist()]
                write_idx += 1
            else:
                physics_binding = self._binding_apis[i.tolist()].GetDirectBinding(materialPurpose="physics")
                material_path = physics_binding.GetMaterialPath()
                if material_path == "":
                    result[write_idx] = None
                else:
                    self._applied_particle_materials[i.tolist()] = ParticleMaterial(prim_path=material_path)
                    result[write_idx] = self._applied_particle_materials[i.tolist()]
                write_idx += 1
        return result

    """
    Operations - Setters.
    """

    def set_particle_contact_offsets(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the contact offset used for interactions between particles.

            Note: Must be larger than solid and fluid rest offsets.

            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]): The contact offset.
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.clone_tensor(
                self._physics_view.get_particle_contact_offsets(), device=self._device
            )
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_particle_contact_offsets(new_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if "particleContactOffset" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(
                        "particleContactOffset property needs to be set for {} before setting its value".format(
                            self.name
                        )
                    )
                self._prims[i.tolist()].GetAttribute("particleContactOffset").Set(values[idx_count].tolist())
                idx_count += 1

    def set_solid_rest_offsets(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the rest offset used for solid-solid or solid-fluid particle interactions.

            Note: Must be smaller than particle contact offset.

            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]): solid rest offset to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).        
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.clone_tensor(
                self._physics_view.get_solid_rest_offsets(), device=self._device
            )
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_solid_rest_offsets(new_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if "solidRestOffset" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(
                        "solidRestOffset property needs to be set for {} before setting its value".format(self.name)
                    )
                self._prims[i.tolist()].GetAttribute("solidRestOffset").Set(values[idx_count].tolist())
                idx_count += 1

    def set_fluid_rest_offsets(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the rest offset used for fluid-fluid particle interactions.

            Note: Must be smaller than particle contact offset.

            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]): fluid rest offset to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.clone_tensor(
                self._physics_view.get_fluid_rest_offsets(), device=self._device
            )
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_fluid_rest_offsets(new_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if "fluidRestOffset" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(
                        "fluidRestOffset property needs to be set for {} before setting its value".format(self.name)
                    )
                self._prims[i.tolist()].GetAttribute("fluidRestOffset").Set(values[idx_count].tolist())
                idx_count += 1

    def set_winds(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the winds velocities applied to the current particle system.

            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]): The wind applied to the current particle system. shape is (M, 3).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view). 
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            new_values = self._backend_utils.clone_tensor(self._physics_view.get_wind(), device=self._device)
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_wind(new_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                if "wind" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error("wind property needs to be set for {} before setting its value".format(self.name))
                self._prims[i.tolist()].GetAttribute("wind").Set(tuple(values[idx_count].tolist()))
                idx_count += 1

    def set_max_velocities(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the maximum particle velocity for particle systems.
        
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "maxVelocity" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error("maxVelocity property needs to be set for {} before setting its value".format(self.name))
            self._prims[i.tolist()].GetAttribute("maxVelocity").Set(values[idx_count].tolist())
            idx_count += 1

    def set_max_depenetration_velocities(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """
            Set the maximum velocity permitted to be introduced by the solver to depenetrate intersecting particles for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "maxDepenetrationVelocity" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "maxDepenetrationVelocity property needs to be set for {} before setting its value".format(
                        self.name
                    )
                )
            self._prims[i.tolist()].GetAttribute("maxDepenetrationVelocity").Set(values[idx_count].tolist())
            idx_count += 1

    def set_rest_offsets(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the rest offset used for collisions with non-particle objects such as rigid or deformable bodies for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "restOffset" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error("restOffset property needs to be set for {} before setting its value".format(self.name))
            self._prims[i.tolist()].GetAttribute("restOffset").Set(values[idx_count].tolist())
            idx_count += 1

    def set_contact_offsets(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the contact offset used for collisions with non-particle objects such as rigid or deformable bodies for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "contactOffset" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "contactOffset property needs to be set for {} before setting its value".format(self.name)
                )
            self._prims[i.tolist()].GetAttribute("contactOffset").Set(values[idx_count].tolist())
            idx_count += 1

    def set_solver_position_iteration_counts(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the number of solver iterations for position for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "solverPositionIterationCount" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "solverPositionIteration property needs to be set for {} before setting its value".format(self.name)
                )
            self._prims[i.tolist()].GetAttribute("solverPositionIterationCount").Set(values[idx_count].tolist())
            idx_count += 1

    def set_max_neighborhoods(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the particle neighborhood size for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "maxNeighborhood" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "maxNeighborhood property needs to be set for {} before setting its value".format(self.name)
                )
            self._prims[i.tolist()].GetAttribute("maxNeighborhood").Set(values[idx_count].tolist())
            idx_count += 1

    def set_global_self_collisions_enabled(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Enable self collisions to follow particle-object-specific settings for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "globalSelfCollisionEnabled" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "globalSelfCollisionEnabled property needs to be set for {} before setting its value".format(
                        self.name
                    )
                )
            self._prims[i.tolist()].GetAttribute("globalSelfCollisionEnabled").Set(values[idx_count].tolist())
            idx_count += 1

    def set_enable_ccds(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Enable continuous collision detection for particles for particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "enableCCD" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error("enableCCD property needs to be set for {} before setting its value".format(self.name))
            self._prims[i.tolist()].GetAttribute("enableCCD").Set(values[idx_count].tolist())
            idx_count += 1

    def set_particle_systems_enabled(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set enabling of the particle systems.
            
            Args:
                values (Optional[Union[np.ndarray, torch.Tensor]]):  maximum particle velocity tensor to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "particleSystemEnabled" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "particleSystemEnabled property needs to be set for {} before setting its value".format(self.name)
                )
            self._prims[i.tolist()].GetAttribute("particleSystemEnabled").Set(values[idx_count].tolist())
            idx_count += 1

    def set_simulation_owners(
        self, values: Sequence[str], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """ Set the PhysicsScene that simulates particle systems.
            
            Args:
                values (Sequence[str]): PhysicsScene list to set particle systems to. shape is (M, ).
                indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to manipulate. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).              
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "simulationOwner" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    "simulationOwner property needs to be set for {} before setting its value".format(self.name)
                )
            self._prims[i.tolist()].GetRelationship("simulationOwner").SetTargets([values[idx_count]])
            idx_count += 1

    """
    Operations - Getters.
    """

    def get_particle_contact_offsets(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The contact offset used for interactions between particles in the view concatenated. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            results = self._physics_view.get_particle_contact_offsets()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                results[write_idx] = self._prims[i.tolist()].GetAttribute("particleContactOffset").Get()
                write_idx += 1
            return results

    def get_solid_rest_offsets(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The rest offset used for solid-solid or solid-fluid particle interactions. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            results = self._physics_view.get_solid_rest_offsets()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                results[write_idx] = self._prims[i.tolist()].GetAttribute("solidRestOffset").Get()
                write_idx += 1
            return results

    def get_fluid_rest_offsets(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The rest offset used for fluid-fluid particle interactions. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            results = self._physics_view.get_fluid_rest_offsets()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                results[write_idx] = self._prims[i.tolist()].GetAttribute("fluidRestOffset").Get()
                write_idx += 1
            return results

    def get_winds(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The winds applied to the current particle system. shape is (M, 3).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            results = self._physics_view.get_wind()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                results[write_idx] = self._backend_utils.create_tensor_from_list(
                    self._prims[i.tolist()].GetAttribute("wind").Get(), dtype="float32", device=self._device
                )
                write_idx += 1
            return results

    def get_max_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The maximum particle velocities for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("maxVelocity").Get()
            write_idx += 1
        return results

    def get_max_depenetration_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The maximum velocity permitted to be introduced by the solver to
                                                depenetrate intersecting particles for particle systems for each particle system. shape is (M, ).
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("maxDepenetrationVelocity").Get()
            write_idx += 1
        return results

    def get_rest_offsets(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The rest offset used for collisions with non-particle objects for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("restOffset").Get()
            write_idx += 1
        return results

    def get_contact_offsets(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The contact offset  used for collisions with non-particle objects for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("contactOffset").Get()
            write_idx += 1
        return results

    def get_solver_position_iteration_counts(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]: The number of solver iterations for positions for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="int32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("solverPositionIteration").Get()
            write_idx += 1
        return results

    def get_max_neighborhoods(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]:  The particle neighborhood size for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="int32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("maxNeighborhood").Get()
            write_idx += 1
        return results

    def get_global_self_collisions_enabled(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]:  Whether self collisions to follow particle-object-specific settings 
                                                is enabled or disabled. for each particle system. shape is (M, ).  
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("globalSelfCollisionEnabled").Get()
            write_idx += 1
        return results

    def get_enable_ccds(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]:  Whether continuous collision detection for particles is enabled or disabled for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("enableCCD").Get()
            write_idx += 1
        return results

    def get_particle_systems_enabled(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """
        Returns:
            Union[np.ndarray, torch.Tensor]:  Whether particle system is enabled or not for each particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("particleSystemEnabled").Get()
            write_idx += 1
        return results

    def get_simulation_owners(self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None) -> Sequence[str]:
        """
        Returns:
            Sequence[str]: The physics scene prim path attached to particle system. shape is (M, ).  
        
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = []
        for i in indices:
            results.append(self._prims[i.tolist()].GetRelationship("simulationOwner").Get())
        return results
