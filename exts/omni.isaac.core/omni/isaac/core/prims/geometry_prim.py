# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional, Sequence, Union, List
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.prims._impl.single_prim_wrapper import _SinglePrimWrapper
from omni.isaac.core.prims.geometry_prim_view import GeometryPrimView
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from pxr import UsdGeom
import numpy as np
import torch


class GeometryPrim(_SinglePrimWrapper):
    """Provides high level functions to deal with a Geom prim and its attributes/ properties.
           The prim_path should correspond to type UsdGeom.Cube, UsdGeom.Capsule, UsdGeom.Cone, UsdGeom.Cylinder,
           UsdGeom.Sphere or UsdGeom.Mesh.

        Args:
            prim_path (str): prim path of the Prim to encapsulate or create.
            name (str, optional): shortname to be used as a key by Scene class.
                                    Note: needs to be unique if the object is added to the Scene.
                                    Defaults to "xform_prim".
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                            (with respect to its parent prim). shape is (3, ).
                                                            Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                            (depends if translation or position is specified).
                                                            quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                            Defaults to None, which means left unchanged.
            scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                    Defaults to None, which means left unchanged.
            visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
            collision (bool, optional): Set to True if the geometry should have a collider (i.e not only a visual geometry).
                                        Defaults to False.
            track_contact_forces (bool, Optional) : if enabled, the view will track the net contact forces on each geometry prim in the view. 
                                                    Note that the collision flag should be set to True to report contact forces. Defaults to False.
            prepare_contact_sensors (bool, Optional): applies contact reporter API to the prim if it already does not have one. Defaults to False.
            disable_stablization (bool, optional): disables the contact stablization parameter in the physics context. Defaults to True.
            contact_filter_prim_paths_expr (Optional[List[str]], Optional): a list of filter expressions which allows for tracking contact forces 
                                                                    between the geometry prim and this subset through get_contact_force_matrix(). 
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "geometry_prim",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        collision: bool = False,
        track_contact_forces: bool = False,
        prepare_contact_sensor: bool = False,
        disable_stablization: bool = True,
        contact_filter_prim_paths_expr: Optional[List[str]] = [],
    ) -> None:
        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._device = None
            self._backend_utils = np_utils
        if position is not None:
            position = self._backend_utils.convert(position, self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if translation is not None:
            translation = self._backend_utils.convert(translation, self._device)
            translation = self._backend_utils.expand_dims(translation, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if scale is not None:
            scale = self._backend_utils.convert(scale, self._device)
            scale = self._backend_utils.expand_dims(scale, 0)
        if visible is not None:
            visible = self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)
        collision = self._backend_utils.create_tensor_from_list([collision], dtype="bool", device=self._device)
        self._geometry_prim_view = GeometryPrimView(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            translations=translation,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            collisions=collision,
            track_contact_forces=track_contact_forces,
            prepare_contact_sensors=prepare_contact_sensor,
            disable_stablization=disable_stablization,
            contact_filter_prim_paths_expr=contact_filter_prim_paths_expr,
        )
        _SinglePrimWrapper.__init__(self, view=self._geometry_prim_view)

    @property
    def geom(self) -> UsdGeom.Gprim:
        """
        Returns:
            UsdGeom.Gprim: USD geometry object encapsulated.
        """
        return self._geometry_prim_view.geoms[0]

    def set_contact_offset(self, offset: float) -> None:
        """
        Args:
            offset (float): Contact offset of a collision shape. Allowed range [maximum(0, rest_offset), 0].
                            Default value is -inf, means default is picked by simulation based on the shape extent.
        """
        offset = self._backend_utils.create_tensor_from_list([offset], dtype="float32", device=self._device)
        self._geometry_prim_view.set_contact_offsets(offsets=offset)
        return

    def get_contact_offset(self) -> float:
        """
        Returns:
            float: contact offset of the collision shape.
        """
        return self._geometry_prim_view.get_contact_offsets()[0]

    def set_rest_offset(self, offset: float) -> None:
        """
        Args:
            offset (float): Rest offset of a collision shape. Allowed range [-max_float, contact_offset.
                            Default value is -inf, means default is picked by simulatiion. For rigid bodies its zero.
        """
        offset = self._backend_utils.create_tensor_from_list([offset], dtype="float32", device=self._device)
        self._geometry_prim_view.set_rest_offsets(offsets=offset)
        return

    def get_rest_offset(self) -> float:
        """
        Returns:
            float: rest offset of the collision shape.
        """
        return self._geometry_prim_view.get_rest_offsets()[0]

    def set_torsional_patch_radius(self, radius: float) -> None:
        """
        Args:
            radius (float): radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].
        """
        radius = self._backend_utils.create_tensor_from_list([radius], dtype="float32", device=self._device)
        self._geometry_prim_view.set_torsional_patch_radii(radii=radius)
        return

    def get_torsional_patch_radius(self) -> float:
        """
        Returns:
            float: radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].
        """
        return self._geometry_prim_view.get_torsional_patch_radii()[0]

    def set_min_torsional_patch_radius(self, radius: float) -> None:
        """
        Args:
            radius (float): minimum radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].
        """
        radius = self._backend_utils.create_tensor_from_list([radius], dtype="float32", device=self._device)
        self._geometry_prim_view.set_min_torsional_patch_radii(radii=radius)
        return

    def get_min_torsional_patch_radius(self) -> float:
        """
        Returns:
            float: minimum radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].
        """
        return self._geometry_prim_view.get_min_torsional_patch_radii()[0]

    def set_collision_approximation(self, approximation_type: str) -> None:
        """

        Args:
            approximation_type (str): approximation used for collision, could be "none", "convexHull" or "convexDecomposition"
        """
        self._geometry_prim_view.set_collision_approximations([approximation_type])
        return

    def get_collision_approximation(self) -> str:
        """
        Returns:
            str: approximation used for collision, could be "none", "convexHull" or "convexDecomposition"
        """
        return self._geometry_prim_view.get_collision_approximations()[0]

    def set_collision_enabled(self, enabled: bool) -> None:
        """

        Args:
        """
        if enabled:
            self._geometry_prim_view.enable_collision()
        else:
            self._geometry_prim_view.disable_collision()
        return

    def get_collision_enabled(self) -> bool:
        """
        Returns:
        """
        return self._geometry_prim_view.is_collision_enabled()[0]

    def apply_physics_material(self, physics_material: PhysicsMaterial, weaker_than_descendants: bool = False):
        """Used to apply physics material to the held prim and optionally its descendants.

        Args:
            physics_material (PhysicsMaterial): physics material to be applied to the held prim. This where you want to
                                                define friction, restitution..etc. Note: if a physics material is not
                                                defined, the defaults will be used from PhysX.
            weaker_than_descendants (bool, optional): True if the material shouldn't override the descendants
                                                      materials, otherwise False. Defaults to False.
        """
        self._geometry_prim_view.apply_physics_materials(
            physics_materials=physics_material, weaker_than_descendants=weaker_than_descendants
        )
        return

    def get_applied_physics_material(self) -> PhysicsMaterial:
        """Returns the current applied physics material in case it was applied using apply_physics_material or not.

        Returns:
            PhysicsMaterial: the current applied physics material.
        """
        return self._geometry_prim_view.get_applied_physics_materials()[0]

    def get_net_contact_forces(self, dt: float = 1.0) -> Union[np.ndarray, torch.Tensor]:
        """
        If contact forces of the prims in the view are tracked, this method returns the net contact forces on prims. 
        i.e., a matrix of dimension (1, 3)

        Args:
            dt (float): time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Union[np.ndarray, torch.Tensor]: Net contact forces of the prims with shape (3).

        """
        return self._geometry_prim_view.get_net_contact_forces(dt=dt)[0]

    def get_contact_force_matrix(self, dt: float = 1.0) -> Union[np.ndarray, torch.Tensor]:
        """
        If the object is initialized with filter_paths_expr list, this method returns the contact forces between the prims 
        in the view and the filter prims. i.e., a matrix of dimension (self._contact_view.num_filters, 3) 
        where num_filters is the determined according to the filter_paths_expr parameter.

        Args:
            dt (float): time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Union[np.ndarray, torch.Tensor]: Net contact forces of the prims with shape (self._geometry_prim_view._contact_view.num_filters, 3).
        """
        return self._geometry_prim_view._contact_view.get_contact_force_matrix(dt=dt)[0]
