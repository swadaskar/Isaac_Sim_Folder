# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence
import numpy as np
from omni.isaac.core.materials.visual_material import VisualMaterial
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.utils.string import find_unique_string_name
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.utils.stage import get_current_stage


class VisualCuboid(GeometryPrim):
    """_summary_

    Args:
        prim_path (str): _description_
        name (str, optional): _description_. Defaults to "visual_cube".
        position (Optional[Sequence[float]], optional): _description_. Defaults to None.
        translation (Optional[Sequence[float]], optional): _description_. Defaults to None.
        orientation (Optional[Sequence[float]], optional): _description_. Defaults to None.
        scale (Optional[Sequence[float]], optional): _description_. Defaults to None.
        visible (Optional[bool], optional): _description_. Defaults to None.
        color (Optional[np.ndarray], optional): _description_. Defaults to None.
        size (Optional[float], optional): _description_. Defaults to None.
        visual_material (Optional[VisualMaterial], optional): _description_. Defaults to None.

    Raises:
        Exception: _description_
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "visual_cube",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        size: Optional[float] = None,
        visual_material: Optional[VisualMaterial] = None,
    ) -> None:
        if is_prim_path_valid(prim_path):
            prim = get_prim_at_path(prim_path)
            if not prim.IsA(UsdGeom.Cube):
                raise Exception("The prim at path {} cannot be parsed as a Cube object".format(prim_path))
            cubeGeom = UsdGeom.Cube(prim)
        else:
            cubeGeom = UsdGeom.Cube.Define(get_current_stage(), prim_path)
            if size is None:
                size = 1.0
            if visible is None:
                visible = True
            if visual_material is None:
                if color is None:
                    color = np.array([0.5, 0.5, 0.5])
                visual_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/visual_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
                visual_material = PreviewSurface(prim_path=visual_prim_path, color=color)
        GeometryPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            collision=False,
        )
        if visual_material is not None:
            VisualCuboid.apply_visual_material(self, visual_material)
        if size is not None:
            VisualCuboid.set_size(self, size)
        size = VisualCuboid.get_size(self)
        extent_size = size * VisualCuboid.get_local_scale(self)
        cubeGeom.GetExtentAttr().Set(
            [
                Gf.Vec3f([-extent_size[0] / 2.0, -extent_size[1] / 2.0, -extent_size[2] / 2.0]),
                Gf.Vec3f([extent_size[0] / 2.0, extent_size[1] / 2.0, extent_size[2] / 2.0]),
            ]
        )
        return

    def set_size(self, size: float) -> None:
        """[summary]

        Args:
            size (float): [description]
        """
        self.geom.CreateSizeAttr(size)
        return

    def get_size(self) -> np.ndarray:
        """[summary]

        Returns:
            float: [description]
        """
        return self.geom.GetSizeAttr().Get()


class FixedCuboid(VisualCuboid):
    """_summary_

        Args:
            prim_path (str): _description_
            name (str, optional): _description_. Defaults to "fixed_cube".
            position (Optional[np.ndarray], optional): _description_. Defaults to None.
            translation (Optional[np.ndarray], optional): _description_. Defaults to None.
            orientation (Optional[np.ndarray], optional): _description_. Defaults to None.
            scale (Optional[np.ndarray], optional): _description_. Defaults to None.
            visible (Optional[bool], optional): _description_. Defaults to None.
            color (Optional[np.ndarray], optional): _description_. Defaults to None.
            size (Optional[float], optional): _description_. Defaults to None.
            visual_material (Optional[VisualMaterial], optional): _description_. Defaults to None.
            physics_material (Optional[PhysicsMaterial], optional): _description_. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "fixed_cube",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        size: Optional[float] = None,
        visual_material: Optional[VisualMaterial] = None,
        physics_material: Optional[PhysicsMaterial] = None,
    ) -> None:
        set_offsets = False
        if not is_prim_path_valid(prim_path):
            # set default values if no physics material given
            if physics_material is None:
                static_friction = 0.2
                dynamic_friction = 1.0
                restitution = 0.0
                physics_material_path = find_unique_string_name(
                    initial_name="/World/Physics_Materials/physics_material",
                    is_unique_fn=lambda x: not is_prim_path_valid(x),
                )
                physics_material = PhysicsMaterial(
                    prim_path=physics_material_path,
                    dynamic_friction=dynamic_friction,
                    static_friction=static_friction,
                    restitution=restitution,
                )
            set_offsets = True
        VisualCuboid.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            size=size,
            visual_material=visual_material,
        )
        GeometryPrim.set_collision_enabled(self, True)
        if physics_material is not None:
            FixedCuboid.apply_physics_material(self, physics_material)
        if set_offsets:
            FixedCuboid.set_rest_offset(self, 0.0)
            FixedCuboid.set_contact_offset(self, 0.1)
            FixedCuboid.set_torsional_patch_radius(self, 1.0)
            FixedCuboid.set_min_torsional_patch_radius(self, 0.8)
        return


class DynamicCuboid(RigidPrim, FixedCuboid):
    """_summary_

        Args:
            prim_path (str): _description_
            name (str, optional): _description_. Defaults to "dynamic_cube".
            position (Optional[np.ndarray], optional): _description_. Defaults to None.
            translation (Optional[np.ndarray], optional): _description_. Defaults to None.
            orientation (Optional[np.ndarray], optional): _description_. Defaults to None.
            scale (Optional[np.ndarray], optional): _description_. Defaults to None.
            visible (Optional[bool], optional): _description_. Defaults to None.
            color (Optional[np.ndarray], optional): _description_. Defaults to None.
            size (Optional[float], optional): _description_. Defaults to None.
            visual_material (Optional[VisualMaterial], optional): _description_. Defaults to None.
            physics_material (Optional[PhysicsMaterial], optional): _description_. Defaults to None.
            mass (Optional[float], optional): _description_. Defaults to None.
            density (Optional[float], optional): _description_. Defaults to None.
            linear_velocity (Optional[Sequence[float]], optional): _description_. Defaults to None.
            angular_velocity (Optional[Sequence[float]], optional): _description_. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "dynamic_cube",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        size: Optional[float] = None,
        visual_material: Optional[VisualMaterial] = None,
        physics_material: Optional[PhysicsMaterial] = None,
        mass: Optional[float] = None,
        density: Optional[float] = None,
        linear_velocity: Optional[Sequence[float]] = None,
        angular_velocity: Optional[Sequence[float]] = None,
    ) -> None:
        if not is_prim_path_valid(prim_path):
            if mass is None:
                mass = 0.02
        FixedCuboid.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            size=size,
            visual_material=visual_material,
            physics_material=physics_material,
        )
        RigidPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            mass=mass,
            density=density,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )
