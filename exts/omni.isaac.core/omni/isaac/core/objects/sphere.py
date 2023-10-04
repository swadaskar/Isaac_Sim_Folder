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


class VisualSphere(GeometryPrim):
    """_summary_

        Args:
            prim_path (str): _description_
            name (str, optional): _description_. Defaults to "visual_sphere".
            position (Optional[Sequence[float]], optional): _description_. Defaults to None.
            translation (Optional[Sequence[float]], optional): _description_. Defaults to None.
            orientation (Optional[Sequence[float]], optional): _description_. Defaults to None.
            scale (Optional[Sequence[float]], optional): _description_. Defaults to None.
            visible (Optional[bool], optional): _description_. Defaults to True.
            color (Optional[np.ndarray], optional): _description_. Defaults to None.
            radius (Optional[float], optional): _description_. Defaults to None.
            visual_material (Optional[VisualMaterial], optional): _description_. Defaults to None.

        Raises:
            Exception: _description_
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "visual_sphere",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = True,
        color: Optional[np.ndarray] = None,
        radius: Optional[float] = None,
        visual_material: Optional[VisualMaterial] = None,
    ) -> None:
        if is_prim_path_valid(prim_path):
            prim = get_prim_at_path(prim_path)
            if not prim.IsA(UsdGeom.Sphere):
                raise Exception("The prim at path {} cannot be parsed as a Sphere object".format(prim_path))
            sphereGeom = UsdGeom.Sphere(prim)
        else:
            sphereGeom = UsdGeom.Sphere.Define(get_current_stage(), prim_path)
            # TODO: double check the capsule extent
            if radius is None:
                radius = 1.0
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
            VisualSphere.apply_visual_material(self, visual_material)
        if radius is not None:
            VisualSphere.set_radius(self, radius)
        radius = VisualSphere.get_radius(self)
        sphereGeom.GetExtentAttr().Set([Gf.Vec3f([-radius, -radius, -radius]), Gf.Vec3f([radius, radius, radius])])
        return

    def set_radius(self, radius: float) -> None:
        """[summary]

        Args:
            radius (float): [description]
        """
        self.geom.GetRadiusAttr().Set(radius)
        return

    def get_radius(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        return self.geom.GetRadiusAttr().Get()


class FixedSphere(VisualSphere):
    """_summary_

        Args:
            prim_path (str): _description_
            name (str, optional): _description_. Defaults to "fixed_sphere".
            position (Optional[np.ndarray], optional): _description_. Defaults to None.
            translation (Optional[np.ndarray], optional): _description_. Defaults to None.
            orientation (Optional[np.ndarray], optional): _description_. Defaults to None.
            scale (Optional[np.ndarray], optional): _description_. Defaults to None.
            visible (Optional[bool], optional): _description_. Defaults to None.
            color (Optional[np.ndarray], optional): _description_. Defaults to None.
            radius (Optional[np.ndarray], optional): _description_. Defaults to None.
            visual_material (Optional[VisualMaterial], optional): _description_. Defaults to None.
            physics_material (Optional[PhysicsMaterial], optional): _description_. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "fixed_sphere",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        radius: Optional[np.ndarray] = None,
        visual_material: Optional[VisualMaterial] = None,
        physics_material: Optional[PhysicsMaterial] = None,
    ) -> None:
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
        VisualSphere.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            radius=radius,
            visual_material=visual_material,
        )
        GeometryPrim.set_collision_enabled(self, True)
        if physics_material is not None:
            FixedSphere.apply_physics_material(self, physics_material)
        return


class DynamicSphere(RigidPrim, FixedSphere):
    """_summary_

        Args:
            prim_path (str): _description_
            name (str, optional): _description_. Defaults to "dynamic_sphere".
            position (Optional[np.ndarray], optional): _description_. Defaults to None.
            translation (Optional[np.ndarray], optional): _description_. Defaults to None.
            orientation (Optional[np.ndarray], optional): _description_. Defaults to None.
            scale (Optional[np.ndarray], optional): _description_. Defaults to None.
            visible (Optional[bool], optional): _description_. Defaults to None.
            color (Optional[np.ndarray], optional): _description_. Defaults to None.
            radius (Optional[np.ndarray], optional): _description_. Defaults to None.
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
        name: str = "dynamic_sphere",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        radius: Optional[np.ndarray] = None,
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
        FixedSphere.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            color=color,
            radius=radius,
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
