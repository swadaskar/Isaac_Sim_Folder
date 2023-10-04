# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Tuple, Sequence
from omni.isaac.core.materials.visual_material import VisualMaterial
from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.utils.types import XFormPrimState
from omni.isaac.core.utils.string import find_unique_string_name
from pxr import Gf, PhysicsSchemaTools, Usd
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.utils.prims import (
    get_prim_path,
    is_prim_path_valid,
    get_first_matching_child_prim,
    get_prim_type_name,
)
from omni.isaac.core.utils.stage import get_current_stage, get_stage_units
import numpy as np
import carb


class GroundPlane(object):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "ground_plane".
            size (Optional[float], optional): [description]. Defaults to 5000.0.
            z_position (float, optional): [description]. Defaults to 0.
            scale (Optional[np.ndarray], optional): [description]. Defaults to None.
            visible (bool, optional): [description]. Defaults to True.
            color (Optional[np.ndarray], optional): [description]. Defaults to None.
            physics_material_path (Optional[PhysicsMaterial], optional): [description]. Defaults to None.
            visual_material (Optional[VisualMaterial], optional): [description]. Defaults to None.
            static_friction (float, optional): [description]. Defaults to 0.5.
            dynamic_friction (float, optional): [description]. Defaults to 0.5.
            restitution (float, optional): [description]. Defaults to 0.8.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "ground_plane",
        size: Optional[float] = None,
        z_position: Optional[float] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        physics_material: Optional[PhysicsMaterial] = None,
        visual_material: Optional[VisualMaterial] = None,
    ) -> None:
        # wrap two object the xform and the collision plane
        if not is_prim_path_valid(prim_path):
            carb.log_info("Creating a new Ground Plane prim at path {}".format(prim_path))
            stage = get_current_stage()
            if size is None:
                size = 50.0 / get_stage_units()
            if z_position is None:
                z_position = 0.0
            PhysicsSchemaTools.addGroundPlane(
                stage, prim_path, "Z", size, Gf.Vec3f(0, 0, z_position), Gf.Vec3f([0.0, 0.0, 0.0])
            )
            collision_prim_path = prim_path + "/geom"
            # set default values if no physics material given
            if physics_material is None:
                static_friction = 0.5
                dynamic_friction = 0.5
                restitution = 0.8
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
            if visual_material is None:
                if color is None:
                    color = np.array([0.5, 0.5, 0.5])
                visual_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/visual_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
                visual_material = PreviewSurface(prim_path=visual_prim_path, color=color)
        else:
            collision_prim_path = get_prim_path(
                get_first_matching_child_prim(prim_path=prim_path, predicate=lambda x: get_prim_type_name(x) == "Plane")
            )

        self._xform_prim = XFormPrim(
            prim_path=prim_path, name=name, position=None, orientation=None, scale=scale, visible=visible
        )
        self._collision_prim = GeometryPrim(
            prim_path=collision_prim_path,
            name=name + "_collision_plane",
            position=None,
            orientation=None,
            scale=scale,
            visible=visible,
            collision=True,
        )
        if z_position is not None:
            position = self._xform_prim._backend_utils.create_tensor_from_list(
                [0, 0, z_position], dtype="float32", device=self._xform_prim._device
            )
            self._xform_prim.set_world_pose(position=position)
            self._xform_prim.set_default_state(position=position)
            self._collision_prim.set_world_pose(position=position)
            self._collision_prim.set_default_state(position=position)
        if physics_material is None:
            self._collision_prim.apply_physics_material(physics_material)
        if visual_material is not None:
            self._xform_prim.apply_visual_material(visual_material)
        return

    @property
    def prim_path(self) -> str:
        """
        Returns:
            str: prim path in the stage.
        """
        return self._xform_prim.prim_path

    @property
    def name(self) -> Optional[str]:
        """
        Returns:
            str: name given to the prim when instantiating it. Otherwise None.
        """
        return self._xform_prim.name

    @property
    def prim(self) -> Usd.Prim:
        """
        Returns:
            Usd.Prim: USD Prim object that this object holds.
        """
        return self._xform_prim.prim

    @property
    def xform_prim(self) -> XFormPrim:
        """_summary_

        Returns:
            XFormPrim: _description_
        """
        return self._xform_prim

    @property
    def collision_geometry_prim(self) -> GeometryPrim:
        """_summary_

        Returns:
            GeometryPrim: _description_
        """
        return self._collision_prim

    def initialize(self, physics_sim_view=None) -> None:
        self._xform_prim.initialize(physics_sim_view=physics_sim_view)
        self._collision_prim.initialize(physics_sim_view=physics_sim_view)
        return

    def post_reset(self) -> None:
        """Resets the prim to its default state (position and orientation).
        """
        self._xform_prim.post_reset()
        self._collision_prim.post_reset()
        return

    def is_valid(self) -> bool:
        """
        Returns:
            bool: True is the current prim path corresponds to a valid prim in stage. False otherwise.
        """
        return self._xform_prim.is_valid()

    def apply_physics_material(self, physics_material: PhysicsMaterial, weaker_than_descendants: bool = False):
        """Used to apply physics material to the held prim and optionally its descendants.

        Args:
            physics_material (PhysicsMaterial): physics material to be applied to the held prim. This where you want to
                                                define friction, restitution..etc. Note: if a physics material is not
                                                defined, the defaults will be used from PhysX.
            weaker_than_descendants (bool, optional): True if the material shouldn't override the descendants
                                                      materials, otherwise False. Defaults to False.
        """
        self._collision_prim.apply_physics_material(
            physics_materials=physics_material, weaker_than_descendants=weaker_than_descendants
        )
        return

    def get_applied_physics_material(self) -> PhysicsMaterial:
        """Returns the current applied physics material in case it was applied using apply_physics_material or not.

        Returns:
            PhysicsMaterial: the current applied physics material.
        """
        return self._collision_prim.get_applied_physics_material()

    def set_world_pose(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Sets prim's pose with respect to the world's frame.

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        self._collision_prim.set_world_pose(position=position, orientation=orientation)
        self._xform_prim.set_world_pose(position=position, orientation=orientation)
        return

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Gets prim's pose with respect to the world's frame.

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is position in the world frame of the prim. shape is (3, ). 
                                           second index is quaternion orientation in the world frame of the prim.
                                           quaternion is scalar-first (w, x, y, z). shape is (4, ).
        """
        return self._xform_prim.get_world_pose()

    def get_default_state(self) -> XFormPrimState:
        """
        Returns:
            XFormPrimState: returns the default state of the prim (position and orientation) that is used after each reset.
        """
        return self._xform_prim.get_default_state()

    def set_default_state(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Sets the default state of the prim (position and orientation), that will be used after each reset.

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        self._xform_prim.set_default_state(position=position, orientation=orientation)
        self._collision_prim.set_default_state(position=position, orientation=orientation)
        return
