# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional
import numpy as np
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from pxr import UsdGeom
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage


class DynamicObject(RigidPrim, GeometryPrim):
    """Creates and adds a prim to stage from USD reference path, and wraps the prim with RigidPrim and GeometryPrim to 
       provide access to APIs for rigid body attributes, physics materials and collisions. Please note that this class 
       assumes the object has only a single mesh prim defining its geometry.

    Args:
        usd_path (str): USD reference path the Prim refers to.
        prim_path (str): prim path of the Prim to encapsulate or create.
        mesh_path (str): prim path of the underlying mesh Prim.
        name (str, optional): shortname to be used as a key by Scene class. Note: needs to be unique if the object is 
                              added to the Scene. Defaults to "dynamic_object".
        position (Optional[np.ndarray], optional): position in the world frame of the prim. Shape is (3, ). Defaults to 
                                                   None, which means left unchanged.
        translation (Optional[np.ndarray], optional): translation in the local frame of the prim (with respect to its 
                                                      parent prim). Shape is (3, ). Defaults to None, which means left 
                                                      unchanged.
        orientation (Optional[np.ndarray], optional): quaternion orientation in the world/local frame of the prim
                                                      (depends if translation or position is specified). Quaternion is 
                                                      scalar-first (w, x, y, z). Shape is (4, ). Defaults to None, which 
                                                      means left unchanged.
        scale (Optional[np.ndarray], optional): local scale to be applied to the prim's dimensions. Shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        mass (Optional[float], optional): mass in kg. Defaults to None.
        linear_velocity (Optional[np.ndarray], optional): linear velocity in the world frame. Defaults to None.
        angular_velocity (Optional[np.ndarray], optional): angular velocity in the world frame. Defaults to None.
    """

    def __init__(
        self,
        usd_path: str,
        prim_path: str,
        mesh_path: str,
        name: str = "dynamic_object",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: bool = True,
        mass: Optional[float] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ) -> None:

        if is_prim_path_valid(mesh_path):
            prim = get_prim_at_path(mesh_path)
            if not prim.IsA(UsdGeom.Mesh):
                raise Exception("The prim at path {} cannot be parsed as a Mesh object".format(mesh_path))

        self.usd_path = usd_path

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        GeometryPrim.__init__(
            self,
            prim_path=mesh_path,
            name=name,
            translation=translation,
            orientation=orientation,
            visible=visible,
            collision=True,
        )

        self.set_collision_approximation("convexHull")

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
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )
