# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
import numpy as np
import typing

# omniverse
from pxr import UsdGeom, Gf
import omni.physx

# isaacsim
from omni.isaac.core.utils.stage import get_current_stage


def ray_cast(
    position: np.array, orientation: np.array, offset: np.array, max_dist: float = 100.0
) -> typing.Tuple[typing.Union[None, str], float]:
    """Projects a raycast forward along x axis with specified offset

    If a hit is found within the maximum distance, then the object's prim path and distance to it is returned.
    Otherwise, a None and 10000 is returned.

    Args:
        position (np.array): origin's position for ray cast
        orientation (np.array): origin's orientation for ray cast
        offset (np.array): offset for ray cast
        max_dist (float, optional): maximum distance to test for collisions in stage units. Defaults to 100.0.

    Returns:
        typing.Tuple[typing.Union[None, str], float]: path to geometry that was hit and hit distance, returns None, 10000 if no hit occurred
    """
    input_tr = Gf.Matrix4f()
    input_tr.SetTranslate(Gf.Vec3f(*position.tolist()))
    input_tr.SetRotateOnly(Gf.Quatf(*orientation.tolist()))
    offset_transform = Gf.Matrix4f()
    offset_transform.SetTranslate(Gf.Vec3f(*offset.tolist()))
    raycast_tf = offset_transform * input_tr
    trans = raycast_tf.ExtractTranslation()
    direction = raycast_tf.ExtractRotation().TransformDir((1, 0, 0))
    origin = (trans[0], trans[1], trans[2])
    ray_dir = (direction[0], direction[1], direction[2])

    hit = omni.physx.get_physx_scene_query_interface().raycast_closest(origin, ray_dir, max_dist)
    if hit["hit"]:
        usdGeom = UsdGeom.Mesh.Get(get_current_stage(), hit["rigidBody"])
        distance = hit["distance"]
        return usdGeom.GetPath().pathString, distance
    return None, 10000.0
