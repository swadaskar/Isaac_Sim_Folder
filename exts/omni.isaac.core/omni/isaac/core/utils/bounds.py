# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
import typing
import numpy as np

# omniverse
import carb
from pxr import UsdGeom, Usd, Gf

# isaacsim
from omni.isaac.core.utils.prims import get_prim_at_path


def recompute_extents(
    prim: UsdGeom.Boundable, time: Usd.TimeCode = Usd.TimeCode.Default(), include_children: bool = False
) -> None:
    """Recomputes and overwrites the extents attribute for a UsdGeom.Boundable prim

    Args:
        prim (UsdGeom.Boundable): Input prim to recompute extents for
        time (Usd.TimeCode, optional): timecode to use for computing extents. Defaults to Usd.TimeCode.Default().
        include_children (bool, optional): include children of specified prim in calculation. Defaults to False.

    Raises:
        ValueError: If prim is not of UsdGeom.Boundable type
    """
    #
    def update_extents(prim: UsdGeom.Boundable, time: Usd.TimeCode = Usd.TimeCode.Default()):
        compute_prim = UsdGeom.Boundable(prim)
        if compute_prim:
            bounds = []
            mesh = UsdGeom.Mesh(compute_prim)
            if mesh:
                bounds = mesh.ComputeExtent(mesh.GetPointsAttr().Get())
            else:
                bounds = UsdGeom.Boundable.ComputeExtentFromPlugins(compute_prim, time)

            if compute_prim.GetExtentAttr().HasValue():
                compute_prim.GetExtentAttr().Set(bounds)
            else:
                compute_prim.CreateExtentAttr(bounds)
        else:
            raise ValueError(f"Input prim is not of type UsdGeom.Boundable, is instead {type(prim)}")

    if include_children:
        for p in Usd.PrimRange(prim.GetPrim()):
            try:
                update_extents(p, time)
            except ValueError:
                carb.log_info(f"Skipping {p}, not boundable")
    else:
        update_extents(prim, time)


def create_bbox_cache(time: Usd.TimeCode = Usd.TimeCode.Default(), use_extents_hint: bool = True) -> UsdGeom.BBoxCache:
    """Helper function to create a Bounding Box Cache object that can be used for computations

    Args:
        time (Usd.TimeCode, optional): time at which cache should be initialized. Defaults to Usd.TimeCode.Default().
        use_extents_hint (bool, optional): Use existing extents attribute on prim to compute bounding box. Defaults to True.

    Returns:
        UsdGeom.BboxCache: Initialized bbox cache
    """
    return UsdGeom.BBoxCache(time=time, includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=use_extents_hint)


def compute_aabb(bbox_cache: UsdGeom.BBoxCache, prim_path: str, include_children: bool = False) -> np.array:
    """Compute an AABB for a given prim_path, a combined AABB is computed if include_children is True

    Args:
        bbox_cache (UsdGeom.BboxCache): Existing Bounding box cache to use for computation
        prim_path (str): prim path to compute AABB for
        include_children (bool, optional): include children of specified prim in calculation. Defaults to False.

    Returns:
        np.array: Bounding box for this prim, [min x, min y, min z, max x, max y, max z]
    """
    total_bounds = Gf.BBox3d()
    prim = get_prim_at_path(prim_path)
    if include_children:
        for p in Usd.PrimRange(prim):
            total_bounds = Gf.BBox3d.Combine(
                total_bounds, Gf.BBox3d(bbox_cache.ComputeWorldBound(p).ComputeAlignedRange())
            )
    else:
        total_bounds = Gf.BBox3d(bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange())

    range = total_bounds.GetRange()
    return np.array([*range.GetMin(), *range.GetMax()])


def compute_combined_aabb(bbox_cache: UsdGeom.BBoxCache, prim_paths: typing.List[str]) -> np.array:
    """Computes a combined AABB given a list of prim paths

    Args:
        bbox_cache (UsdGeom.BboxCache): Existing Bounding box cache to use for computation
        prim_paths (typing.List[str]): List of prim paths to compute combined AABB for

    Returns:
        np.array: Bounding box for input prims, [min x, min y, min z, max x, max y, max z]
    """
    total_bounds = Gf.BBox3d()
    for prim_path in prim_paths:
        prim = get_prim_at_path(prim_path)
        bounds = bbox_cache.ComputeWorldBound(prim)
        total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(bounds.ComputeAlignedRange()))
    range = total_bounds.GetRange()
    return np.array([*range.GetMin(), *range.GetMax()])
