# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np

# omniverse
from pxr import Usd, UsdGeom

# isaacsim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.transformations import get_relative_transform


def get_mesh_vertices_relative_to(mesh_prim: UsdGeom.Mesh, coord_prim: Usd.Prim) -> np.ndarray:
    """Get vertices of the mesh prim in the coordinate system of the given prim.

    Args:
        mesh_prim (UsdGeom.Mesh): mesh prim to get the vertice points.
        coord_prim (Usd.Prim): prim used as relative coordinate.

    Returns:
        np.ndarray: vertices of the mesh in the coordinate system of the given prim. Shape is (N, 3).
    """

    # Vertices of the mesh in the mesh's coordinate system
    vertices_vec3f = UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()
    vertices = np.array(vertices_vec3f)
    vertices_tf_row_major = np.pad(vertices, ((0, 0), (0, 1)), constant_values=1.0)

    # Transformation matrix from the coordinate system of the mesh to the coordinate system of the prim
    relative_tf_column_major = get_relative_transform(mesh_prim, coord_prim)
    relative_tf_row_major = np.transpose(relative_tf_column_major)

    # Transform points so they are in the coordinate system of the top-level ancestral xform prim
    points_in_relative_coord = vertices_tf_row_major @ relative_tf_row_major

    points_in_meters = points_in_relative_coord[:, :-1] * get_stage_units()

    return points_in_meters
