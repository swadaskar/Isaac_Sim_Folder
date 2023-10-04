# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import UsdGeom

AXES_INDICES = {"X": 0, "x": 0, "Y": 1, "y": 1, "Z": 2, "z": 2}
"""Mapping from axis name to axis ID."""

AXES_TOKEN = {
    "X": UsdGeom.Tokens.x,
    "x": UsdGeom.Tokens.x,
    "Y": UsdGeom.Tokens.y,
    "y": UsdGeom.Tokens.y,
    "Z": UsdGeom.Tokens.z,
    "z": UsdGeom.Tokens.z,
}
"""Mapping from axis name to axis USD token."""
