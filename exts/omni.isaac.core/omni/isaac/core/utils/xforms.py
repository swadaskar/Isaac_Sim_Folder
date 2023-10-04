# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import Usd, UsdGeom, Gf
import numpy as np


def clear_xform_ops(prim: Usd.Prim):
    """ Remove all xform ops from input prim.

    Args:
        prim (Usd.Prim): The input USD prim.
    """
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    # Remove any authored transform properties
    authored_prop_names = prim.GetAuthoredPropertyNames()
    for prop_name in authored_prop_names:
        if prop_name.startswith("xformOp:"):
            prim.RemoveProperty(prop_name)


def reset_and_set_xform_ops(
    prim: Usd.Prim, translation: Gf.Vec3d, orientation: Gf.Quatd, scale: Gf.Vec3d = Gf.Vec3d([1.0, 1.0, 1.0])
):
    """Reset xform ops to isaac sim defaults, and set their values

    Args:
        prim (Usd.Prim): Prim to reset
        translation (Gf.Vec3d): translation to set
        orientation (Gf.Quatd): orientation to set
        scale (Gf.Vec3d, optional): scale to set. Defaults to Gf.Vec3d([1.0, 1.0, 1.0]).
    """
    xformable = UsdGeom.Xformable(prim)
    clear_xform_ops(prim)

    xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_scale.Set(scale)

    xform_op_tranlsate = xformable.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_tranlsate.Set(translation)

    xform_op_rot = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_rot.Set(orientation)

    xformable.SetXformOpOrder([xform_op_tranlsate, xform_op_rot, xform_op_scale])


def reset_xform_ops(prim: Usd.Prim):
    """Reset xform ops for a prim to isaac sim defaults, 

    Args:
        prim (Usd.Prim): Prim to reset xform ops on
    """
    properties = prim.GetPropertyNames()
    xformable = UsdGeom.Xformable(prim)
    # get current position and orientation
    T_p_w = xformable.ComputeParentToWorldTransform(Usd.TimeCode.Default())
    T_l_w = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    T_l_p = Gf.Transform()
    T_l_p.SetMatrix(Gf.Matrix4d(np.matmul(T_l_w, np.linalg.inv(T_p_w)).tolist()))
    current_translation = T_l_p.GetTranslation()
    current_orientation = T_l_p.GetRotation().GetQuat()

    reset_and_set_xform_ops(current_translation, current_orientation)
