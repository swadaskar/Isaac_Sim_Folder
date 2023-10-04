# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import Gf
from omni.isaac.dynamic_control import _dynamic_control


def _vec3d_quatd_to_dctransform(translation: Gf.Vec3d, quat: Gf.Quatd) -> _dynamic_control.Transform:
    pose_t = (translation[0], translation[1], translation[2])
    pose_r = (quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2], quat.GetReal())
    return _dynamic_control.Transform(pose_t, pose_r)


def create_transform(translation, rotation) -> _dynamic_control.Transform:
    if isinstance(rotation, Gf.Rotation):
        return _vec3d_quatd_to_dctransform(translation, rotation.GetQuat())
    if isinstance(rotation, Gf.Quatd):
        return _vec3d_quatd_to_dctransform(translation, rotation)


def create_transform_from_mat(mat: Gf.Matrix4d) -> _dynamic_control.Transform:
    trans = mat.ExtractTranslation()
    q = mat.ExtractRotation().GetQuaternion()
    (q_x, q_y, q_z) = q.GetImaginary()
    quat = [q_x, q_y, q_z, q.GetReal()]
    tr = _dynamic_control.Transform()
    tr.p = trans
    tr.r = quat
    return tr
