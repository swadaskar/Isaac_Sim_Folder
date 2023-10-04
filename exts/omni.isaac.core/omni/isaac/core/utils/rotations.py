# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
import math
import typing
import numpy as np

# omniverse
from pxr import Gf

# internal global constants
_FLOAT_EPS = np.finfo(np.float32).eps
_EPS4 = _FLOAT_EPS * 4.0


def quat_to_rot_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert input quaternion to rotation matrix.

    Args:
        quat (np.ndarray): Input quaternion (w, x, y, z).

    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    # might need to be normalized
    rotm = Gf.Matrix3f(Gf.Quatf(*quat.tolist())).GetTranspose()
    return np.array(rotm)


def matrix_to_euler_angles(mat: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to Euler XYZ angles.

    Args:
        mat (np.ndarray): A 3x3 rotation matrix.

    Returns:
        np.ndarray: Euler XYZ angles (in radians).
    """
    cy = np.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0])
    singular = cy < _EPS4
    if not singular:
        roll = math.atan2(mat[2, 1], mat[2, 2])
        pitch = math.atan2(-mat[2, 0], cy)
        yaw = math.atan2(mat[1, 0], mat[0, 0])
    else:
        roll = math.atan2(-mat[1, 2], mat[1, 1])
        pitch = math.atan2(-mat[2, 0], cy)
        yaw = 0
    return np.array([roll, pitch, yaw])


def quat_to_euler_angles(quat: np.ndarray, degrees: bool = False) -> np.ndarray:
    """Convert input quaternion to Euler XYZ matrix.

    Args:
        quat (np.ndarray): Input quaternion (w, x, y, z).
        degrees (bool, optional): Whether returned angles should be in degrees.

    Returns:
        np.ndarray: Euler XYZ angles (in radians).
    """
    rpy = matrix_to_euler_angles(quat_to_rot_matrix(quat))
    if degrees:
        return np.rad2deg(rpy)
    else:
        return rpy


def euler_angles_to_quat(euler_angles: np.ndarray, degrees: bool = False) -> np.ndarray:
    """Convert Euler XYZ angles to quaternion.

    Args:
        euler_angles (np.ndarray):  Euler XYZ angles.
        degrees (bool, optional): Whether input angles are in degrees. Defaults to False.

    Returns:
        np.ndarray: quaternion (w, x, y, z).
    """
    roll, pitch, yaw = euler_angles
    if degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

    cr = np.cos(roll / 2.0)
    sr = np.sin(roll / 2.0)
    cy = np.cos(yaw / 2.0)
    sy = np.sin(yaw / 2.0)
    cp = np.cos(pitch / 2.0)
    sp = np.sin(pitch / 2.0)
    w = (cr * cp * cy) + (sr * sp * sy)
    x = (sr * cp * cy) - (cr * sp * sy)
    y = (cr * sp * cy) + (sr * cp * sy)
    z = (cr * cp * sy) - (sr * sp * cy)
    return np.array([w, x, y, z])


def euler_to_rot_matrix(euler_angles: np.ndarray, degrees: bool = False) -> Gf.Rotation:
    """Convert from Euler XYZ angles to rotation matrix.

    Args:
        euler_angles (np.ndarray): Euler XYZ angles.
        degrees (bool, optional): Whether input angles are in degrees. Defaults to False.

    Returns:
        Gf.Rotation: Pxr rotation object.
    """
    return Gf.Rotation(Gf.Quatf(*euler_angles_to_quat(euler_angles, degrees)))


def lookat_to_quatf(camera: Gf.Vec3f, target: Gf.Vec3f, up: Gf.Vec3f) -> Gf.Quatf:
    """[summary]

    Args:
        camera (Gf.Vec3f): [description]
        target (Gf.Vec3f): [description]
        up (Gf.Vec3f): [description]

    Returns:
        Gf.Quatf: Pxr quaternion object.
    """
    F = (target - camera).GetNormalized()
    R = Gf.Cross(up, F).GetNormalized()
    U = Gf.Cross(F, R)

    q = Gf.Quatf()
    trace = R[0] + U[1] + F[2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        q = Gf.Quatf(0.25 / s, Gf.Vec3f((U[2] - F[1]) * s, (F[0] - R[2]) * s, (R[1] - U[0]) * s))
    else:
        if R[0] > U[1] and R[0] > F[2]:
            s = 2.0 * math.sqrt(1.0 + R[0] - U[1] - F[2])
            q = Gf.Quatf((U[2] - F[1]) / s, Gf.Vec3f(0.25 * s, (U[0] + R[1]) / s, (F[0] + R[2]) / s))
        elif U[1] > F[2]:
            s = 2.0 * math.sqrt(1.0 + U[1] - R[0] - F[2])
            q = Gf.Quatf((F[0] - R[2]) / s, Gf.Vec3f((U[0] + R[1]) / s, 0.25 * s, (F[1] + U[2]) / s))
        else:
            s = 2.0 * math.sqrt(1.0 + F[2] - R[0] - U[1])
            q = Gf.Quatf((R[1] - U[0]) / s, Gf.Vec3f((F[0] + R[2]) / s, (F[1] + U[2]) / s, 0.25 * s))
    return q


def gf_quat_to_np_array(orientation: typing.Union[Gf.Quatd, Gf.Quatf, Gf.Quaternion]) -> np.ndarray:
    """Converts a pxr Quaternion type to a numpy array following [w, x, y, z] convention.

    Args:
        orientation (typing.Union[Gf.Quatd, Gf.Quatf, Gf.Quaternion]): Input quaternion object.

    Returns:
        np.ndarray: A (4,) quaternion array in (w, x, y, z).
    """
    quat = np.zeros(4)
    quat[1:] = orientation.GetImaginary()
    quat[0] = orientation.GetReal()
    return quat


def gf_rotation_to_np_array(orientation: Gf.Rotation) -> np.ndarray:
    """Converts a pxr Rotation type to a numpy array following [w, x, y, z] convention.

    Args:
        orientation (Gf.Rotation): Pxr rotation object.

    Returns:
        np.ndarray: A (4,) quaternion array in (w, x, y, z).
    """
    return gf_quat_to_np_array(orientation.GetQuat())
