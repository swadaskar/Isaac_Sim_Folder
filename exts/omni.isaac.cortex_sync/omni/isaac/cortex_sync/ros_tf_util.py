# Copyright (c) 2020-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import print_function

import copy
import math
import numpy as np
from numpy.linalg import norm
from typing import Tuple

import rospy

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, Pose, PoseStamped, Point

import omni.isaac.cortex.math_util as math_util


class Vec3Type:
    """ A simple 3D vector type with x,y,z fields.

    This class is used mainly as an interface with signature similar to many message types, for
    instance from ROS.

    Args:
        x: The x component.
        y: The y component.
        z: The z component.
    """

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


def numpy_vec(vec3: Vec3Type) -> np.ndarray:
    """ Create a 3D numpy vector from a vec3 message type.

    vec3 can be any type with fields x, y, z. Returns numpy vector with
    elements [x,y,z].

    Args:
        vec3: A 3D vector type with fields x,y,z.

    Returns: A numpy array representing the 3D vector (x,y,z).
    """
    v = np.array([vec3.x, vec3.y, vec3.z])
    return v


def transform_msg_to_pq(transform_msg: Transform) -> Tuple[np.ndarray, np.ndarray]:
    """ Convert a ROS geometry_msgs/Transform message type to a (position, quaternion) tuple.

    Args:
        transform_msg: The message to convert.

    Returns: A pair (p,q) with 3D position p and 4D quaternion q corresponding to the information
        in the provided transform message.
    """
    p_msg = transform_msg.translation
    p = np.array([p_msg.x, p_msg.y, p_msg.z])

    q_msg = transform_msg.rotation
    q = np.array([q_msg.w, q_msg.x, q_msg.y, q_msg.z])

    return p, q


def transform_msg_to_T(transform_msg: Transform) -> np.ndarray:
    """ Convert a ROS geometry_msgs/Transform message type to a homogeneous transform matrix.

    Args:
        transform_msg: The message to convert.

    Returns: A 4x4 homogeneous transform matrix encoding the information in the provided
        transform message.
    """
    p, q = transform_msg_to_pq(transform_msg)
    T = math_util.pq2T(p, q)
    return T


def pack_transform_stamped(T: np.ndarray, frame_name: str, in_coords: str, stamp: rospy.Time) -> TransformStamped:
    """ Pack the provided homogeneous transform matrix T, along with meta information, into a
    geometry_msgs/TransformStamped message.

    Args:
        T: homogeneous transform matrix representing the transform.
        frame_name: msg.child_frame_id (name of the object)
        in_coords: msg.header.frame_id (coordinates the object frame is described in)
        stamp: msg.header.stamp (timestamp of the transform)

    Returns: A TransformStamped message containing all of the provided information.
    """
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = stamp
    transform_stamped.header.frame_id = in_coords
    transform_stamped.child_frame_id = frame_name

    R, p = math_util.unpack_T(T)
    q = math_util.matrix_to_quat(R)

    p_msg = Vector3()
    p_msg.x = p[0]
    p_msg.y = p[1]
    p_msg.z = p[2]

    q_msg = Quaternion()
    q_msg.w = q[0]
    q_msg.x = q[1]
    q_msg.y = q[2]
    q_msg.z = q[3]
    transform_stamped.transform.translation = p_msg
    transform_stamped.transform.rotation = q_msg
    return transform_stamped
