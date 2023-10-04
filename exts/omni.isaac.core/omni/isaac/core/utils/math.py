# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import copy
import numpy as np
from typing import Union


def radians_to_degrees(rad_angles: np.ndarray) -> np.ndarray:
    """Converts input angles from radians to degrees.

    Args:
        rad_angles (np.ndarray): Input array of angles (in radians).

    Returns:
        np.ndarray: Array of angles in degrees.
    """
    return rad_angles * (180.0 / np.pi)


def cross(a: Union[np.ndarray, list], b: Union[np.ndarray, list]) -> list:
    """Computes the cross-product between two 3-dimensional vectors.

    Args:
        a (np.ndarray, list): A 3-dimensional vector
        b (np.ndarray, list): A 3-dimensional vector

    Returns:
        np.ndarray: Cross product between input vectors.
    """
    return [a[1] * b[2] - a[2] * b[1], a[0] * b[2] - a[2] * b[0], a[0] * b[1] - a[1] * b[0]]


def normalize(v):
    """ Normalizes the vector inline (and also returns it). """
    v /= np.linalg.norm(v)
    return v


def normalized(v):
    """ Returns a normalized copy of the provided vector. """
    if v is None:
        return None
    return normalize(copy.deepcopy(v))
