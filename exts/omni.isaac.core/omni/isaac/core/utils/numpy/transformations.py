# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import numpy as np
from pxr import Gf
from scipy.spatial.transform import Rotation
from omni.isaac.core.utils.numpy.tensor import create_zeros_tensor
from omni.isaac.core.utils.numpy.rotations import gf_quat_to_tensor


def tf_matrices_from_poses(translations: np.ndarray, orientations: np.ndarray, device=None) -> np.ndarray:
    """[summary]

    Args:
        translations (Union[np.ndarray, torch.Tensor]): translations with shape (N, 3).
        orientations (Union[np.ndarray, torch.Tensor]): quaternion representation (scalar first) with shape (N, 4).

    Returns:
        Union[np.ndarray, torch.Tensor]: transformation matrices with shape (N, 4, 4)
    """

    result = np.zeros([orientations.shape[0], 4, 4], dtype=np.float32)
    r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]])
    result[:, :3, :3] = r.as_matrix()
    result[:, :3, 3] = translations
    result[:, 3, 3] = 1
    return result


def get_local_from_world(parent_transforms, positions, orientations, device=None):
    calculated_translations = create_zeros_tensor(shape=[positions.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[positions.shape[0], 4], dtype="float32", device=device)
    my_world_transforms = tf_matrices_from_poses(translations=positions, orientations=orientations)
    # TODO: vectorize this
    for i in range(positions.shape[0]):
        local_transform = np.matmul(np.linalg.inv(np.transpose(parent_transforms[i])), my_world_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(np.transpose(local_transform).tolist()))
        calculated_translations[i] = np.array(transform.GetTranslation())
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_translations, calculated_orientations


def get_world_from_local(parent_transforms, translations, orientations, device=None):
    calculated_positions = create_zeros_tensor(shape=[translations.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[translations.shape[0], 4], dtype="float32", device=device)
    my_local_transforms = tf_matrices_from_poses(translations=translations, orientations=orientations)
    # TODO: vectorize this
    for i in range(translations.shape[0]):
        world_transform = np.matmul(parent_transforms[i], my_local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(np.transpose(world_transform).tolist()))
        calculated_positions[i] = np.array(transform.GetTranslation())
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_positions, calculated_orientations


def get_pose(positions, orientations, device=None):
    pose = np.concatenate([positions, orientations], axis=-1)
    return pose
