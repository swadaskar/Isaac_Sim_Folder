# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import torch
from scipy.spatial.transform import Rotation
from pxr import Gf
from omni.isaac.core.utils.torch.tensor import create_zeros_tensor
from omni.isaac.core.utils.torch.rotations import gf_quat_to_tensor, quat_conjugate, quat_mul, quat_apply


def tf_matrices_from_poses(translations: torch.Tensor, orientations: torch.Tensor, device=None) -> torch.Tensor:
    """[summary]

    Args:
        translations (Union[np.ndarray, torch.Tensor]): translations with shape (N, 3).
        orientations (Union[np.ndarray, torch.Tensor]): quaternion representation (scalar first) with shape (N, 4).

    Returns:
        Union[np.ndarray, torch.Tensor]: transformation matrices with shape (N, 4, 4)
    """
    result = torch.zeros([orientations.shape[0], 4, 4], dtype=torch.float32, device=device)
    r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]].detach().cpu().numpy())
    result[:, :3, :3] = torch.from_numpy(r.as_matrix()).float().to(device)
    result[:, :3, 3] = translations
    result[:, 3, 3] = 1
    return result


def get_local_from_world(parent_transforms, positions, orientations, device):
    calculated_translations = create_zeros_tensor(shape=[positions.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[positions.shape[0], 4], dtype="float32", device=device)
    my_world_transforms = tf_matrices_from_poses(translations=positions, orientations=orientations, device=device)
    # TODO: vectorize this
    for i in range(positions.shape[0]):
        local_transform = torch.linalg.solve(
            torch.transpose(parent_transforms[i].to(device), 0, 1), my_world_transforms[i]
        )
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(torch.transpose(local_transform, 0, 1).tolist()))
        calculated_translations[i] = torch.tensor(transform.GetTranslation(), dtype=torch.float32, device=device)
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_translations, calculated_orientations


def get_world_from_local(parent_transforms, translations, orientations, device):
    calculated_positions = create_zeros_tensor(shape=[translations.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[translations.shape[0], 4], dtype="float32", device=device)
    my_local_transforms = tf_matrices_from_poses(translations=translations, orientations=orientations, device=device)
    # TODO: vectorize this
    for i in range(translations.shape[0]):
        world_transform = torch.matmul(parent_transforms[i], my_local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(torch.transpose(world_transform, 0, 1).tolist()))
        calculated_positions[i] = torch.tensor(transform.GetTranslation(), dtype=torch.float32, device=device)
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_positions, calculated_orientations


def get_pose(positions, orientations, device):
    if type(positions) != torch.Tensor:
        positions = torch.tensor(positions, device=device, dtype=torch.float)
    if type(orientations) != torch.Tensor:
        orientations = torch.tensor(orientations, device=device, dtype=torch.float)
    pose = torch.cat([positions.to(device), orientations.to(device)], dim=-1)
    return pose


@torch.jit.script
def get_world_from_local_position(pos_offset_local: torch.Tensor, pose_global: torch.Tensor):
    """ Convert a point from the local frame to the global frame
    Args:
        pos_offset_local: Point in local frame. Shape: [N, 3]
        pose_global: The spatial pose of this point. Shape: [N, 7]
    Returns:
        Position in the global frame. Shape: [N, 3]
    """
    quat_pos_local = torch.cat(
        [
            pos_offset_local,
            torch.zeros(pos_offset_local.shape[0], 1, dtype=torch.float32, device=pos_offset_local.device),
        ],
        dim=-1,
    )
    quat_global = pose_global[:, 3:7]
    quat_global_conj = quat_conjugate(quat_global)
    pos_offset_global = quat_mul(quat_global, quat_mul(quat_pos_local, quat_global_conj))[:, 0:3]

    result_pos_gloal = pos_offset_global + pose_global[:, 0:3]

    return result_pos_gloal


# NB: do not make this function jit, since it is passed around as an argument.
def normalise_quat_in_pose(pose):
    """Takes a pose and normalises the quaternion portion of it.

    Args:
        pose: shape N, 7
    Returns:
        Pose with normalised quat. Shape N, 7
    """
    pos = pose[:, 0:3]
    quat = pose[:, 3:7]
    quat /= torch.norm(quat, dim=-1, p=2).reshape(-1, 1)
    return torch.cat([pos, quat], dim=-1)


@torch.jit.script
def tf_inverse(q, t):
    q_inv = quat_conjugate(q)
    return q_inv, -quat_apply(q_inv, t)


@torch.jit.script
def tf_apply(q, t, v):
    return quat_apply(q, v) + t


@torch.jit.script
def tf_vector(q, v):
    return quat_apply(q, v)


@torch.jit.script
def tf_combine(q1, t1, q2, t2):
    return quat_mul(q1, q2), quat_apply(q1, t2) + t1
