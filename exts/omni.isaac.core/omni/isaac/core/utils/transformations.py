# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
import torch
import numpy as np
from scipy.spatial.transform import Rotation
from typing import Union, Tuple, Sequence

# omniverse
from pxr import Gf, Usd, UsdGeom

# isaacsim
from omni.isaac.core.utils.rotations import gf_quat_to_np_array
from omni.isaac.core.simulation_context.simulation_context import SimulationContext


def tf_matrix_from_pose(translation: Sequence[float], orientation: Sequence[float]) -> np.ndarray:
    """Compute input pose to transformation matrix.

    Args:
        pos (Sequence[float]): The translation vector.
        rot (Sequence[float]): The orientation quaternion.

    Returns:
        np.ndarray: A 4x4 matrix.
    """
    translation = np.asarray(translation)
    orientation = np.asarray(orientation)
    mat = Gf.Transform()
    mat.SetRotation(Gf.Rotation(Gf.Quatd(*orientation.tolist())))
    mat.SetTranslation(Gf.Vec3d(*translation.tolist()))
    return np.transpose(mat.GetMatrix())


def pose_from_tf_matrix(transformation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Gets pose corresponding to input transformation matrix.

    Args:
        transformation (np.ndarray): Column-major transformation matrix. shape is (4, 4).

    Returns:
        Tuple[np.ndarray, np.ndarray]: first index is translation corresponding to transformation. shape is (3, ).
                                       second index is quaternion orientation corresponding to transformation.
                                       quaternion is scalar-first (w, x, y, z). shape is (4, ).
    """
    mat = Gf.Transform()
    mat.SetMatrix(Gf.Matrix4d(np.transpose(transformation)))
    calculated_translation = np.array(mat.GetTranslation())
    calculated_orientation = gf_quat_to_np_array(mat.GetRotation().GetQuat())
    return calculated_translation, calculated_orientation


def tf_matrices_from_poses(
    translations: Union[np.ndarray, torch.Tensor], orientations: Union[np.ndarray, torch.Tensor]
) -> Union[np.ndarray, torch.Tensor]:
    """[summary]

    Args:
        translations (Union[np.ndarray, torch.Tensor]): translations with shape (N, 3).
        orientations (Union[np.ndarray, torch.Tensor]): quaternion representation (scalar first) with shape (N, 4).

    Returns:
        Union[np.ndarray, torch.Tensor]: transformation matrices with shape (N, 4, 4)
    """
    # TODO: add a torch pathway
    backend = "numpy"
    if SimulationContext.instance() is not None:
        backend = SimulationContext.instance().backend
    if backend == "numpy":
        result = np.zeros([orientations.shape[0], 4, 4], dtype=np.float32)
        r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]])
        result[:, :3, :3] = r.as_matrix()
        result[:, :3, 3] = translations
        result[:, 3, 3] = 1
    elif backend == "torch":
        device = None
        if SimulationContext.instance() is not None:
            device = SimulationContext.instance().device
        result = torch.zeros([orientations.shape[0], 4, 4], dtype=torch.float32, device=device)
        r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]].detach().cpu().numpy())
        result[:, :3, :3] = torch.from_numpy(r.as_matrix()).float().to(device)
        result[:, :3, 3] = translations
        result[:, 3, 3] = 1
    return result


def get_relative_transform(source_prim: Usd.Prim, target_prim: Usd.Prim) -> np.ndarray:
    """Get the relative transformation matrix from the source prim to the target prim.

    Args:
        source_prim (Usd.Prim): source prim from which frame to compute the relative transform.
        target_prim (Usd.Prim): target prim to which frame to compute the relative transform.
    
    Returns:
        np.ndarray: Column-major transformation matrix with shape (4, 4).
    """

    # Row-major transformation matrix
    source_to_world_row_major_tf = UsdGeom.Xformable(source_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    target_to_world_row_major_tf = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # Convert to column-major transformation matrix
    source_to_world_column_major_tf = np.transpose(source_to_world_row_major_tf)
    target_to_world_column_major_tf = np.transpose(target_to_world_row_major_tf)

    world_to_target_column_major_tf = np.linalg.inv(target_to_world_column_major_tf)
    source_to_target_column_major_tf = world_to_target_column_major_tf @ source_to_world_column_major_tf

    return source_to_target_column_major_tf


def get_translation_from_target(
    translation_from_source: np.ndarray, source_prim: Usd.Prim, target_prim: Usd.Prim
) -> np.ndarray:
    """Get a translation with respect to the target's frame, from a translation in the source's frame.

    Args:
        translation_from_source (np.ndarray): translation from the frame of the prim at source_path. Shape is (3, ).
        source_prim (Usd.Prim): prim path of the prim whose frame the original/untransformed translation
                           (translation_from_source) is defined with respect to.
        target_prim (Usd.Prim): prim path of the prim whose frame corresponds to the target frame that the returned
                           translation will be defined with respect to.

    Returns:
        np.ndarray: translation with respect to the target's frame. Shape is (3, ).
    """

    translation_from_source_homogenous = np.pad(translation_from_source, ((0, 1)), constant_values=1.0)

    source_to_target = get_relative_transform(source_prim, target_prim)

    translation_from_target_homogenous = translation_from_source_homogenous @ np.transpose(source_to_target)
    translation_from_target = translation_from_target_homogenous[:-1]

    return translation_from_target


def get_world_pose_from_relative(
    coord_prim: Usd.Prim, relative_translation: np.ndarray, relative_orientation: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Get a pose defined in the world frame from a pose defined relative to the frame of the coord_prim.

    Args:
        coord_prim (Usd.Prim): path of the prim whose frame the relative pose is defined with respect to.
        relative_translation (np.ndarray): translation relative to the frame of the prim at prim_path. Shape is (3, ).
        relative_orientation (np.ndarray): quaternion orientation relative to the frame of the prim at prim_path.
                                           Quaternion is scalar-first (w, x, y, z). Shape is (4, ).

    Returns:
        Tuple[np.ndarray, np.ndarray]: first index is position in the world frame. Shape is (3, ). Second index is
                                       quaternion orientation in the world frame. Quaternion is scalar-first
                                       (w, x, y, z). Shape is (4, ).
    """

    # Row-major transformation matrix from the prim's coordinate system to the world coordinate system
    prim_transform_matrix = UsdGeom.Xformable(coord_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # Convert transformation matrix to column-major
    prim_to_world = np.transpose(prim_transform_matrix)

    # Column-major transformation matrix from the pose to the frame the pose is defined with respect to
    relative_pose_to_prim = tf_matrix_from_pose(relative_translation, relative_orientation)

    # Chain the transformations
    relative_pose_to_world = prim_to_world @ relative_pose_to_prim

    # Translation and quaternion with respect to the world frame of the relatively defined pose
    world_position, world_orientation = pose_from_tf_matrix(relative_pose_to_world)

    return world_position, world_orientation
