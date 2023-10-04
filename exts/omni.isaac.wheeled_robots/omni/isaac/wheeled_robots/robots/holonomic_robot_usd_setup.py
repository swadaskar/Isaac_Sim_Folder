# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.rotations import gf_rotation_to_np_array
import omni
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import numpy as np
import carb


class HolonomicRobotUsdSetup:
    """[summary]
    Generic Holonomic Robot Setup. Extract from USD or compile from user input the necessary information for holonomic controller.
    Args:
        name (str): [description]
        prim_path (str): path of the robot articulation
        com_prim_path (str): path of the xform representing the center of mass of the vehicle
    """

    def __init__(self, robot_prim_path: str, com_prim_path: str):
        self._robot_prim_path = robot_prim_path
        self._com_prim_path = com_prim_path  # TODO: make this one of USD attribute
        self.from_usd(self._robot_prim_path, self._com_prim_path)

    def from_usd(self, robot_prim_path, com_prim_path):
        """
           if the USD contains all the necessary information, automatically extract them and compile 
        """
        stage = get_current_stage()
        robot_prim = get_prim_at_path(robot_prim_path)
        if self._com_prim_path == "":
            com_prim = robot_prim  # if no com prim given, assume robot root prim is also com prim
        else:
            com_prim = get_prim_at_path(com_prim_path)

        self._mecanum_joints = [j for j in Usd.PrimRange(robot_prim) if j.GetAttribute("isaacmecanumwheel:angle")]
        self._num_wheels = len(self._mecanum_joints)
        self._wheel_radius = [j.GetAttribute("isaacmecanumwheel:radius").Get() for j in self._mecanum_joints]
        self._mecanum_angles = [j.GetAttribute("isaacmecanumwheel:angle").Get() for j in self._mecanum_joints]
        self._wheel_dof_names = [j.GetName() for j in self._mecanum_joints]
        self._wheel_positions = np.zeros((self._num_wheels, 3), dtype=float)  ## xyz for position
        self._wheel_orientations = np.zeros((self._num_wheels, 4), dtype=float)  ## quaternion for orientation
        com_pose = Gf.Matrix4f(omni.usd.utils.get_world_transform_matrix(com_prim))
        for i, j in enumerate(self._mecanum_joints):
            joint = UsdPhysics.RevoluteJoint(j)
            chassis_prim = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
            chassis_pose = Gf.Matrix4f(omni.usd.utils.get_world_transform_matrix(chassis_prim))
            p_0 = joint.GetLocalPos0Attr().Get()
            r_0 = joint.GetLocalRot0Attr().Get()
            local_0 = Gf.Matrix4f()
            local_0.SetTranslate(p_0)
            local_0.SetRotateOnly(r_0)
            joint_pose = local_0 * chassis_pose
            self._wheel_positions[i, :] = joint_pose.ExtractTranslation() - com_pose.ExtractTranslation()
            self._wheel_orientations[i, :] = gf_rotation_to_np_array(
                joint_pose.ExtractRotation() * ((com_pose.ExtractRotation()).GetInverse())
            )

        axis = {"X": np.array([1, 0, 0]), "Y": np.array([0, 1, 0]), "Z": np.array([0, 0, 1])}
        self._up_axis = axis[UsdGeom.GetStageUpAxis(stage)]
        self._wheel_axis = axis[joint.GetAxisAttr().Get()]

    def get_holonomic_controller_params(self):
        return (
            self._wheel_radius,
            self._wheel_positions,
            self._wheel_orientations,
            self._mecanum_angles,
            self._wheel_axis,
            self._up_axis,
        )

    def get_articulation_controller_params(self):
        return self._wheel_dof_names

    @property
    def wheel_radius(self):
        return self._wheel_radius

    @property
    def wheel_positions(self):
        return self._wheel_positions

    @property
    def wheel_orientations(self):
        return self._wheel_orientations

    @property
    def mecanum_angles(self):
        return self._mecanum_angles

    @property
    def wheel_dof_names(self):
        return self._wheel_dof_names

    @property
    def wheel_axis(self):
        return self._wheel_axis

    @property
    def up_axis(self):
        return self._up_axis
