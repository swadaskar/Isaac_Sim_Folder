# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


# Solver Problem definition

# min (x.T @ x)
# s.t:
#     V.T @ x == v_input
#     cross(V,wheel_distances_to_com) @ x == w_input
#

# Import packages.
import osqp
from scipy import sparse
import numpy as np
from numpy import linalg
from pxr import Gf
import omni
import carb
from omni.isaac.core.utils.rotations import euler_to_rot_matrix, quat_to_rot_matrix
from omni.isaac.core.utils.math import cross
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController
from typing import Optional, Tuple


class HolonomicController(BaseController):
    """[summary]
    Generic Holonomic drive controller. Model must have drive joints to mecanum wheels defined in the USD with the rollers angle and radius.

    Args:
        name (str): [description]
        wheel_radius (np.ndarray): radius of the wheels, array of 1 can be used if all wheels are the same size
        wheel_positions (np.ndarray): position of the wheels relative to center of mass of the vehicle. number of wheels x [x,y,z]
        wheel_orientations (np.ndarray): orientation of the wheels in quaternions relative to center of mass frame of the vehicle. number of wheels x [quaternions]
        mecanum_angles (np.ndarray): mecanum wheel angles. Array of 1 can be used if all wheels have the same angle.
        wheel_axis (np.ndarray): the spinning axis of the wheels. Defaults to [1,0,0]
        up_axis (np.ndarray): Defaults to z- axis
    """

    def __init__(
        self,
        name: str,
        wheel_radius: Optional[np.ndarray] = None,
        wheel_positions: Optional[np.ndarray] = None,
        wheel_orientations: Optional[np.ndarray] = None,
        mecanum_angles: Optional[np.ndarray] = None,
        wheel_axis: float = np.array([1, 0, 0]),
        up_axis: float = np.array([0, 0, 1]),  # default to z_axis
        max_linear_speed: float = 1.0e20,
        max_angular_speed: float = 1.0e20,
        max_wheel_speed: float = 1.0e20,
        linear_gain: float = 1.0,
        angular_gain: float = 1.0,
    ) -> None:
        super().__init__(name)

        self.num_wheels = len(wheel_positions)
        if len(wheel_radius) == 1:
            self.wheel_radius = [wheel_radius] * self.num_wheels
        else:
            self.wheel_radius = wheel_radius
        self.wheel_positions = np.asarray(wheel_positions)
        self.wheel_orientations = np.asarray(wheel_orientations)
        if len(mecanum_angles) == 1:
            self.mecanum_angles = [mecanum_angles] * self.num_wheels
        else:
            self.mecanum_angles = mecanum_angles
        self.wheel_axis = wheel_axis
        self.up_axis = up_axis
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_wheel_speed = (max_wheel_speed,)
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain

        self.joint_commands = np.zeros((self.num_wheels, 0), dtype=float)
        self.build_base()

    def build_base(self):
        self.base_dir = np.zeros((3, self.num_wheels), dtype=float)
        self.wheel_dists_inv = np.zeros((3, self.num_wheels), dtype=float)

        for i in range(self.num_wheels):
            p_0 = self.wheel_positions[i]
            r_0 = quat_to_rot_matrix(self.wheel_orientations[i])

            joint_pose = np.zeros((4, 4))
            joint_pose[:3, :3] = r_0.T
            joint_pose[3, :3] = p_0
            joint_pose[3, 3] = 1

            mecanum_angle = self.mecanum_angles[i]
            mecanum_radius = self.wheel_radius[i]
            m_rot = euler_to_rot_matrix(Gf.Vec3d(*self.up_axis.tolist()) * mecanum_angle, True)
            j_axis = Gf.Vec3f(
                m_rot.TransformDir(Gf.Matrix4f(joint_pose).TransformDir(Gf.Vec3d(*self.wheel_axis.tolist())))
            ).GetNormalized()

            self.base_dir[0, i] = j_axis[0] * mecanum_radius
            self.base_dir[1, i] = j_axis[1] * mecanum_radius
            for k in range(2):
                self.wheel_dists_inv[k, i] = p_0[k]

        self.P = sparse.csc_matrix(np.diag(self.wheel_radius) / np.linalg.norm(self.wheel_radius))
        self.b = sparse.csc_matrix(np.zeros((6, 1)))
        V = self.base_dir
        W = np.cross(V, self.wheel_dists_inv, axis=0)
        self.A = sparse.csc_matrix(np.concatenate((V, W), axis=0))
        self.l = np.array([0.0, 0.0, -np.inf, -np.inf, -np.inf, 0.0])
        self.u = np.array([0.0, 0.0, np.inf, np.inf, np.inf, 0.0])

        self.prob = osqp.OSQP()

        self.prob.setup(self.P, A=self.A, l=self.l, u=self.u, verbose=False)

        self.prob.solve()

    def forward(self, command: np.ndarray) -> ArticulationAction:
        """Calculating the wheels speed given the desired vehicle speed.

        Args:
            command (np.ndarray): [forward_velocity, lateral_velocity, yaw_velocity].

        Returns:
            ArticulationAction: [description]
        """
        if isinstance(command, list):
            command = np.array(command)
        if command.shape[0] != 3:
            raise Exception("command should be of length 3, delta x,y, and angular velocity")
        if (np.array(command) == 0).all():
            self.joint_commands = [float(0) for i in range(self.num_wheels)]
        else:
            v = np.array([command[0], command[1], 0]).reshape((3)) * self.linear_gain
            w = np.array([(command[2])]) * self.angular_gain

            if np.linalg.norm(v) > 0:
                v_norm = v / np.linalg.norm(v)
            else:
                v_norm = v

            if np.linalg.norm(v) > self.max_linear_speed:
                v = v_norm * self.max_linear_speed
            if np.linalg.norm(w) > self.max_angular_speed:
                w = w / abs(w) * np.array([self.max_angular_speed])

            self.l[0:2] = self.u[0:2] = v[0:2] / self.max_linear_speed
            self.l[-1] = self.u[-1] = w / self.max_linear_speed
            self.prob.update(l=self.l, u=self.u)
            res = None
            try:
                res = self.prob.solve()
            except Exception as e:
                carb.log_error("error:", e)

            if res is not None:
                values = res.x.reshape([res.x.shape[0]]) * self.max_linear_speed

                if np.max(np.abs(values)) > self.max_wheel_speed:
                    m = np.max(np.abs(values))
                    scale = self.max_wheel_speed / m
                    values = values * scale
                self.joint_commands = [float(values[i]) for i in range(self.num_wheels)]
        return ArticulationAction(joint_velocities=list(self.joint_commands))

    def reset(self) -> None:
        """[summary]
        """
        return
