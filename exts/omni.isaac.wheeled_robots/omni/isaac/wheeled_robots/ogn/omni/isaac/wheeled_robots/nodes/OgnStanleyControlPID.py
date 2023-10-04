# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import math
import omni
import numpy as np
import omni.graph.core as og
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.wheeled_robots.ogn.OgnStanleyControlPIDDatabase import OgnStanleyControlPIDDatabase
from omni.isaac.wheeled_robots.controllers.stanley_control import State, pid_control, stanley_control, normalize_angle


class OgnStanleyControlPIDInternalState(BaseResetNode):
    def __init__(self):

        self.target_idx = 0  # path array index
        # store target pos to prevent repeated & unnecessary db.inputs.target access
        self.target = [0, 0, 0]  # [x, y, z_rot]
        self.rv = []  # store path arrays to avoid repeated inputs access
        self.rx = []
        self.ry = []
        self.ryaw = []
        self.sp = []  # speed profile, used for linear speed control and path drawing
        self.argb = []  # stores color info for path drawing
        self.thresholds = (
            []
        )  # threshold to switch to rotate only - only [0] (distance) value is used, expected to be same double[2] array as CheckGoal2D node's thresholds input

        self.wb = 0  # save wheelBase and step to prevent unnecessary input calls
        self.s = 0

        super().__init__(initialize=False)

    def custom_reset(self):  # reset all saved values to prevent carrying over into different run
        self.target_idx = 0
        self.target = [0, 0, 0]
        self.rv = []
        self.rx = []
        self.ry = []
        self.ryaw = []
        self.sp = []
        self.thresholds = []

        self.wb = 0
        self.s = 0


class OgnStanleyControlPID:
    @staticmethod
    def initialize(graph_context, node):
        db = OgnStanleyControlPIDDatabase(node)
        state = OgnStanleyControlPIDDatabase.per_node_internal_state(node)
        state.outputs = db.outputs

    @staticmethod
    def internal_state():
        return OgnStanleyControlPIDInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state

        # save thresholds if changed
        state.thresholds = db.inputs.thresholds

        # pull reachedGoal input from CheckGoal2D node
        reachedGoal = db.inputs.reachedGoal
        if (
            reachedGoal[0] and reachedGoal[1]
        ):  # if target pos & rot reached, stop movement (and allow future nodes to run)
            db.outputs.linearVelocity = 0
            db.outputs.angularVelocity = 0
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return True

        # get pos/rot/velocity data
        pos = db.inputs.currentPosition
        x = pos[0]
        y = pos[1]
        _, _, rot = quatd4_to_euler(db.inputs.currentOrientation)
        cs = db.inputs.currentSpeed
        v = np.hypot(cs[0], cs[1])

        if db.inputs.targetChanged:  # if retargeted
            state.target_idx = 0  # reset path array index
            state.target = db.inputs.target  # store new target

            pathArrays = db.inputs.pathArrays  # pull new path arrays from QuinticPathPlanner
            arr_length = int(
                len(pathArrays) / 4
            )  # since rv, rx, ry, and ryaw are concatenated together, each array is 1/4 length of full input array

            # separate into component path arrays
            state.rv = np.array(pathArrays[0:arr_length])
            state.rx = np.array(pathArrays[arr_length : arr_length * 2])
            state.ry = np.array(pathArrays[arr_length * 2 : arr_length * 3])
            state.ryaw = np.array(pathArrays[arr_length * 3 : arr_length * 4])

            # calculate speed profile with suggested/arbitrary target & min speed values
            state.sp = calc_speed_profile(np.array(state.rv), db.inputs.maxVelocity, 0.5, 0.05)

            # save rgb path curve if user switches on path drawing mid-simulation
            state.argb = draw_path_setup(state.sp)

        # check if rotate_only using distance threshold
        state.rotate_only = np.hypot(x - state.target[0], y - state.target[1]) <= state.thresholds[0] or reachedGoal[0]

        # store wheelBase and step to prevent unnecessary input calls
        if state.wb == 0:
            state.wb = db.inputs.wheelBase
            state.s = db.inputs.step

        # if wheelBase or step are 0, divide by 0 errors will occur
        if state.wb == 0:
            print("Error: wheel base is 0!")
            return False
        elif state.s == 0:
            print("Error: step is 0!")
            return False

        gains = db.inputs.gains
        K = gains[0]
        Kp = gains[1]
        Ks = gains[2]
        # create new stanley control State object to store current odometry info about robot
        stanley_state = State(state.wb * Kp, x=x, y=y, yaw=rot % (2 * np.pi), v=v, Ks=Ks)

        if not state.rotate_only:  # if driving & steering is needed
            ai = pid_control(state.sp[state.target_idx], stanley_state.v, Kp) / state.s  # linear acceleration
            di, state.target_idx = stanley_control(
                stanley_state, state.rx, state.ry, state.ryaw, state.target_idx, K
            )  # delta rot and path array index closest to current pos/rot

            stanley_state.update(
                ai, di, state.s
            )  # use acceleration and delta rot values to determine linear and angular velocity outputs
            v = stanley_state.v  # save linear and angular velocity outputs
            w = stanley_state.w

        else:  # if position reached but not rotation
            v = 0  # stop linear velocity
            theta_diff = math.atan2(
                math.sin(state.target[2] - rot), math.cos(state.target[2] - rot)
            )  # find diff between current rotation and target rotation

            # rotation direction determined by pos/neg sign of theta_diff, rotation magnitude limited to 1
            if theta_diff > 0:
                w = min(((theta_diff) * Kp / state.s), 1)
            else:
                w = max(((theta_diff) * Kp / state.s), -1)

        kw = 1
        # Allow additional steering to use differential drive (backwards spin on one wheel to tighten the cornering radius)
        if not reachedGoal[0] and v > 0:
            kw = 1 + abs((state.wb * w) / v) * (1 * Kp / state.s)

        # output linear/angular velocity values
        db.outputs.linearVelocity = v
        db.outputs.angularVelocity = kw * w

        # begin next node (Differential and Angular controllers, if configured)
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        if (
            db.inputs.drawPath
        ):  # if user enables path drawing, draw the path using previously computed color values and path arrays
            draw_path(state.rx, state.ry, state.argb)

        return True


def quatd4_to_euler(orientation):
    # implementation for quat_to_euler_angles that normalizes outputs
    x, y, z, w = tuple(orientation)
    roll, pitch, yaw = quat_to_euler_angles(np.array([w, x, y, z]))

    return normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)


def calc_speed_profile(
    cyaw, max_speed, target_speed, min_speed=1
):  # calculate speed profile (copied from simple_robot_controller in demo utils)
    max_c = max([abs(c) for c in cyaw])
    if max_c == 0:
        print("Error: max yaw is 0!")
        return False

    speed_profile = np.array(cyaw) / max_c * max_speed

    # speed down
    res = min(int(len(cyaw) / 3), int(max_speed * 60))

    for i in range(1, res):
        speed_profile[-i] = min(speed_profile[-i], speed_profile[-i] / (float(res - i)) ** 0.5)  # / (res))
        if speed_profile[-i] <= min_speed:
            speed_profile[-i] = min_speed

    return speed_profile


def draw_path_setup(sp):  # use speed profile to create color array (copied from simple_robot_controller in demo utils)
    color = [(0, t / np.max(sp), 0) for t in sp]
    rgb_bytes = [(np.clip(c, 0, 1.0) * 255).astype("uint8").tobytes() for c in color]
    argb_bytes = [b"\xff" + b for b in rgb_bytes]
    argb = [int.from_bytes(b, byteorder="big") for b in argb_bytes]

    return argb


def draw_path(
    rx, ry, argb
):  # use color and path arrays to draw path along quintic curve (copied from simple_robot_controller in demo utils)
    from omni.debugdraw import _debugDraw
    import carb
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    stage_unit = UsdGeom.GetStageMetersPerUnit(stage)

    for i in range(len(rx) - 1):
        _debugDraw.acquire_debug_draw_interface().draw_line(
            carb.Float3(rx[i] / stage_unit, ry[i] / stage_unit, 0.14 / stage_unit),
            argb[i],
            carb.Float3(rx[i + 1] / stage_unit, ry[i + 1] / stage_unit, 0.14 / stage_unit),
            argb[i - 1],
        )
