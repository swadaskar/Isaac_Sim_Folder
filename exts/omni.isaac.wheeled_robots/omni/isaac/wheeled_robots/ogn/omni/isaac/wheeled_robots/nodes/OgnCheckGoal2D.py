# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
import omni.graph.core as og
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.wheeled_robots.controllers.stanley_control import normalize_angle
from omni.isaac.wheeled_robots.ogn.OgnCheckGoal2DDatabase import OgnCheckGoal2DDatabase


class OgnCheckGoal2DInternalState(BaseResetNode):
    # modeled after OgnDifferentialController state layout
    def __init__(self):
        # store target pos to prevent repeated & unnecessary db.inputs.target access
        self.target = [0, 0, 0]  # [x, y, z_rot]
        super().__init__(initialize=False)

    def custom_reset(self):
        # reset target to origin (not an ideal reset solution but technically works)
        self.target = [0, 0, 0]


class OgnCheckGoal2D:
    @staticmethod
    def initialize(graph_context, node):
        db = OgnCheckGoal2DDatabase(node)
        state = OgnCheckGoal2DDatabase.per_node_internal_state(node)
        state.outputs = db.outputs

    @staticmethod
    def internal_state():
        return OgnCheckGoal2DInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state

        # if planner outputs targetChanged = True, new target data will be accessed and stored
        if db.inputs.targetChanged:
            state.target = db.inputs.target

        # get current pos/rot data
        pos = db.inputs.currentPosition
        x = pos[0]
        y = pos[1]
        _, _, rot = quatd4_to_euler(db.inputs.currentOrientation)

        # compare & output if diff between current pos/rot and target pos/rot is above threshold limits
        t = db.inputs.thresholds
        db.outputs.reachedGoal = [np.hypot(x - state.target[0], y - state.target[1]) <= t[0], rot <= t[1]]

        # begin next node (steering control)
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True


def quatd4_to_euler(orientation):
    # implementation for quat_to_euler_angles that normalizes outputs
    x, y, z, w = tuple(orientation)
    roll, pitch, yaw = quat_to_euler_angles(np.array([w, x, y, z]))

    return normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)
