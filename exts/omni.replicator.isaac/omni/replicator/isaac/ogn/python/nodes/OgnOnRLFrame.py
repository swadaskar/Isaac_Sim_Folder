# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.graph.core as og
from omni.replicator.isaac.scripts import context

import numpy as np


class OgnOnRLFrameInternalState:
    def __init__(self):
        self.frame_count = None


class OgnOnRLFrame:
    @staticmethod
    def internal_state():
        return OgnOnRLFrameInternalState()

    @staticmethod
    def compute(db) -> bool:
        if not context._context or not context._context.trigger:
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return True

        context._context.trigger = False
        state = db.internal_state
        reset_inds = context.get_reset_inds()

        if state.frame_count is None:
            state.frame_count = np.zeros(db.inputs.num_envs)

        if reset_inds is not None and len(reset_inds) > 0:
            state.frame_count[reset_inds] = 0
            db.outputs.resetInds = reset_inds
        else:
            db.outputs.resetInds = []

        db.outputs.frameNum = state.frame_count

        state.frame_count += 1

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
