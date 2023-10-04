# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.timeline
import omni.graph.core as og


class OgnIsaacSimulationGateInternalState:
    def __init__(self):
        self.frame = 0


class OgnIsaacSimulationGate:
    """
    Isaac Sim Simulation Gate
    """

    @staticmethod
    def internal_state():
        return OgnIsaacSimulationGateInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state
        timeline = omni.timeline.acquire_timeline_interface()
        # If the timeline is stopped or step is set to zero, skip execution
        if timeline.is_playing() and db.inputs.step > 0:
            state.frame = state.frame + 1
            if state.frame >= db.inputs.step:
                state.frame = 0
                db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        else:
            state.frame = 0
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
        return True
