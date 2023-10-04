# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
from omni.kit.viewport.utility import create_viewport_window, get_active_viewport_window
from omni.isaac.core.utils.viewports import get_window_from_id, get_id_from_index


class OgnIsaacCreateViewportInternalState:
    def __init__(self):
        self.window = None


class OgnIsaacCreateViewport:
    """
    Isaac Sim Create Viewport
    """

    @staticmethod
    def internal_state():
        return OgnIsaacCreateViewportInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state
        if state.window is None:
            if len(db.inputs.name) > 0:
                state.window = create_viewport_window(db.inputs.name)
            else:
                if db.inputs.viewportId == 0:
                    state.window = get_active_viewport_window()
                else:
                    state.window = create_viewport_window(str(db.inputs.viewportId))
        db.outputs.viewport = state.window.title
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True
