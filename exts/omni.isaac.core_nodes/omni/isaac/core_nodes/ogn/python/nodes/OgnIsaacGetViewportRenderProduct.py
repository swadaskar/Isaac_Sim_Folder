# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
import carb

# from omni.kit.viewport.utility import create_viewport_window, get_active_viewport_window
# from omni.isaac.core.utils.viewports import get_window_from_id, get_id_from_index
from omni.replicator.core.scripts.utils.viewport_manager import HydraTextureWrapper
from omni.isaac.core_nodes.ogn.OgnIsaacGetViewportRenderProductDatabase import OgnIsaacGetViewportRenderProductDatabase
from omni.kit.viewport.utility import get_viewport_from_window_name


class OgnIsaacGetViewportRenderProductInternalState:
    def __init__(self):
        viewport = None


class OgnIsaacGetViewportRenderProduct:
    """
    Isaac Sim Create Hydra Texture
    """

    @staticmethod
    def internal_state():
        return OgnIsaacGetViewportRenderProductInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state
        viewport_api = get_viewport_from_window_name(db.inputs.viewport)
        if viewport_api:
            db.internal_state.viewport = viewport_api
        if db.internal_state.viewport == None:
            carb.log_warn("viewport name {db.inputs.viewport} not found")
            db.internal_state.initialized = False
            return False

        viewport = db.internal_state.viewport
        db.outputs.renderProductPath = viewport.get_render_product_path()
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release(node):
        try:
            state = OgnIsaacGetViewportRenderProductDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.viewport = None
