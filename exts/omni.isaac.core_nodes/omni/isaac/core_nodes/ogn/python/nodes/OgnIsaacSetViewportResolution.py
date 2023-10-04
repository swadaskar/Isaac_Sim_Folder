# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
from omni.kit.viewport.utility import get_viewport_from_window_name, get_active_viewport

from omni.isaac.core.utils.carb import set_carb_setting
import carb


class OgnIsaacSetViewportResolution:
    """
    Isaac Sim Set Viewport Resolution
    """

    @staticmethod
    def compute(db) -> bool:
        viewport_name = db.inputs.viewport
        if viewport_name:
            viewport_api = get_viewport_from_window_name(viewport_name)
        else:
            viewport_api = get_active_viewport()

        if viewport_api:
            viewport_api.set_texture_resolution((db.inputs.width, db.inputs.height))
            set_carb_setting(carb.settings.get_settings(), "/app/hydra/aperture/conform", 3)
            set_carb_setting(carb.settings.get_settings(), "/app/hydra/aperture/conform", 4)

        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True
