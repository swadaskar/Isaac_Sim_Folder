# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.prims.xform_prim import XFormPrim
from typing import Optional, Sequence


class BaseSensor(XFormPrim):
    def __init__(
        self,
        prim_path: str,
        name: str = "base_sensor",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
    ) -> None:
        XFormPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
        )
        return

    def initialize(self, physics_sim_view=None) -> None:
        XFormPrim.initialize(self, physics_sim_view=physics_sim_view)
        return

    def post_reset(self) -> None:
        # XFormPrim.post_reset(self)
        return
