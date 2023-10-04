# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.controllers.articulation_controller import ArticulationController


class Robot(Articulation):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "robot".
            position (Optional[Sequence[float]], optional): [description]. Defaults to None.
            translation (Optional[Sequence[float]], optional): [description]. Defaults to None.
            orientation (Optional[Sequence[float]], optional): [description]. Defaults to None.
            scale (Optional[Sequence[float]], optional): [description]. Defaults to None.
            visible (bool, optional): [description]. Defaults to True.
            articulation_controller (Optional[ArticulationController], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "robot",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: bool = True,
        articulation_controller: Optional[ArticulationController] = None,
    ) -> None:
        Articulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=articulation_controller,
        )
        self._sensors = list()
        return

    def post_reset(self) -> None:
        """[summary]
        """
        Articulation.post_reset(self)
        return
