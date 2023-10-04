# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Union
import numpy as np
import torch
from omni.isaac.core.articulations.articulation_view import ArticulationView


class RobotView(ArticulationView):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "robot".
            position (Optional[np.ndarray], optional): [description]. Defaults to None.
            translation (Optional[np.ndarray], optional): [description]. Defaults to None.
            orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            scale (Optional[np.ndarray], optional): [description]. Defaults to None.
            visible (bool, optional): [description]. Defaults to True.
            articulation_controller (Optional[ArticulationController], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "rigid_prim_view",
        positions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor]] = None,
    ) -> None:
        ArticulationView.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
        )
        self._sensors = list()
        return

    def post_reset(self) -> None:
        """[summary]
        """
        ArticulationView.post_reset(self)
        return
