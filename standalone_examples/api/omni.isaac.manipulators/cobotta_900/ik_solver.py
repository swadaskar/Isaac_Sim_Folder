# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation
from typing import Optional
from omni.isaac.core.utils.extensions import get_extension_path_from_name
import os


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        urdf_extension_path = get_extension_path_from_name("omni.isaac.urdf")
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=os.path.join(os.path.dirname(__file__), "../rmpflow/robot_descriptor.yaml"),
            urdf_path=os.path.join(urdf_extension_path, "data/urdf/robots/cobotta_pro_900/cobotta_pro_900.urdf"),
        )
        if end_effector_frame_name is None:
            end_effector_frame_name = "onrobot_rg6_base_link"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return
