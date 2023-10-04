# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.motion_generation import ArticulationKinematicsSolver, interface_config_loader, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation
from typing import Optional


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Dofbot robot.  This class loads a LulaKinematicsSovler object

    Args:
        robot_articulation (Articulation): An initialized Articulation object representing this Dofbot
        end_effector_frame_name (Optional[str]): The name of the Dofbot end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
    """

    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("DofBot")
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        if end_effector_frame_name is None:
            end_effector_frame_name = "link5"

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return
