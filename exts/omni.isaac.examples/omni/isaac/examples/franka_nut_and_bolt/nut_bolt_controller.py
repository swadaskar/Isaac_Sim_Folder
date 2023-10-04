# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
import numpy as np
import typing
from omni.isaac.manipulators.controllers.pick_place_controller import PickPlaceController
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
from .screw_controller import ScrewController
from omni.isaac.franka.franka import Franka
from .nut_vibra_table_controller import VibraFSM


class NutBoltController(BaseController):
    """ 
        A state machine to tie nuts onto bolts with a vibrating table feeding the nuts 

        - State 0: Pick and Place from pickup location on vibration table to different bolts
        - State 1: Screw nut onto bolt

        Args:
            name (str): Name id of the controller
            franka (Franka): Franka Robot

        """

    def __init__(self, name: str, franka: Franka) -> None:
        BaseController.__init__(self, name=name)
        self._event = 0
        self._franka = franka
        self._gripper = self._franka.gripper
        self._end_effector_initial_height = self._franka.get_world_pose()[0][2] + (0.4 / get_stage_units())
        self._pause = False
        self._cspace_controller = RMPFlowController(name="pickplace_cspace_controller", robot_articulation=self._franka)
        pick_place_events_dt = [0.008, 0.005, 1, 0.1, 0.05, 0.01, 0.0025]
        self._pick_place_controller = PickPlaceController(
            name="pickplace_controller",
            cspace_controller=self._cspace_controller,
            gripper=self._gripper,
            end_effector_initial_height=self._end_effector_initial_height,
            events_dt=pick_place_events_dt,
        )
        self._screw_controller = ScrewController(
            name=f"screw_controller", cspace_controller=self._cspace_controller, gripper=self._gripper
        )
        self._vibraSM = VibraFSM()
        self._i = self._vibraSM._i
        self._vibraSM.stop_feed_after_delay(delay_sec=5.0)
        return

    def is_paused(self) -> bool:
        """

        Returns:
            bool: True if the state machine is paused. Otherwise False.
        """
        return self._pause

    def get_current_event(self) -> int:
        """

        Returns:
            int: Current event/ phase of the state machine
        """
        return self._event

    def forward(
        self,
        initial_picking_position: np.ndarray,
        bolt_top: np.ndarray,
        gripper_to_nut_offset: np.ndarray,
        x_offset: np.ndarray,
    ) -> np.ndarray:
        """Runs the controller one step.

        Args:
            initial_picking_position (np.ndarray): initial nut position at table feeder
            bolt_top (np.ndarray):  bolt target position
            
        # """
        _vibra_table_transforms = np.array([0.0, 0.0, 0.0])
        if self.is_paused():
            return _vibra_table_transforms
        offsetPos = self._vibraSM.update()
        _vibra_table_transforms = np.array(offsetPos, dtype=float)
        if self._vibraSM._state == "stop" and self._event == 0:
            initial_effector_orientation = quat_to_euler_angles(self._gripper.get_world_pose()[1])
            initial_effector_orientation[2] = np.pi / 2
            initial_end_effector_orientation = euler_angles_to_quat(initial_effector_orientation)
            actions = self._pick_place_controller.forward(
                picking_position=initial_picking_position + gripper_to_nut_offset,
                placing_position=bolt_top - np.array([x_offset, 0.0, 0.0]),
                current_joint_positions=self._franka.get_joint_positions(),
                end_effector_orientation=initial_end_effector_orientation,
            )
            self._franka.apply_action(actions)
            if self._pick_place_controller.is_done():
                self._vibraSM._set_delayed_state_change(delay_sec=1.0, nextState="backward")
                self._event = 1

        if self._event == 1:
            actions2 = self._screw_controller.forward(
                franka_art_controller=self._franka.get_articulation_controller(),
                bolt_position=bolt_top,
                current_joint_positions=self._franka.get_joint_positions(),
                current_joint_velocities=self._franka.get_joint_velocities(),
            )
            self._franka.apply_action(actions2)
            if self._screw_controller.is_paused():
                self.pause()
                self._i += 1
        return _vibra_table_transforms

    def reset(self, franka: Franka) -> None:
        """Resets the state machine to start from the first phase/ event

        Args:
            franka (Franka): Franka Robot
            
        """
        BaseController.reset(self)
        self._event = 0
        self._pause = False
        self._franka = franka
        self._gripper = self._franka.gripper
        self._end_effector_initial_height = self._franka.get_world_pose()[0][2] + (0.4 / get_stage_units())
        self._pick_place_controller.reset(end_effector_initial_height=self._end_effector_initial_height)
        self._screw_controller.reset()
        return

    def pause(self) -> None:
        """Pauses the state machine's time and phase.
        """
        self._pause = True
        return

    def resume(self) -> None:
        """Resumes the state machine's time and phase.
        """
        self._pause = False
        return
