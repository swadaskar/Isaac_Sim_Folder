# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.controllers.articulation_controller import ArticulationController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
import numpy as np
import typing
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
from omni.isaac.manipulators.grippers.gripper import Gripper


class ScrewController(BaseController):
    """ 
        A state machine for screwing nuts on bolts 

        Each phase runs for 1 second, which is the internal time of the state machine

        Dt of each phase/ event step is defined
       
        - State 0: Lower end_effector down to encircle the nut
        - State 1: Close grip
        - State 2: Re-Center end-effector grip with that of the nut and bolt
        - State 3: Screw Clockwise
        - State 4: Open grip (initiates at this state and cycles until limit)
        - State 5: Screw counter-clockwise
        
        Args:
            name (str): Name id of the controller
            cspace_controller (BaseController): a cartesian space controller that returns an ArticulationAction type
            gripper (Gripper): a gripper controller for open/ close actions.
            events_dt (typing.Optional[typing.List[float]], optional): Dt of each phase/ event step. 10 phases dt has to be defined. Defaults to None.

        Raises:
            Exception: events dt need to be list or numpy array
            Exception: events dt need have length of 5 or less
        """

    def __init__(
        self,
        name: str,
        cspace_controller: BaseController,
        gripper: Gripper,
        events_dt: typing.Optional[typing.List[float]] = None,
    ) -> None:
        BaseController.__init__(self, name=name)
        self._event = 4
        self._t = 0
        self._events_dt = events_dt
        if self._events_dt is None:
            self._events_dt = [0.005, 0.1, 0.05, 0.005, 0.1, 0.01]
        else:
            if not isinstance(self._events_dt, np.ndarray) and not isinstance(self._events_dt, list):
                raise Exception("events dt need to be list or numpy array")
            elif isinstance(self._events_dt, np.ndarray):
                self._events_dt = self._events_dt.tolist()
            if len(self._events_dt) > 5:
                raise Exception("events dt need have length of 5 or less")
        self._cspace_controller = cspace_controller
        self._gripper = gripper
        self._pause = False
        self._start = True
        self._screw_position = np.array([0.0, 0.0, 0.0])
        self._screw_speed = 60.0 / 180.0 * np.pi
        self._screw_speed_back = 120.0 / 180.0 * np.pi
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
        franka_art_controller: ArticulationController,
        bolt_position: np.ndarray,
        current_joint_positions: np.ndarray,
        current_joint_velocities: np.ndarray,
    ) -> ArticulationAction:
        """Runs the controller one step.

        Args:
            franka_art_controller (ArticulationController): Robot's Articulation Controller.
            bolt_position (np.ndarray): bolt position to reference for screwing position.
            current_joint_positions (np.ndarray): Current joint positions of the robot.
            current_joint_velocities (np.ndarray): Current joint velocities of the robot.
            
        Returns:
            ArticulationAction: action to be executed by the ArticulationController
        """
        if self._pause or self._event >= len(self._events_dt):
            target_joints = [None] * current_joint_positions.shape[0]
            return ArticulationAction(joint_positions=target_joints)

        if self._event == 0 and self._start:
            self._screw_position = bolt_position
            self._start = False
            self._target_end_effector_orientation = self._gripper.get_world_pose()[1]

        if self._event == 0:
            franka_art_controller.switch_dof_control_mode(dof_index=6, mode="position")
            orientation_quat = self._gripper.get_world_pose()[1]
            self.orientation_euler = quat_to_euler_angles(orientation_quat)
            target_orientation_euler = np.array([self.orientation_euler[0], self.orientation_euler[1], -np.pi / 2])
            target_orientation_quat = euler_angles_to_quat(target_orientation_euler)
            target_joints = self._cspace_controller.forward(
                target_end_effector_position=self._screw_position,
                target_end_effector_orientation=target_orientation_quat,
            )

        if self._event == 1:
            self._lower = False
            franka_art_controller.switch_dof_control_mode(dof_index=6, mode="position")
            target_joints = self._gripper.forward(action="close")

        if self._event == 2:
            franka_art_controller.switch_dof_control_mode(dof_index=6, mode="position")
            orientation_quat = self._gripper.get_world_pose()[1]
            self.orientation_euler = quat_to_euler_angles(orientation_quat)
            target_orientation_euler = np.array([self.orientation_euler[0], self.orientation_euler[1], -np.pi / 2])
            target_orientation_quat = euler_angles_to_quat(target_orientation_euler)
            finger_pos = current_joint_positions[-2:]
            positive_x_offset = finger_pos[1] - finger_pos[0]
            target_joints = self._cspace_controller.forward(
                target_end_effector_position=self._screw_position - np.array([positive_x_offset, -0.002, 0.001]),
                target_end_effector_orientation=target_orientation_quat,
            )

        if self._event == 3:
            franka_art_controller.switch_dof_control_mode(dof_index=6, mode="velocity")
            target_joint_velocities = [None] * current_joint_velocities.shape[0]
            target_joint_velocities[6] = self._screw_speed
            if current_joint_positions[6] > 2.7:
                target_joint_velocities[6] = 0.0
            target_joints = ArticulationAction(joint_velocities=target_joint_velocities)

        if self._event == 4:
            franka_art_controller.switch_dof_control_mode(dof_index=6, mode="position")
            target_joints = self._gripper.forward(action="open")

        if self._event == 5:
            franka_art_controller.switch_dof_control_mode(dof_index=6, mode="velocity")
            target_joint_velocities = [None] * current_joint_velocities.shape[0]
            target_joint_velocities[6] = -self._screw_speed_back
            if current_joint_positions[6] < -0.4:
                target_joint_velocities[6] = 0.0
            target_joints = ArticulationAction(joint_velocities=target_joint_velocities)

        self._t += self._events_dt[self._event]
        if self._t >= 1.0:
            self._event = (self._event + 1) % 6
            self._t = 0
            if self._event == 5:
                if not self._start and (bolt_position[2] - self._screw_position[2] > 0.0198):
                    self.pause()
                    return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])
                if self._start:
                    self._screw_position[2] -= 0.001
                self._screw_position[2] -= 0.0018

        return target_joints

    def reset(self, events_dt: typing.Optional[typing.List[float]] = None) -> None:
        """Resets the state machine to start from the first phase/ event

        Args:
            events_dt (typing.Optional[typing.List[float]], optional):  Dt of each phase/ event step. Defaults to None.

        Raises:
            Exception: events dt need to be list or numpy array
            Exception: events dt need have length of 5 or less
        """
        BaseController.reset(self)
        self._cspace_controller.reset()
        self._event = 4
        self._t = 0
        self._pause = False
        self._start = True
        self._screw_position = np.array([0.0, 0.0, 0.0])
        self._screw_speed = 60.0 / 180.0 * np.pi
        self._screw_speed_back = 120.0 / 180.0 * np.pi
        # self._gripper = gripper
        if events_dt is not None:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, np.ndarray) and not isinstance(self._events_dt, list):
                raise Exception("events dt need to be list or numpy array")
            elif isinstance(self._events_dt, np.ndarray):
                self._events_dt = self._events_dt.tolist()
            if len(self._events_dt) > 5:
                raise Exception("events dt need have length of 5 or less")
        return

    def is_done(self) -> bool:
        """
        Returns:
            bool: True if the state machine reached the last phase. Otherwise False.
        """
        if self._event >= len(self._events_dt):
            return True
        else:
            return False

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
