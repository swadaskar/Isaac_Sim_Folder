# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper_Properties
import numpy as np
import carb


class SurfaceGripper(object):
    """[summary]

        Args:
            usd_path (Optional[str], optional): [description]. Defaults to None.
            translate (float, optional): [description]. Defaults to 0.
            direction (str, optional): [description]. Defaults to "x".
            grip_threshold (float, optional): [description]. Defaults to 1.
            force_limit (float, optional): [description]. Defaults to 1.0e6.
            torque_limit (float, optional): [description]. Defaults to 1.0e6.
            bend_angle (float, optional): [description]. Defaults to np.pi/24.
            kp (float, optional): [description]. Defaults to 1.0e5.
            kd (float, optional): [description]. Defaults to 1.0e4.
            disable_gravity (bool, optional): [description]. Defaults to True.
        """

    def __init__(
        self,
        usd_path: Optional[str] = None,
        translate: float = 0,
        direction: str = "x",
        grip_threshold: float = 0.01,
        force_limit: float = 1.0e6,
        torque_limit: float = 1.0e4,
        bend_angle: float = np.pi / 24,
        kp: float = 1.0e2,
        kd: float = 1.0e2,
        disable_gravity: bool = True,
    ) -> None:
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._translate = translate
        self._direction = direction
        self._grip_threshold = grip_threshold
        self._force_limit = force_limit
        self._torque_limit = torque_limit
        self._bend_angle = bend_angle
        self._kp = kp
        self._kd = kd
        self._disable_gravity = disable_gravity
        self._virtual_gripper = None
        self._usd_path = usd_path
        return

    def initialize(self, root_prim_path: str) -> None:
        """[summary]

        Args:
            root_prim_path (str): [description]
        """
        if self._usd_path is not None:
            add_reference_to_stage(usd_path=self._usd_path, prim_path=root_prim_path)
        virtual_gripper_props = Surface_Gripper_Properties()
        virtual_gripper_props.parentPath = root_prim_path
        virtual_gripper_props.d6JointPath = virtual_gripper_props.parentPath + "/d6FixedJoint"
        virtual_gripper_props.gripThreshold = self._grip_threshold
        virtual_gripper_props.forceLimit = self._force_limit
        virtual_gripper_props.torqueLimit = self._torque_limit
        virtual_gripper_props.bendAngle = self._bend_angle
        virtual_gripper_props.stiffness = self._kp
        virtual_gripper_props.damping = self._kd
        virtual_gripper_props.disableGravity = self._disable_gravity
        tr = _dynamic_control.Transform()
        if self._direction == "x":
            tr.p.x = self._translate
        elif self._direction == "y":
            tr.p.y = self._translate
        elif self._direction == "z":
            tr.p.z = self._translate
        else:
            carb.log_error("Direction specified for the surface gripper doesn't exist")
        virtual_gripper_props.offset = tr
        virtual_gripper = Surface_Gripper(self._dc_interface)
        virtual_gripper.initialize(virtual_gripper_props)
        self._virtual_gripper = virtual_gripper
        return

    def close(self) -> None:
        """[summary]
        """
        if not self.is_closed():
            self._virtual_gripper.close()
        if not self.is_closed():
            carb.log_warn("gripper didn't close successfully")
        return

    def open(self) -> None:
        """[summary]
        """
        result = self._virtual_gripper.open()
        if not result:
            carb.log_warn("gripper didn't close successfully")
        return

    def update(self) -> None:
        """[summary]
        """
        self._virtual_gripper.update()
        return

    def is_closed(self) -> bool:
        """[summary]

        Returns:
            bool: [description]
        """
        return self._virtual_gripper.is_closed()

    def set_translate(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]
        """
        self._translate = value
        return

    def set_direction(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]
        """
        self._direction = value
        return

    def set_force_limit(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]
        """
        self._force_limit = value
        return

    def set_torque_limit(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]
        """
        self._torque_limit = value
        return
