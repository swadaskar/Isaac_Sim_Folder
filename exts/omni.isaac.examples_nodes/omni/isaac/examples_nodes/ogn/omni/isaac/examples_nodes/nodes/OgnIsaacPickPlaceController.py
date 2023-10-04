# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.universal_robots.ur10 import UR10
from omni.isaac.franka.franka import Franka
from omni.isaac.dofbot.dofbot import DofBot
from omni.isaac.universal_robots.controllers import PickPlaceController as PickPlaceControllerUR
from omni.isaac.dofbot.controllers import PickPlaceController as PickPlaceControllerDofBot
from omni.isaac.franka.controllers import PickPlaceController as PickPlaceControllerFranka
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.examples_nodes.ogn.OgnIsaacPickPlaceControllerDatabase import OgnIsaacPickPlaceControllerDatabase


class OgnIsaacPickPlaceControllerInternalState(BaseResetNode):
    """
    nodes for moving an articulated robot with joint commands
    """

    def __init__(self):
        self.robot_prim_path = None
        self.robot_model = None
        self.robot_prim = None

        # Pick and Place Params
        self.robot = None
        self.gripper = None
        self.controller_handle = None
        self.events_dt = None
        self.picking_position = None
        self.placing_position = None
        self.end_effector_offset = None

        super().__init__(initialize=False)

    def initialize_controller(self):
        controller_name = "pick_place"
        if "franka" in self.robot_model.lower():
            self.robot = Franka(prim_path=self.robot_prim_path)
            self.controller_handle = PickPlaceControllerFranka(
                name=controller_name + "_" + self.robot_model.lower(),
                robot_articulation=self.robot,
                gripper=self.robot.gripper,
                events_dt=self.events_dt,
            )
        elif "ur" in self.robot_model.lower():
            self.robot = UR10(prim_path=self.robot_prim_path)
            self.controller_handle = PickPlaceControllerUR(
                name=controller_name + "_" + self.robot_model.lower(),
                robot_articulation=self.robot,
                gripper=self.robot.gripper,
                events_dt=self.events_dt,
            )
        elif "dofbot" in self.robot_model.lower():
            self.robot = DofBot(prim_path=self.robot_prim_path)
            self.controller_handle = PickPlaceControllerDofBot(
                name=controller_name + "_" + self.robot_model.lower(),
                robot_articulation=self.robot,
                gripper=self.robot.gripper,
                events_dt=self.events_dt,
            )

        self.gripper = self.robot.gripper
        self.robot.initialize()
        self.initialized = True
        return

    def apply_action(self):
        if self.initialized:
            actions = self.controller_handle.forward(
                picking_position=self.picking_position,
                placing_position=self.placing_position,
                current_joint_positions=self.robot.get_joint_positions(),
                end_effector_offset=self.end_effector_offset,
                # end_effector_orientation=euler_angles_to_quat(np.array([0, np.pi, 0])),
            )
            self.robot.get_articulation_controller().apply_action(actions)
        return

    def custom_reset(self):
        self.controller_handle = None
        pass


class OgnIsaacPickPlaceController:
    """
    nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def internal_state():
        return OgnIsaacPickPlaceControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state
        try:
            if not state.initialized:

                state.robot_model = db.inputs.robotModel
                if not state.robot_model:
                    db.log_warn("Please indicate your robot model before initializing: 'UR', 'Franka', or 'DofBot'")
                    return False

                if db.inputs.usePath:
                    state.robot_prim_path = db.inputs.robotPrimPath
                else:
                    if db.inputs.targetPrim.path is None:
                        return False
                    else:
                        state.robot_prim_path = db.inputs.targetPrim.path

                # grab the task params
                state.events_dt = db.inputs.eventsDT
                state.picking_position = db.inputs.pickingPosition
                state.placing_position = db.inputs.placingPosition
                state.end_effector_offset = db.inputs.endEffectorOffset

                # initialize the controller handle for the robot
                state.initialize_controller()

            # update the task params, can change every step
            state.events_dt = db.inputs.eventsDT
            state.picking_position = db.inputs.pickingPosition
            state.placing_position = db.inputs.placingPosition
            state.end_effector_offset = db.inputs.endEffectorOffset

            state.apply_action()

        except Exception as error:
            db.log_warn(str(error))
            return False

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnIsaacPickPlaceControllerDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
