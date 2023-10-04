# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.core_nodes.ogn.OgnIsaacArticulationControllerDatabase import OgnIsaacArticulationControllerDatabase


class OgnIsaacArticulationControllerInternalState(BaseResetNode):
    """
    nodes for moving an articulated robot with joint commands
    """

    def __init__(self):
        self.robot_prim = None
        self.controller_handle = None
        self.joint_names = None
        self.joint_indices = None
        self.joint_picked = False
        super().__init__(initialize=False)

    def initialize_controller(self):
        self.controller_handle = Articulation(self.robot_prim)
        self.controller_handle.initialize()
        self.num_dof = self.controller_handle.num_dof
        self.initialized = True

    def joint_indicator(self):
        if self.joint_names:
            self.joint_indices = []
            for name in self.joint_names:
                self.joint_indices.append(self.controller_handle.get_dof_index(name))
        elif np.size(self.joint_indices) > 0:
            self.joint_indices = self.joint_indices
        else:
            # when indices is none (not []), it defaults too all DOFs
            self.joint_indices = None
        self.joint_picked = True

    def apply_action(self, joint_positions, joint_velocities, joint_efforts):
        if self.initialized:
            joint_actions = ArticulationAction()
            joint_actions.joint_indices = self.joint_indices
            if np.size(joint_positions) > 0:
                joint_actions.joint_positions = joint_positions
            elif np.size(joint_velocities) > 0:
                joint_actions.joint_velocities = joint_velocities
            elif np.size(joint_efforts) > 0:
                joint_actions.joint_efforts = joint_efforts
            self.controller_handle.apply_action(control_actions=joint_actions)

    def custom_reset(self):
        self.controller_handle = None
        pass


class OgnIsaacArticulationController:
    """
    nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def internal_state():
        return OgnIsaacArticulationControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state
        try:
            if not state.initialized:
                if db.inputs.usePath:
                    state.robot_prim = db.inputs.robotPath
                else:
                    if db.inputs.targetPrim.path is None:
                        return False
                    else:
                        state.robot_prim = db.inputs.targetPrim.path

                # initialize the controller handle for the robot
                state.initialize_controller()

            # pick the joints that are being commanded, this can be different at every step
            joint_names = db.inputs.jointNames
            if np.array([joint_names != state.joint_names]).flatten().any():
                state.joint_names = joint_names
                state.joint_picked = False

            joint_indices = db.inputs.jointIndices
            if np.array([joint_indices != state.joint_indices]).flatten().any():
                state.joint_indices = np.array(joint_indices)
                state.joint_picked = False

            if not state.joint_picked:
                state.joint_indicator()

            joint_positions = db.inputs.positionCommand
            joint_velocities = db.inputs.velocityCommand
            joint_efforts = db.inputs.effortCommand
            state.apply_action(joint_positions, joint_velocities, joint_efforts)

        except Exception as error:
            db.log_warn(str(error))
            return False

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnIsaacArticulationControllerDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
