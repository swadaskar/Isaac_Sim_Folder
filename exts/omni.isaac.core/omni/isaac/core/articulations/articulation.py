# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Union, List, Sequence
import numpy as np
import omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.prims._impl.single_prim_wrapper import _SinglePrimWrapper
from omni.isaac.core.utils.types import JointsState, ArticulationAction
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.controllers.articulation_controller import ArticulationController
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
import carb


class Articulation(_SinglePrimWrapper):
    """ Provides high level functions to deal with an articulation prim and its attributes/ properties.
        
        Args:
            prim_path (str): prim path of the Prim to encapsulate or create.
            name (str, optional): shortname to be used as a key by Scene class. 
                                    Note: needs to be unique if the object is added to the Scene. 
                                    Defaults to "articulation".
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                            (with respect to its parent prim). shape is (3, ).
                                                            Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                            (depends if translation or position is specified).
                                                            quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                            Defaults to None, which means left unchanged.
            scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                    Defaults to None, which means left unchanged.
            visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
            articulation_controller (Optional[ArticulationController], optional): a custom ArticulationController which
                                                                                  inherits from it. Defaults to creating the
                                                                                  basic ArticulationController.
            enable_dof_force_sensors (bool, optional): enables the solver computed dof force sensors on articulation joints.
                                                       Defaults to False.
        Raises:
            Exception: [description]

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "articulation",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        articulation_controller: Optional[ArticulationController] = None,
        enable_dof_force_sensors: bool = False,
    ) -> None:
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._device = None
            self._backend_utils = np_utils
        if position is not None:
            position = self._backend_utils.convert(position, self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if translation is not None:
            translation = self._backend_utils.convert(translation, self._device)
            translation = self._backend_utils.expand_dims(translation, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if scale is not None:
            scale = self._backend_utils.convert(scale, self._device)
            scale = self._backend_utils.expand_dims(scale, 0)
        if visible is not None:
            visible = self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)
        self._articulation_view = ArticulationView(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            translations=translation,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            enable_dof_force_sensors=enable_dof_force_sensors,
        )
        self._articulation_controller = articulation_controller
        if self._articulation_controller is None:
            self._articulation_controller = ArticulationController()
        self._handles_initialized = False
        self._handle = None
        _SinglePrimWrapper.__init__(self, view=self._articulation_view)
        return

    @property
    def articulation_handle(self) -> int:
        """[summary]

        Returns:
            int: [description]
        """
        return self._handle

    @property
    def handles_initialized(self) -> bool:
        """[summary]

        Returns:
            bool: [description]
        """
        return self._handles_initialized and self._articulation_view.is_physics_handle_valid()

    @property
    def num_dof(self) -> int:
        """[summary]

        Returns:
            int: [description]
        """
        return self._articulation_view.num_dof

    @property
    def dof_properties(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        return self._dc_interface.get_articulation_dof_properties(self._handle)

    @property
    def dof_names(self) -> List[str]:
        """List of prim names for each DOF.

        Returns:
            list(string): prim names
        """
        return self._articulation_view.dof_names

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None):
        """Create a physics simulation view if not passed and creates an articulation view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        carb.log_info("initializing handles for {}".format(self.prim_path))
        self._handle = self._dc_interface.get_articulation(self.prim_path)
        self._root_handle = self._dc_interface.get_articulation_root_body(self._handle)
        self._articulation_controller.initialize(self._handle, self._articulation_view)
        self._articulation_view.initialize(physics_sim_view=physics_sim_view)
        self._handles_initialized = True
        return

    def get_dof_index(self, dof_name: str) -> int:
        """[summary]

        Args:
            dof_name (str): [description]

        Returns:
            int: [description]
        """
        return self._articulation_view.get_dof_index(dof_name=dof_name)

    def get_articulation_body_count(self) -> int:
        """[summary]

        Returns:
            int: [description]
        """
        return self._articulation_view.get_articulation_body_count()

    def disable_gravity(self) -> None:
        """Keep gravity from affecting the robot
        """
        for body_index in range(self._dc_interface.get_articulation_body_count(self._handle)):
            body = self._dc_interface.get_articulation_body(self._handle, body_index)
            self._dc_interface.set_rigid_body_disable_gravity(body, True)
        return

    def enable_gravity(self) -> None:
        """Gravity will affect the robot
        """
        for body_index in range(self._dc_interface.get_articulation_body_count(self._handle)):
            body = self._dc_interface.get_articulation_body(self._handle, body_index)
            self._dc_interface.set_rigid_body_disable_gravity(body, False)
        return

    def set_joint_positions(
        self, positions: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """[summary]

        Args:
            positions (np.ndarray): [description]
            indices (Optional[Union[list, np.ndarray]], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
        """
        positions = self._backend_utils.expand_dims(positions, 0)
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        self._articulation_view.set_joint_positions(positions=positions, joint_indices=joint_indices)
        return

    def get_joint_positions(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """_summary_

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): _description_. Defaults to None.

        Returns:
            np.ndarray: _description_
        """
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_joint_positions(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def set_joint_velocities(
        self, velocities: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """[summary]

        Args:
            velocities (np.ndarray): [description]
            indices (Optional[Union[list, np.ndarray]], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
        """
        velocities = self._backend_utils.expand_dims(velocities, 0)
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        self._articulation_view.set_joint_velocities(velocities=velocities, joint_indices=joint_indices)
        return

    def set_joint_efforts(self, efforts: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None) -> None:
        """[summary]

        Args:
            efforts (np.ndarray): [description]
            joint_indices (Optional[Union[list, np.ndarray]], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
        """
        efforts = self._backend_utils.expand_dims(efforts, 0)
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        self._articulation_view.set_joint_efforts(efforts=efforts, joint_indices=joint_indices)
        return

    def get_joint_velocities(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """_summary_

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): _description_. Defaults to None.

        Returns:
            np.ndarray: _description_
        """
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_joint_velocities(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def get_joint_efforts(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """ Deprecated function. Please use get_applied_joint_efforts  instead.

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): _description_. Defaults to None.

        Raises:
            Exception: _description_

        Returns:
            np.ndarray: _description_
        """
        carb.log_warn(
            "get_joint_efforts is deprecated. Please use get_applied_joint_efforts to get the applied joint efforts."
        )
        if self._handle is None:
            raise Exception("handles are not initialized yet")
        joint_efforts = self._dc_interface.get_articulation_dof_states(self._handle, _dynamic_control.STATE_EFFORT)
        joint_efforts = [joint_efforts[i][2] for i in range(len(joint_efforts))]
        if joint_indices is None:
            return np.array(joint_efforts)
        else:
            return np.array(joint_efforts[joint_indices])

    def get_applied_joint_efforts(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Gets the efforts applied to the joints

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): _description_. Defaults to None.

        Raises:
            Exception: _description_

        Returns:
            np.ndarray: _description_
        """
        if self._handle is None:
            raise Exception("handles are not initialized yet")
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_applied_joint_efforts(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def get_joints_default_state(self) -> JointsState:
        """ Accessor for the default joints state.

        Returns:
            JointsState: The defaults that the robot is reset to when post_reset() is called (often
            automatically called during world.reset()).
        """
        joints_state = self._articulation_view.get_joints_default_state()
        if joints_state is None:
            return None
        return JointsState(positions=joints_state.positions[0], velocities=joints_state.velocities[0], efforts=None)

    def set_joints_default_state(
        self,
        positions: Optional[np.ndarray] = None,
        velocities: Optional[np.ndarray] = None,
        efforts: Optional[np.ndarray] = None,
    ) -> None:
        """[summary]

        Args:
            positions (Optional[np.ndarray], optional): [description]. Defaults to None.
            velocities (Optional[np.ndarray], optional): [description]. Defaults to None.
            efforts (Optional[np.ndarray], optional): [description]. Defaults to None.
        """
        if positions is not None:
            positions = self._backend_utils.expand_dims(positions, 0)
        if velocities is not None:
            velocities = self._backend_utils.expand_dims(velocities, 0)
        if efforts is not None:
            efforts = self._backend_utils.expand_dims(efforts, 0)
        self._articulation_view.set_joints_default_state(positions=positions, velocities=velocities, efforts=efforts)
        return

    def get_joints_state(self) -> JointsState:
        """[summary]

        Returns:
            JointsState: [description]
        """
        joints_state = self._articulation_view.get_joints_state()
        if joints_state is None:
            return None
        return JointsState(positions=joints_state.positions[0], velocities=joints_state.velocities[0], efforts=None)

    def get_articulation_controller(self) -> ArticulationController:
        """
        Returns:
            ArticulationController: PD Controller of all degrees of freedom of an articulation, can apply position targets, velocity targets and efforts.
        """
        return self._articulation_controller

    def set_linear_velocity(self, velocity: np.ndarray):
        """Sets the linear velocity of the prim in stage.

        Args:
            velocity (np.ndarray):linear velocity to set the rigid prim to. Shape (3,).
        """
        if velocity is not None:
            velocity = self._backend_utils.expand_dims(velocity, 0)
        return self._articulation_view.set_linear_velocities(velocities=velocity)

    def get_linear_velocity(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        result = self._articulation_view.get_linear_velocities()
        if result is not None:
            result = result[0]
        return result

    def set_angular_velocity(self, velocity: np.ndarray) -> None:
        """[summary]

        Args:
            velocity (np.ndarray): [description]
        """
        if velocity is not None:
            velocity = self._backend_utils.expand_dims(velocity, 0)
        self._articulation_view.set_angular_velocities(velocities=velocity)

    def get_angular_velocity(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        result = self._articulation_view.get_angular_velocities()
        if result is not None:
            result = result[0]
        return result

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """[summary]

        Args:
            control_actions (ArticulationAction): actions to be applied for next physics step.
            indices (Optional[Union[list, np.ndarray]], optional): degree of freedom indices to apply actions to. 
                                                                   Defaults to all degrees of freedom.

        Raises:
            Exception: [description]
        """
        self._articulation_controller.apply_action(control_actions=control_actions)
        return

    def get_applied_action(self) -> ArticulationAction:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            ArticulationAction: [description]
        """
        return self._articulation_controller.get_applied_action()

    def set_solver_position_iteration_count(self, count: int) -> None:
        """[summary]

        Args:
            count (int): [description]
        """
        count = self._backend_utils.create_tensor_from_list([count], dtype="int32")
        self._articulation_view.set_solver_position_iteration_counts(count)
        return

    def get_solver_position_iteration_count(self) -> int:
        """[summary]

        Returns:
            int: [description]
        """
        return self._articulation_view.get_solver_position_iteration_counts()[0]

    def set_solver_velocity_iteration_count(self, count: int):
        """[summary]

        Args:
            count (int): [description]
        """
        count = self._backend_utils.create_tensor_from_list([count], dtype="int32")
        self._articulation_view.set_solver_velocity_iteration_counts(count)
        return

    def get_solver_velocity_iteration_count(self) -> int:
        """[summary]

        Returns:
            int: [description]
        """
        return self._articulation_view.get_solver_velocity_iteration_counts()[0]

    def set_stabilization_threshold(self, threshold: float) -> None:
        """[summary]

        Args:
            threshold (float): [description]
        """
        threshold = self._backend_utils.create_tensor_from_list([threshold], dtype="float32")
        self._articulation_view.set_stabilization_thresholds(threshold)
        return

    def get_stabilization_threshold(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        return self._articulation_view.get_stabilization_thresholds()[0]

    def set_enabled_self_collisions(self, flag: bool) -> None:
        """[summary]

        Args:
            flag (bool): [description]
        """
        flag = self._backend_utils.create_tensor_from_list([flag], dtype="bool")
        self._articulation_view.set_enabled_self_collisions(flag)
        return

    def get_enabled_self_collisions(self) -> bool:
        """[summary]

        Returns:
            bool: [description]
        """
        return self._articulation_view.get_enabled_self_collisions()[0]

    def set_sleep_threshold(self, threshold: float) -> None:
        """[summary]

        Args:
            threshold (float): [description]
        """
        threshold = self._backend_utils.create_tensor_from_list([threshold], dtype="float32")
        self._articulation_view.set_sleep_thresholds(threshold)
        return

    def get_sleep_threshold(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        return self._articulation_view.get_sleep_thresholds()[0]
