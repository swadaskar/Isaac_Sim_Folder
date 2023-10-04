# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence
from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
from omni.isaac.core.utils.types import DynamicState
from omni.isaac.core.prims._impl.single_prim_wrapper import _SinglePrimWrapper
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
import numpy as np


class RigidPrim(_SinglePrimWrapper):
    """ Provides high level functions to deal with a rigid body prim and its attributes/ properties.
        If there is an prim present at the path, it will use it. Otherwise, a new XForm prim at
        the specified prim path will be created.
        Notes: if the prim does not already have a rigid body api applied to it before init, it will apply it.

        Args:
            prim_path (str): prim path of the Prim to encapsulate or create.
            name (str, optional): shortname to be used as a key by Scene class. 
                                  Note: needs to be unique if the object is added to the Scene. 
                                  Defaults to "rigid_prim".
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
            mass (Optional[float], optional): mass in kg. Defaults to None.
            linear_velocity (Optional[np.ndarray], optional): linear velocity in the world frame. Defaults to None.
            angular_velocity (Optional[np.ndarray], optional): angular velocity in the world frame. Defaults to None.

        """

    def __init__(
        self,
        prim_path: str,
        name: str = "rigid_prim",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        mass: Optional[float] = None,
        density: Optional[float] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ) -> None:
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
        if mass is not None:
            mass = self._backend_utils.create_tensor_from_list([mass], dtype="float32", device=self._device)
        if density is not None:
            density = self._backend_utils.create_tensor_from_list([density], dtype="float32", device=self._device)
        if linear_velocity is not None:
            linear_velocity = self._backend_utils.expand_dims(linear_velocity, 0)
        if angular_velocity is not None:
            angular_velocity = self._backend_utils.expand_dims(angular_velocity, 0)
        self._rigid_prim_view = RigidPrimView(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            translations=translation,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            masses=mass,
            linear_velocities=linear_velocity,
            angular_velocities=angular_velocity,
        )
        _SinglePrimWrapper.__init__(self, view=self._rigid_prim_view)
        return

    def set_linear_velocity(self, velocity: np.ndarray):
        """Sets the linear velocity of the prim in stage.
        Args:
            velocity (np.ndarray): linear velocity to set the rigid prim to. Shape (3,).
        """
        velocity = self._backend_utils.expand_dims(velocity, 0)
        self._rigid_prim_view.set_linear_velocities(velocities=velocity)
        return

    def get_linear_velocity(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: current linear velocity of the the rigid prim. Shape (3,).
        """
        velocities = self._rigid_prim_view.get_linear_velocities()
        return velocities[0]

    def set_angular_velocity(self, velocity: np.ndarray) -> None:
        """Sets the angular velocity of the prim in stage.
        Args:
            velocity (np.ndarray): angular velocity to set the rigid prim to. Shape (3,).
        """
        velocity = self._backend_utils.expand_dims(velocity, 0)
        self._rigid_prim_view.set_angular_velocities(velocities=velocity)
        return

    def get_angular_velocity(self):
        """
        Returns:
            np.ndarray: current angular velocity of the the rigid prim. Shape (3,).
        """
        velocities = self._rigid_prim_view.get_angular_velocities()
        return velocities[0]

    def set_mass(self, mass: float) -> None:
        """
        Args:
            mass (float): mass of the rigid body in kg.
        """
        masses = self._backend_utils.create_tensor_from_list([mass], dtype="float32")
        self._rigid_prim_view.set_masses(masses=masses)
        return

    def get_mass(self) -> float:
        """
        Returns:
            float: mass of the rigid body in kg.
        """
        masses = self._rigid_prim_view.get_masses()
        return masses[0]

    def set_density(self, density: float) -> None:
        """
        Args:
            mass (float): density of the rigid body.
        """
        densities = self._backend_utils.create_tensor_from_list([density], dtype="float32")
        self._rigid_prim_view.set_densities(densities)
        return

    def get_density(self) -> float:
        """
        Returns:
            float: density of the rigid body.
        """
        return self._rigid_prim_view.get_densities()[0]

    def set_sleep_threshold(self, threshold: float) -> None:
        """
        Args:
            threshold (float): Mass-normalized kinetic energy threshold below which 
                                an actor may go to sleep. Range: [0, inf)
                                Defaults: 0.00005 * tolerancesSpeed* tolerancesSpeed
                                Units: distance^2 / second^2.
        """
        thresholds = self._backend_utils.create_tensor_from_list([threshold], dtype="float32")
        self._rigid_prim_view.set_sleep_thresholds(thresholds)
        return

    def get_sleep_threshold(self) -> float:
        """
        Returns:
            float: Mass-normalized kinetic energy threshold below which 
                    an actor may go to sleep. Range: [0, inf)
                    Defaults: 0.00005 * tolerancesSpeed* tolerancesSpeed
                    Units: distance^2 / second^2.
        """
        return self._rigid_prim_view.get_sleep_thresholds()[0]

    def enable_rigid_body_physics(self) -> None:
        """ enable rigid body physics (enabled by default):
            Object will be moved by external forces such as gravity and collisions
        """
        self._rigid_prim_view.enable_rigid_body_physics()
        return

    def disable_rigid_body_physics(self) -> None:
        """ disable rigid body physics (enabled by default):
            Object will not be moved by external forces such as gravity and collisions
        """
        self._rigid_prim_view.disable_rigid_body_physics()
        return

    def set_default_state(
        self,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ) -> None:
        """ Sets the default state of the prim, that will be used after each reset. 
            
            Args:
                position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                    Defaults to None, which means left unchanged.
                orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim. 
                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                        Defaults to None, which means left unchanged.
                linear_velocity (np.ndarray): linear velocity to set the rigid prim to. Shape (3,).
                angular_velocity (np.ndarray): angular velocity to set the rigid prim to. Shape (3,).
        """
        if position is not None:
            position = self._backend_utils.convert(position, device=self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if linear_velocity is not None:
            linear_velocity = self._backend_utils.convert(linear_velocity, device=self._device)
            linear_velocity = self._backend_utils.expand_dims(linear_velocity, 0)
        if angular_velocity is not None:
            angular_velocity = self._backend_utils.convert(angular_velocity, device=self._device)
            angular_velocity = self._backend_utils.expand_dims(angular_velocity, 0)
        self._rigid_prim_view.set_default_state(
            positions=position,
            orientations=orientation,
            linear_velocities=linear_velocity,
            angular_velocities=angular_velocity,
        )
        return

    def get_default_state(self) -> DynamicState:
        """
        Returns:
            DynamicState: returns the default state of the prim (position, orientation, linear_velocity and 
                          angular_velocity) that is used after each reset.
        """
        view_default_state = self._rigid_prim_view.get_default_state()
        default_state = self._dynamics_view_state_conversion(view_default_state)
        return default_state

    def get_current_dynamic_state(self) -> DynamicState:
        """ 
        Returns:
            DynamicState: the dynamic state of the rigid body including position, orientation, linear_velocity and angular_velocity.
        """
        view_default_state = self._rigid_prim_view.get_current_dynamic_state()
        return self._dynamics_view_state_conversion(view_default_state)

    def _dynamics_view_state_conversion(self, view_state):
        # TODO: a temp function
        position = None
        orientation = None
        linear_velocity = None
        angular_velocity = None
        if view_state.positions is not None:
            position = view_state.positions[0]
        if view_state.orientations is not None:
            orientation = view_state.orientations[0]
        if view_state.linear_velocities is not None:
            linear_velocity = view_state.linear_velocities[0]
        if view_state.angular_velocities is not None:
            angular_velocity = view_state.angular_velocities[0]
        return DynamicState(
            position=position,
            orientation=orientation,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )
