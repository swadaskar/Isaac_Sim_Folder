# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
import numpy as np

from .trajectory import Trajectory
from omni.isaac.core.articulations import Articulation, ArticulationSubset
from omni.isaac.core.utils.types import ArticulationAction
from typing import List
import carb


class ArticulationTrajectory:
    """Wrapper class which takes in a Trajectory object and converts the output to discrete ArticulationActions that may be sent to the provided robot Articulation.

    Args:
        robot_articulation (Articulation): Initialized robot Articulation object representing the simulated USD robot
        trajectory (Trajectory): An instance of a class that implements the Trajectory interface.
        physics_dt (float): Duration of a physics step in Isaac Sim (typically 1/60 s).
    """

    def __init__(self, robot_articulation: Articulation, trajectory: Trajectory, physics_dt: float) -> None:
        self._articulation = robot_articulation
        self._trajectory = trajectory
        self._physics_dt = physics_dt

        self._active_joints_view = ArticulationSubset(robot_articulation, trajectory.get_active_joints())

    def get_action_at_time(self, time: float) -> ArticulationAction:
        """Get an ArticulationAction that will send the robot to the desired position/velocity at a given time in the provided Trajectory.

        Args:
            time (float): Time between the start and end times in the provided Trajectory.  If the time is out of bounds, an error will be thrown.

        Returns:
            ArticulationAction: ArticulationAction that may be passed directly to the robot Articulation to send it to the desired position/velocity at the given time.
        """
        if time < self._trajectory.start_time:
            carb.log_error(
                f"Provided time {time} is before the start time {self._trajectory.start_time} of the Trajectory"
            )

        if time > self._trajectory.end_time:
            carb.log_error(f"Provided time {time} is after the end time {self._trajectory.end_time} of the Trajectory")

        position_target, velocity_target = self._trajectory.get_joint_targets(time)

        position_action_np_array = self._active_joints_view.map_to_articulation_order(position_target)
        velocity_action_np_array = self._active_joints_view.map_to_articulation_order(velocity_target)

        return ArticulationAction(joint_positions=position_action_np_array, joint_velocities=velocity_action_np_array)

    def get_action_sequence(self, timestep: float = None) -> List[ArticulationAction]:
        """Get a sequence of ArticulationActions which sample the entire Trajectory according to the provided timestep.

        Args:
            timestep (float, optional): Timestep used for sampling the provided Trajectory.
                A vlue of 1/60, for example, returns ArticulationActions that represent the desired position/velocity of 
                the robot at 1/60 second intervals.  I.e. a one second trajectory with timestep=1/60 would result in 60 ArticulationActions.
                When not provided, the framerate of Isaac Sim is used. Defaults to None.

        Returns:
            List[ArticulationAction]: Sequence of ArticulationActions that may be passed to the robot Articulation to produce the desired trajectory.
        """
        if timestep is None:
            timestep = self._physics_dt
        actions = []
        for t in np.arange(self._trajectory.start_time, self._trajectory.end_time, timestep):
            actions.append(self.get_action_at_time(t))
        return actions

    def get_trajectory_duration(self) -> float:
        """Returns the duration of the provided Trajectory

        Returns:
            float: Duration of the provided trajectory
        """
        return self._trajectory.end_time - self._trajectory.start_time

    def get_active_joints_subset(self) -> ArticulationSubset:
        """Get view into active joints

        Returns:
            ArticulationSubset: Returns robot states for active joints in an order compatible with the TrajectoryGenerator
        """
        return self._active_joints_view

    def get_robot_articulation(self) -> Articulation:
        """Get the robot Articulation

        Returns:
            Articulation: Articulation object describing the robot.
        """
        return self._articulation

    def get_trajectory(self) -> Trajectory:
        return self._trajectory
