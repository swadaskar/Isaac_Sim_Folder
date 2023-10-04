# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
import carb
from typing import Tuple, List, Union

from ..trajectory import Trajectory
from .kinematics import LulaKinematicsSolver
from .utils import get_pose3

import lula


class LulaTrajectory(Trajectory):
    """Instance of Trajectory interface class for handling lula.Trajectory objects

    Args:
        trajectory (lula.Trajectory): C-space trajectory defined continuously
    """

    def __init__(self, trajectory, active_joints):
        self.trajectory = trajectory
        self.active_joints = active_joints

    @property
    def start_time(self) -> float:
        __doc__ = Trajectory.start_time.__doc__
        return self.trajectory.domain().lower

    @property
    def end_time(self) -> float:
        __doc__ = Trajectory.end_time.__doc__
        return self.trajectory.domain().upper

    def get_active_joints(self) -> List[str]:
        __doc__ = Trajectory.get_active_joints.__doc__
        return self.active_joints

    def get_joint_targets(self, time) -> Tuple[np.array, np.array]:
        __doc__ = Trajectory.get_joint_targets.__doc__
        if time > self.end_time or time < self.start_time:
            carb.log_error("Could not compute joint targets because the provided time is out of bounds")
        return self.trajectory.eval(time, 0), self.trajectory.eval(time, 1)


class LulaCSpaceTrajectoryGenerator:
    """LulaCSpaceTrajectoryGenerator is a class for generating time-optimal trajectories that connect a series of 
    provided c-space waypoints.  

    Args:
        robot_description_path (str): path to a robot description yaml file
        urdf_path (str): path to robot urdf
    """

    def __init__(self, robot_description_path: str, urdf_path: str) -> None:

        self._robot_description = lula.load_robot(robot_description_path, urdf_path)
        self._lula_kinematics = self._robot_description.kinematics()
        self._kinematics_solver = LulaKinematicsSolver(robot_description_path, urdf_path, self._robot_description)

        self._c_space_trajectory_generator = None
        self._task_space_trajectory_generator = None

        self._c_space_trajectory_generator = lula.create_c_space_trajectory_generator(self._lula_kinematics)

    def compute_c_space_trajectory(self, waypoint_positions: np.array) -> LulaTrajectory:
        """Produce a trajectory from a set of provided c_space waypoint positions.  The resulting trajectory
        will use spline-based interpolation to connect the waypoints with an initial and final velocity of 0.  The trajectory is time-optimal:
        i.e. either the velocity, acceleration, or jerk limits are saturated at any given time to produce as trajectory with as short a duration as possible.

        Args:
            waypoint_positions (np.array): Set of c-space coordinates cooresponding to the output of get_active_joints().
                The expected shape is (N x k) where N is the number of waypoints and k is the number of active joints.

        Returns:
            LulaTrajectory: Instance of the Trajectory class which specifies continuous joint_targets for the active joints over a span of time.
        """

        if waypoint_positions.shape[0] < 2:
            carb.log_error("LulaTrajectoryGenerator must be passed at least two waypoints")

        if waypoint_positions.shape[1] != self._lula_kinematics.num_c_space_coords():
            carb.log_error(
                f"LulaTrajectoryGenerator was passed a set of waypoints with invalid shape: {waypoint_positions.shape}."
                + f"  Expecting shape ({waypoint_positions.shape[0]}, {self._lula_kinematics.num_c_space_coords()})."
                + "  Make sure that the provided waypoint_positions corresponds to the output of get_active_joints()."
            )

        trajectory = self._c_space_trajectory_generator.generate_trajectory(waypoint_positions.astype(np.float64))

        if trajectory is None:
            carb.log_warn(
                "LulaTrajectoryGenerator could not generate a trajectory connecting the given waypoints.  Returning None"
            )
            return

        return LulaTrajectory(trajectory, self.get_active_joints())

    def get_active_joints(self) -> List[str]:
        """Return the list of joints by name that are considered to be controllable by the TrajectoryGenerator.
        All inputs and outputs of the LulaTrajectoryGenerator correspond to the joints specified by get_active_joints().

        Returns:
            List[str]: List of joints that are used to generate the desired trajectory.
        """
        return self._kinematics_solver.get_joint_names()

    def set_c_space_position_limits(self, lower_position_limits: np.array, upper_position_limits: np.array) -> None:
        """Set the lower and upper position limits of the active joints to be used when generating a trajectory.

        Args:
            lower_position_limits (np.array): Lower position limits of active joints.
            upper_position_limits (np.array): Upper position limits of active joints.
        """

        if lower_position_limits.shape[0] != self._lula_kinematics.num_c_space_coords():
            carb.log_error(
                f"Provided lower position limits have an incorrect shape: {lower_position_limits.shape}\n"
                + f"Expected shape: ({self._lula_kinematics.num_c_space_coords()},)"
                + "  Make sure that the provided position limits corresponds to the output of get_active_joints()."
            )
        if upper_position_limits.shape[0] != self._lula_kinematics.num_c_space_coords():
            carb.log_error(
                f"Provided upper position limits have an incorrect shape: {upper_position_limits.shape}\n"
                + f"Expected shape: ({self._lula_kinematics.num_c_space_coords()},)"
                + "  Make sure that the provided position limits corresponds to the output of get_active_joints()."
            )

        c_space_position_lower_limits = lower_position_limits.astype(np.float64)
        c_space_position_upper_limits = upper_position_limits.astype(np.float64)
        self._c_space_trajectory_generator.set_position_limits(
            c_space_position_lower_limits, c_space_position_upper_limits
        )

    def set_c_space_velocity_limits(self, velocity_limits: np.array) -> None:
        """Set the velocity limits of the active joints to be used when generating a trajectory.

        Args:
            velocity_limits (np.array): Velocity limits of active joints.
        """
        if velocity_limits.shape[0] != self._lula_kinematics.num_c_space_coords():
            carb.log_error(
                f"Provided velocity limits have an incorrect shape: {velocity_limits.shape}\n"
                + f"Expected shape: ({self._lula_kinematics.num_c_space_coords()},)"
                + "  Make sure that the provided velocity limits corresponds to the output of get_active_joints()."
            )

        c_space_velocity_limits = velocity_limits.astype(np.float64)
        self._c_space_trajectory_generator.set_velocity_limits(c_space_velocity_limits)

    def set_c_space_acceleration_limits(self, acceleration_limits: np.array) -> None:
        """Set the acceleration limits of the active joints to be used when generating a trajectory.

        Args:
            acceleration_limits (np.array): Acceleration limits of active joints.
        """

        if acceleration_limits.shape[0] != self._lula_kinematics.num_c_space_coords():
            carb.log_error(
                f"Provided acceleration limits have an incorrect shape: {acceleration_limits.shape}\n"
                + f"Expected shape: ({self._lula_kinematics.num_c_space_coords()},)"
                + "  Make sure that the provided acceleration limits corresponds to the output of get_active_joints()."
            )
        c_space_acceleration_limits = acceleration_limits.astype(np.float64)
        self._c_space_trajectory_generator.set_acceleration_limits(c_space_acceleration_limits)

    def set_c_space_jerk_limits(self, jerk_limits: np.array) -> None:
        """Set the jerk limits of the active joints to be used when generating a trajectory.

        Args:
            jerk_limits (np.array): Jerk limits of active joints.
        """

        if jerk_limits.shape[0] != self._lula_kinematics.num_c_space_coords():
            carb.log_error(
                f"Provided jerk limits have an incorrect shape: {jerk_limits.shape}\n"
                + f"Expected shape: ({self._lula_kinematics.num_c_space_coords()},)"
                + "  Make sure that the provided jerk limits corresponds to the output of get_active_joints()."
            )

        c_space_jerk_limits = jerk_limits.astype(np.float64)
        self._c_space_trajectory_generator.set_jerk_limits(c_space_jerk_limits)

    def set_solver_param(self, param_name: str, param_val: Union[int, float, str]):
        """Set solver parameters for the cspace trajectory generator.  A complete list of
        parameters is provided in this docstring.

        'max_segment_iterations': (int)
            In general, a trajectory is locally time-optimal if at least one derivative for one of the
            c-space coordinates is fully saturated, with no derivative limits for any of the c-space
            coordinates exceeded.

            This time-optimality can be enforced for each `CubicSpline` segment or for each
            `PiecewiseCubicSpline` as a whole. The former will, in general, generate trajectories with
            smaller spans, but will require more expensive iterations (and thus more time) to converge.
            The latter will, in general, require less iterations (and thus less time) to converge, but
            the generated trajectories will tend to have longer spans.

            When attempting to find a time-optimal trajectory, the (more expensive) per-segment method
            will first be attempted for `max_per_segment_iterations`. Then, if not yet converged, the
            method acting on the entire spline will be attempted for `max_aggregate_iterations`.

            To maximize speed, `max_segment_iterations` should be relatively low (or even zero to remove
            this search completely). To maximize time-optimality of the generated trajectory,
            `max_segment_iterations` should be relatively high.

            The sum of `max_segment_iterations` and `max_aggregate_iterations` must be at least 1

        'max_aggragate_iterations': (int)
            See max_segment_iterations

        'convergence_dt': (float)
            The search for optimal time values will terminate if the maximum change to any time value
            during a given iteration is less than the `convergence_dt`.

            `convergence_dt` must be positive.

        'max_dilation_iterations': (int)
            After the segment-wise and/or aggregate time-optimal search has converged or reached maximum
            iterations, the resulting set of splines will be tested to see if any derivative limits are
            exceeded.

            If any derivative limits are exceeded, the splines will be iteratively scaled in time to
            reduce the maximum achieved derivative. This process will repeat until no derivative limits
            are exceeded (success) or `max_dilation_iterations_` are reached (failure).
            For a well-tuned set of solver parameters, very few dilation steps should be required
            (often none will be required or a single iteration is sufficient to bring a slightly
            over-saturated trajectory within the derivative limits).

        'dilation_dt': (float)
            For the iterative dilation step described in `setMaxDilationIterations()` documentation, the
            `dilation_dt` is the "epsilon" value added to the span of the trajectory that exceeds
            derivative limits.

            `dilation_dt` must be positive.
        
        'min_time_span': (float)
            Specify the minimum allowable time span between adjacent waypoints/endpoints.
            `min_time_span` must be positive.

            This is most likely to affect the time span between the endpoints and "free-position" points
            that are used to enable acceleration bound constraints. If no jerk limit is provided, these free-position points may
            tend to become arbitrarily close in position and time to the endpoints. This `min_time_span`
            prevents this time span from approaching zero.

            In general, a jerk limit is recommended for preventing abrupt changes in acceleration rather
            than relying on the `min_time_span` for this purpose.

        'time_split_method': (string)
            Often waypoints for a trajectory may specify positions without providing time values for when
            these waypoint position should be attained. In this case, we can use the distance between
            waypoints to assign time values for each waypoint.

            Assuming a unitary time domain s.t. t_0 = 0 and t_N = 1, we can assign the intermediate time
            values according to:

              t_k = t_(k-1) + (d_k / d),

            where d = sum(d_k) for k = [0, N-1] and N is the number of points.

            Many options exist for the computing the distance metric d_k, with common options described
            below (and implemented in `ComputeTimeValues()`.
            See Eqn 4.37 in "Trajectory Planning for Automatic Machines and Robots" (2008) by
            Biagiotti & Melchiorri for more detailed motivations.
            Valid distribution choices are given below:

            'uniform':
                For a "uniform distribution" w.r.t time, the positions are ignored and d_k can simply be
                computed as:

                  d_k = 1 / (N - 1)

                resulting in uniform time intervals between all points.

            'chord_length':
                For a "chord length distribution", the time intervals between waypoints are proportional to
                the Euclidean distance between waypoints:

                  d_k = \|q_(k+1) - q_k\|

                where q represents the position of the waypoint.

            'centripetal':
                For a "centripetal distribution", the time intervals between waypoints are proportional to the
                square root of the Euclidean distance between waypoints:

                  d_k = \|q_(k+1) - q_k\|^(1/2)

                where q represents the position of the waypoint.

        Args:
            param_name (str): Parameter name from the above list of parameters
            param_val (Union[int, float, str]): Value to which the given parameter will be set
        """
        self._c_space_trajectory_generator.set_solver_param(param_name, param_val)


class LulaTaskSpaceTrajectoryGenerator:

    get_active_joints = LulaCSpaceTrajectoryGenerator.get_active_joints

    set_c_space_position_limits = LulaCSpaceTrajectoryGenerator.set_c_space_position_limits
    set_c_space_velocity_limits = LulaCSpaceTrajectoryGenerator.set_c_space_velocity_limits
    set_c_space_acceleration_limits = LulaCSpaceTrajectoryGenerator.set_c_space_acceleration_limits
    set_c_space_jerk_limits = LulaCSpaceTrajectoryGenerator.set_c_space_jerk_limits

    set_c_space_trajectory_generator_solver_param = LulaCSpaceTrajectoryGenerator.set_solver_param

    def __init__(self, robot_description_path: str, urdf_path: str) -> None:
        self._robot_description = lula.load_robot(robot_description_path, urdf_path)
        self._lula_kinematics = self._robot_description.kinematics()
        self._kinematics_solver = LulaKinematicsSolver(robot_description_path, urdf_path, self._robot_description)

        self._c_space_trajectory_generator = None
        self._task_space_trajectory_generator = None

        self._c_space_trajectory_generator = lula.create_c_space_trajectory_generator(self._lula_kinematics)
        self._path_conversion_config = lula.TaskSpacePathConversionConfig()

    def get_all_frame_names(self) -> List[str]:
        """Return a list of all frames in the robot URDF that may be used to follow a trajectory

        Returns:
            List[str]: List of all frame names in the robot URDF
        """
        return self._lula_kinematics.frame_names()

    def compute_task_space_trajectory_from_points(
        self, positions: np.array, orientations: np.array, frame_name: str
    ) -> LulaTrajectory:
        """Return a LulaTrajectory that connects the provided positions and orientations at the specified frame in the robot.

        Args:
            positions (np.array): Taskspace positions that the robot end effector should pass through with shape (N x 3) where N is the number of provided positions.
                Positions is assumed to be in meters.
            orientations (np.array): Taskspace quaternion orientations that the robot end effector should pass through with shape (N x 4) where N is the number of provided
                orientations.  The length of this argument must match the length of the positions argument.
            frame_name (str): Name of the end effector frame in the robot URDF.
        
        Returns:
            LulaTrajectory: Instance of the omni.isaac.motion_generation.Trajectory class.  If no trajectory could be generated, None is returned.
        """

        if positions.shape[0] != orientations.shape[0]:
            carb.log_error(
                "Provided positions must have the same number of rows as provided orientations: one for each point in the task_space."
            )
            return None

        path_spec = lula.create_task_space_path_spec(get_pose3(positions[0], rot_quat=orientations[0]))

        for i in range(1, len(positions)):
            path_spec.add_linear_path(get_pose3(positions[i], rot_quat=orientations[i]))

        return self.compute_task_space_trajectory_from_path_spec(path_spec, frame_name)

    def compute_task_space_trajectory_from_path_spec(
        self, task_space_path_spec: lula.TaskSpacePathSpec, frame_name: str
    ) -> LulaTrajectory:
        """Return a LulaTrajectory that follows the path specified by the provided TaskSpacePathSpec

        Args:
            task_space_path_spec (lula.TaskSpacePathSpec): An object describing a taskspace path
            frame_name (str): Name of the end effector frame

        Returns:
            LulaTrajectory: Instance of the omni.isaac.motion_generation.Trajectory class.  If no trajectory could be generated, None is returned.
        """

        c_space_path = lula.convert_task_space_path_spec_to_c_space(
            task_space_path_spec, self._lula_kinematics, frame_name, self._path_conversion_config
        )

        if c_space_path is None:
            return None

        trajectory = self._c_space_trajectory_generator.generate_trajectory(c_space_path.waypoints())

        return LulaTrajectory(trajectory, self.get_active_joints())

    def get_path_conversion_config(self) -> lula.TaskSpacePathConversionConfig:
        """Get a reference to the config object that lula uses to convert task-space paths to c-space paths.

        The values of the returned TaskSpacePathConversionConfig object can be modified directly to affect lula task-space path conversions.
        See help(lula.TaskSpacePathConversionConfig) for a detailed description of the editable parameters.

        Returns:
            lula.TaskSpacePathConversionConfig: Configuration class for converting from task-space paths to c-space paths.
        """
        return self._path_conversion_config
