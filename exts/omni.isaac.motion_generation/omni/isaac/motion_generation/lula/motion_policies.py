# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
import time
from typing import Tuple, List, Union

import lula
import carb
from ..motion_policy_interface import MotionPolicy
from .interface_helper import LulaInterfaceHelper
from .kinematics import LulaKinematicsSolver
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid, delete_prim
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from omni.isaac.core.utils.math import normalized
from omni.isaac.core import objects
from pxr import Sdf


class RmpFlow(LulaInterfaceHelper, MotionPolicy):
    """
    RMPflow is a real-time, reactive motion policy that smoothly guides a robot to task space targets while avoiding dynamic obstacles.  
    This class implements the MotionPolicy interface, as well as providing a number of RmpFlow-specific functions such as visualizing 
    the believed robot position and changing internal settings.

    Args:
        robot_description_path (str): Path to a robot description yaml file
        urdf_path (str): Path to robot urdf
        rmpflow_config_path (str): Path to an rmpflow parameter yaml file
        end_effector_frame_name (str): Name of the robot end effector frame (must be present in the robot urdf)
        maximum_substep_size (float): Maximum substep size [sec] that RmpFlow will use when internally integrating between steps of a simulation.  For stability and performance,
            RmpFlow rolls out the robot actions at a higher framerate than Isaac Sim.  For example, while Isaac Sim may be running at 60 Hz, RmpFlow can be set to take internal 
            steps that are no larger than 1/300 seconds.  In this case, RmpFlow will perform 5 sub-steps every time it returns an action to the 60 Hz simulation.  
            
            In general, the maximum_substep_size argument should be at most 1/200.  Choosing a very small maximum_substep_size such as 1/1000 is unnecessary, as the resulting actions will not
            significantly differ from a choice of 1/500, but it will internally require twice the steps to compute.
        ignore_robot_state_updates (bool): Defaults to False.
            If False: RmpFlow will set the internal robot state to match the arguments to compute_joint_targets().  When paired with ArticulationMotionPolicy, this means that RMPflow uses the simulated robot's state at every frame.
            If True: RmpFlow will roll out the robot state internally after it is initially specified in the first call to compute_joint_targets().  
    """

    def __init__(
        self,
        robot_description_path: str,
        urdf_path: str,
        rmpflow_config_path: str,
        end_effector_frame_name: str,
        maximum_substep_size: float,
        ignore_robot_state_updates=False,
    ) -> None:

        self.maximum_substep_size = maximum_substep_size
        if maximum_substep_size <= 0:
            carb.log_error("maximum_substep_size argument must be positive.")

        self.ignore_robot_state_updates = ignore_robot_state_updates

        self.end_effector_frame_name = end_effector_frame_name

        MotionPolicy.__init__(self)

        robot_description = lula.load_robot(robot_description_path, urdf_path)

        LulaInterfaceHelper.__init__(self, robot_description)

        self._rmpflow_config_path = rmpflow_config_path
        # Create RMPflow configuration.
        rmpflow_config = lula.create_rmpflow_config(
            rmpflow_config_path, self._robot_description, self.end_effector_frame_name, self._world.add_world_view()
        )

        # Create RMPflow policy.
        self._policy = lula.create_rmpflow(rmpflow_config)

        self._robot_joint_positions = None
        self._robot_joint_velocities = None

        self._end_effector_position_target = None
        self._end_effector_rotation_target = None

        self._collision_spheres = []
        self._ee_visual = None

    def set_ignore_state_updates(self, ignore_robot_state_updates) -> None:
        """An RmpFlow specific method; set an internal flag in RmpFlow: ignore_robot_state_updates

        Args:
            ignore_robot_state_updates (bool): 
                If False: 
                    RmpFlow will set the internal robot state to match the arguments to compute_joint_targets().  
                    When paired with ArticulationMotionPolicy, this means that RMPflow uses the simulated robot's state at every frame.
                If True: 
                    RmpFlow will roll out the robot state internally after it is initially specified in the first call to compute_joint_targets().  
                    The caller may override this flag and directly change the internal robot state with RmpFlow.set_internal_robot_joint_states().
        """
        self.ignore_robot_state_updates = ignore_robot_state_updates

    def set_cspace_target(self, active_joint_targets) -> None:
        """Set a cspace target for RmpFlow.  RmpFlow always has a cspace target, and setting a new cspace target does not override a position target.
        RmpFlow uses the cspace target to help resolve null space behavior when a position target can be acheived in a variety of ways.
        If the end effector target is explicitly set to None, RmpFlow will move the robot to the cspace target

        Args:
            active_joint_targets (np.array): cspace position target for active joints in the robot
        """
        self._policy.set_cspace_attractor(active_joint_targets.astype(np.float64))

    def update_world(self, updated_obstacles: List = None) -> None:
        LulaInterfaceHelper.update_world(self, updated_obstacles)
        self._policy.update_world_view()

    def compute_joint_targets(
        self,
        active_joint_positions: np.array,
        active_joint_velocities: np.array,
        watched_joint_positions: np.array,
        watched_joint_velocities: np.array,
        frame_duration: float,
    ) -> Tuple[np.array, np.array]:
        """Compute robot joint targets for the next frame based on the current robot position.
        RmpFlow will ignore active joint positions and velocities if it has been set to ignore_robot_state_updates
        RmpFlow does not currently support watching joints that it is not actively controlling. 

        Args:
            active_joint_positions (np.array): current positions of joints specified by get_active_joints()
            active_joint_velocities (np.array): current velocities of joints specified by get_active_joints()
            watched_joint_positions (np.array): current positions of joints specified by get_watched_joints()
                This will always be empty for RmpFlow.
            watched_joint_velocities (np.array): current velocities of joints specified by get_watched_joints()
                This will always be empty for RmpFlow.
            frame_duration (float): duration of the physics frame

        Returns:
            Tuple[np.array,np.array]:
            active_joint_position_targets : Position targets for the robot in the next frame 

            active_joint_velocity_targets : Velocity targets for the robot in the next frame 
        """

        self._update_robot_joint_states(active_joint_positions, active_joint_velocities, frame_duration)
        return self._robot_joint_positions, self._robot_joint_velocities

    def visualize_collision_spheres(self) -> None:
        """An RmpFlow specific debugging method.  This function creates visible sphere prims that match the locations and radii
        of the collision spheres that RmpFlow uses to prevent robot collisions.  Once created, RmpFlow will update the sphere locations
        whenever its internal robot state changes.  This can be used alongside RmpFlow.ignore_robot_state_updates(True) to validate RmpFlow's
        internal representation of the robot as well as help tune the PD gains on the simulated robot; i.e. the simulated robot should
        match the positions of the RmpFlow collision spheres over time.  

        Visualizing collision spheres as prims on the stage is likely to significantly slow down the framerate of the simulation.  This function should only be used for debugging purposes
        """

        if len(self._collision_spheres) == 0:
            self._create_collision_sphere_prims(True)
        else:
            with Sdf.ChangeBlock():
                for sphere in self._collision_spheres:
                    sphere.set_visibility(True)

    def visualize_end_effector_position(self) -> None:
        """An RmpFlow specific debugging method.  This function creates a visible cube whose translation and orientation match where RmpFlow
        believes the robot end effector to be.  Once created, RmpFlow will update the position of the cube whenever its internal robot state changes.
        """

        if self._ee_visual is None:
            self._create_ee_visual(True)
        else:
            self._ee_visual.set_visibility(True)

    def stop_visualizing_collision_spheres(self) -> None:
        """An RmpFlow specific debugging method.  This function removes the collision sphere prims created by either RmpFlow.visualize_collision_spheres() or 
        RmpFlow.get_collision_spheres_as_prims().  Rather than making the prims invisible, they are deleted from the stage to increase performance
        """
        self.delete_collision_sphere_prims()
        self._collision_spheres = []

    def stop_visualizing_end_effector(self) -> None:
        """An RmpFlow specific debugging method.  This function removes the end effector prim that can be created by visualize_end_effector_position() or
        get_end_effector_position_as_prim()
        """
        self.delete_end_effector_prim()

    def get_collision_spheres_as_prims(self) -> List:
        """An RmpFlow specific debugging method.  This function is similar to RmpFlow.visualize_collision_spheres().  If the collision spheres have already been added to the stage as prims,
        they will be returned.  If the collision spheres have not been added to the stage as prims, they will be created and returned.  If created in this function, the spheres will be invisible
        until RmpFlow.visualize_collision_spheres() is called.  

        Visualizing collision spheres on the stage is likely to significantly slow down the framerate of the simulation.  This function should only be used for debugging purposes

        Returns:
            collision_spheres (List[core.objects.sphere.VisualSphere]): List of prims representing RmpFlow's internal collision spheres 

        """

        if len(self._collision_spheres) == 0:
            self._create_collision_sphere_prims(False)

        return self._collision_spheres

    def get_end_effector_as_prim(self) -> objects.cuboid.VisualCuboid:
        """An RmpFlow specific debugging method.  This function is similar to RmpFlow.visualize_end_effector_position().  If the end effector has already been visualized as a prim,
        it will be returned.  If the end effector is not being visualized, a cuboid will be created and returned.  If created in this function, the end effector will be invisible
        until RmpFlow.visualize_end_effector_position() is called.  

        Returns:
            end_effector_prim (objects.cuboid.VisualCuboid): Cuboid whose translation and orientation match RmpFlow's believed robot end effector position.
        """
        if self._ee_visual is not None:
            return self._ee_visual

        self._create_ee_visual(False)
        return self._ee_visual

    def delete_collision_sphere_prims(self) -> None:
        """An RmpFlow specific debugging method.  This function deletes any prims that have been created by RmpFlow to visualize its internal collision spheres
        """
        for sphere in self._collision_spheres:
            delete_prim(sphere.prim_path)

        self._collision_spheres = []

    def delete_end_effector_prim(self) -> None:
        """An RmpFlow specific debugging method.  If RmpFlow is maintaining a prim for its believed end effector position, this function will delete the prim.
        """
        if self._ee_visual is not None:
            delete_prim(self._ee_visual.prim_path)

        self._ee_visual = None

    def reset(self) -> None:
        """Reset RmpFlow to its initial state
        """

        LulaInterfaceHelper.reset(self)

        rmpflow_config = lula.create_rmpflow_config(
            self._rmpflow_config_path,
            self._robot_description,
            self.end_effector_frame_name,
            self._world.add_world_view(),
        )
        self._policy = lula.create_rmpflow(rmpflow_config)

        self._robot_joint_positions = None
        self._robot_joint_velocities = None

        self._end_effector_position_target = None
        self._end_effector_rotation_target = None

        self.configure_visualize = False

        self.delete_collision_sphere_prims()
        self.delete_end_effector_prim()

        self._collision_spheres = []
        self._ee_visual = None

    def set_internal_robot_joint_states(
        self,
        active_joint_positions: np.array,
        active_joint_velocities: np.array,
        watched_joint_positions: np.array,
        watched_joint_velocities: np.array,
    ) -> None:
        """An RmpFlow specific method; this function overwrites the robot state regardless of the ignore_robot_state_updates flag.
        RmpFlow does not currently support watching joints that it is not actively controlling. 

        Args:
            active_joint_positions (np.array): current positions of joints specified by get_active_joints()
            active_joint_velocities (np.array): current velocities of joints specified by get_active_joints()
            watched_joint_positions (np.array): current positions of joints specified by get_watched_joints().  
                This will always be empty for RmpFlow. 
            watched_joint_velocities (np.array): current velocities of joints specified by get_watched_joints()
                This will always be empty for RmpFlow.
        """

        self._robot_joint_positions = active_joint_positions
        self._robot_joint_velocities = active_joint_velocities

        self._update_visuals()
        return

    def get_internal_robot_joint_states(self) -> Tuple[np.array, np.array, np.array, np.array]:
        """An RmpFlow specific method; this function returns the internal robot state that is believed by RmpFlow

        Returns:
            Tuple[np.array,np.array,np.array,np.array]:

            active_joint_positions: believed positions of active joints 

            active_joint_velocities: believed velocities of active joints 

            watched_joint_positions: believed positions of watched robot joints.  This will always be empty for RmpFlow. 

            watched_joint_velocities: believed velocities of watched robot joints.  This will always be empty for RmpFlow. 

        """

        return self._robot_joint_positions, self._robot_joint_velocities, np.empty(0), np.empty(0)

    def get_default_cspace_position_target(self):
        """An RmpFlow specific method; this function returns the default cspace position specified in the
            Lula robot_description YAML file

        Returns:
            np.array: Default cspace position target used by RMPflow when none is specified.
        """
        return self._robot_description.default_c_space_configuration()

    def get_active_joints(self) -> List[str]:
        """Returns a list of joint names that RmpFlow is controlling.  

        Some articulated robot joints may be ignored by some policies. E.g., the gripper of the Franka arm is not used 
        to follow targets, and the RmpFlow config files excludes the joints in the gripper from the list of active
        joints.

        Returns:
            active_joints (List[str]): Names of active joints.  
                The order of the joints in this list matches the order that the joints are expected 
                in functions like RmpFlow.compute_joint_targets(active_joint_positions, active_joint_velocities,...)
        """
        return LulaInterfaceHelper.get_active_joints(self)

    def get_watched_joints(self) -> List[str]:
        """Currently, RmpFlow is not capable of watching joint states that are not being directly controlled (active joints)
        If RmpFlow is controlling a robot arm at the end of an externally controlled body, set_robot_base_pose() can be used to make RmpFlow aware of the robot position
        This means that RmpFlow is not currently able to support controlling a set of DOFs in a robot that are not sequentially linked to each other or are not connected 
        via fixed transforms to the end effector.  

        Returns:
            watched_joints (List[str]): Empty list
        """
        return []

    def get_end_effector_pose(self, active_joint_positions: np.array) -> Tuple[np.array, np.array]:
        return LulaInterfaceHelper.get_end_effector_pose(self, active_joint_positions, self.end_effector_frame_name)

    def get_kinematics_solver(self) -> LulaKinematicsSolver:
        """Return a LulaKinematicsSolver that uses the same robot description as RmpFlow.  The robot base pose of the LulaKinematicsSolver
        will be set to the same base pose as RmpFlow, but the two objects must then have their base poses updated separately.

        Returns:
            LulaKinematicsSolver: Kinematics solver using the same cspace as RmpFlow
        """
        solver = LulaKinematicsSolver(None, None, robot_description=self._robot_description)
        solver.set_robot_base_pose(self._robot_pos / self._meters_per_unit, rot_matrices_to_quats(self._robot_rot))

        return solver

    def set_end_effector_target(self, target_position=None, target_orientation=None) -> None:
        __doc__ = MotionPolicy.set_end_effector_target.__doc__

        if target_orientation is not None:
            target_rotation = quats_to_rot_matrices(target_orientation)
        else:
            target_rotation = None

        if target_position is not None:
            self._end_effector_position_target = target_position * self._meters_per_unit
        else:
            self._end_effector_position_target = None

        self._end_effector_rotation_target = target_rotation

        self._set_end_effector_target()

    def set_robot_base_pose(self, robot_position: np.array, robot_orientation: np.array) -> None:
        LulaInterfaceHelper.set_robot_base_pose(self, robot_position, robot_orientation)
        self._set_end_effector_target()

    def add_obstacle(self, obstacle: objects, static: bool = False) -> bool:
        __doc__ = MotionPolicy.add_obstacle.__doc__
        return MotionPolicy.add_obstacle(self, obstacle, static)

    def add_cuboid(
        self,
        cuboid: Union[objects.cuboid.DynamicCuboid, objects.cuboid.FixedCuboid, objects.cuboid.VisualCuboid],
        static: bool = False,
    ) -> bool:
        return LulaInterfaceHelper.add_cuboid(self, cuboid, static)

    def add_sphere(
        self, sphere: Union[objects.sphere.DynamicSphere, objects.sphere.VisualSphere], static: bool = False
    ) -> bool:
        return LulaInterfaceHelper.add_sphere(self, sphere, static)

    def add_capsule(
        self, capsule: Union[objects.capsule.DynamicCapsule, objects.capsule.VisualCapsule], static: bool = False
    ) -> bool:
        return LulaInterfaceHelper.add_capsule(self, capsule, static)

    def add_ground_plane(self, ground_plane: objects.ground_plane.GroundPlane) -> bool:
        return LulaInterfaceHelper.add_ground_plane(self, ground_plane)

    def disable_obstacle(self, obstacle: objects) -> bool:
        return LulaInterfaceHelper.disable_obstacle(self, obstacle)

    def enable_obstacle(self, obstacle: objects) -> bool:
        return LulaInterfaceHelper.enable_obstacle(self, obstacle)

    def remove_obstacle(self, obstacle: objects) -> bool:
        return LulaInterfaceHelper.remove_obstacle(self, obstacle)

    def _set_end_effector_target(self):
        target_position = self._end_effector_position_target
        target_rotation = self._end_effector_rotation_target

        if target_position is None and target_rotation is None:
            self._policy.clear_end_effector_position_attractor()
            self._policy.clear_end_effector_orientation_attractor()
            return

        trans, rot = LulaInterfaceHelper._get_pose_rel_robot_base(self, target_position, target_rotation)

        if trans is not None:
            self._policy.set_end_effector_position_attractor(trans)
        else:
            self._policy.clear_end_effector_position_attractor()

        if rot is not None:
            self._policy.set_end_effector_orientation_attractor(lula.Rotation3(rot))
        else:
            self._policy.clear_end_effector_orientation_attractor()

    def _create_ee_visual(self, is_visible):
        if self._robot_joint_positions is None:
            joint_positions = np.zeros(self._robot_description.num_c_space_coords())
        else:
            joint_positions = self._robot_joint_positions

        ee_pos, rot_mat = self.get_end_effector_pose(joint_positions)
        prim_path = find_unique_string_name("/lula/end_effector", lambda x: not is_prim_path_valid(x))
        self._ee_visual = objects.cuboid.VisualCuboid(prim_path, size=0.1 / self._meters_per_unit)
        self._ee_visual.set_world_pose(position=ee_pos, orientation=rot_matrices_to_quats(rot_mat))
        self._ee_visual.set_visibility(is_visible)

    def _create_collision_sphere_prims(self, is_visible):
        if self._robot_joint_positions is None:
            joint_positions = self._robot_description.default_c_space_configuration()
        else:
            joint_positions = self._robot_joint_positions.astype(np.float64)

        sphere_poses = self._policy.collision_sphere_positions(joint_positions)
        sphere_radii = self._policy.collision_sphere_radii()
        for i, (sphere_pose, sphere_rad) in enumerate(zip(sphere_poses, sphere_radii)):
            prim_path = find_unique_string_name("/lula/collision_sphere" + str(i), lambda x: not is_prim_path_valid(x))
            self._collision_spheres.append(
                objects.sphere.VisualSphere(prim_path, radius=sphere_rad / self._meters_per_unit)
            )
        with Sdf.ChangeBlock():
            for sphere, sphere_pose in zip(self._collision_spheres, sphere_poses):
                sphere.set_world_pose(sphere_pose / self._meters_per_unit)
                sphere.set_visibility(is_visible)

    def _update_collision_sphere_prims(self):
        if len(self._collision_spheres) == 0:
            return

        joint_positions = self._robot_joint_positions.astype(np.float64)

        sphere_poses = self._policy.collision_sphere_positions(joint_positions)

        for col_sphere, new_pose in zip(self._collision_spheres, sphere_poses):
            col_sphere.set_world_pose(position=new_pose / self._meters_per_unit)

    def _update_end_effector_prim(self):
        if self._ee_visual is None:
            return

        ee_pos, rot_mat = self.get_end_effector_pose(self._robot_joint_positions)
        self._ee_visual.set_world_pose(ee_pos, rot_matrices_to_quats(rot_mat))

    def _update_visuals(self):
        with Sdf.ChangeBlock():
            self._update_collision_sphere_prims()
            self._update_end_effector_prim()

    def _update_robot_joint_states(self, joint_positions, joint_velocities, frame_duration):
        if (
            self._robot_joint_positions is None
            or self._robot_joint_velocities is None
            or not self.ignore_robot_state_updates
        ):
            self._robot_joint_positions, self._robot_joint_velocities = self._euler_integration(
                joint_positions, joint_velocities, frame_duration
            )
        else:
            self._robot_joint_positions, self._robot_joint_velocities = self._euler_integration(
                self._robot_joint_positions, self._robot_joint_velocities, frame_duration
            )
        self._update_visuals()

    def _euler_integration(self, joint_positions, joint_velocities, frame_duration):
        num_steps = np.ceil(frame_duration / self.maximum_substep_size).astype(int)
        policy_timestep = frame_duration / num_steps

        for i in range(num_steps):
            joint_accel = self._evaluate_acceleration(joint_positions, joint_velocities)
            joint_positions += policy_timestep * joint_velocities
            joint_velocities += policy_timestep * joint_accel

        return joint_positions, joint_velocities

    def _evaluate_acceleration(self, joint_positions, joint_velocities):
        joint_positions = joint_positions.astype(np.float64)
        joint_velocities = joint_velocities.astype(np.float64)
        joint_accel = np.zeros_like(joint_positions)
        self._policy.eval_accel(joint_positions, joint_velocities, joint_accel)
        return joint_accel


class RmpFlowSmoothed(RmpFlow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.desired_speed_scalar = 1.0
        self.speed_scalar = 1.0
        self.time_at_last_jerk_reduction = None
        self.qdd = None

        # Params
        self.min_time_between_jerk_reductions = 0.5
        self.min_speed_scalar = 0.2
        self.use_big_jerk_speed_scaling = True
        self.big_jerk_limit = 10.0
        self.use_medium_jerk_truncation = True
        self.max_medium_jerk = 7.0
        self.speed_scalar_alpha_blend = 0.985  # Used for real world experiments.
        self.verbose = False

    def _eval_speed_scaled_accel(self, joint_positions, joint_velocities):
        qdd_eval = self._evaluate_acceleration(joint_positions, joint_velocities / (self.speed_scalar))
        qdd_eval *= self.speed_scalar ** 2

        return qdd_eval

    def _euler_integration(self, joint_positions, joint_velocities, frame_duration):
        num_steps = np.ceil(frame_duration / self.maximum_substep_size).astype(int)
        step_dt = frame_duration / num_steps

        q = joint_positions
        qd = joint_velocities

        # Jerk monitoring and reduction is intended to handle jerk in physical robots. It's
        # important then to use real wall-clock time when monitoring it.
        now = time.time()

        for i in range(num_steps):
            if self.qdd is None:
                self.qdd = self._eval_speed_scaled_accel(q, qd)
                continue

            jerk_reduction_performed = False

            # Reduces the speed down to a minimum if a big jerk is experience.
            if self.use_big_jerk_speed_scaling:
                is_first = True
                while True:
                    qdd_eval = self._eval_speed_scaled_accel(q, qd)

                    # Just go through this once. We simply want to make sure qdd_eval is evaluated
                    # again after the reduction.
                    if not is_first:
                        break

                    # Don't do jerk reductions too frequently.
                    if (
                        self.time_at_last_jerk_reduction is not None
                        and (now - self.time_at_last_jerk_reduction) < self.min_time_between_jerk_reductions
                    ):
                        break

                    jerk = np.linalg.norm(qdd_eval - self.qdd)
                    if jerk > self.big_jerk_limit:
                        self.speed_scalar = self.min_speed_scalar
                        if self.verbose:
                            print("<jerk reduction> new speed scalar = %f" % self.speed_scalar)
                        jerk_reduction_performed = True

                    is_first = False

            # Truncate the jerks. This addresses transient jerks.
            if self.use_medium_jerk_truncation:
                qdd_eval = self._eval_speed_scaled_accel(q, qd)

                jerk = np.linalg.norm(qdd_eval - self.qdd)
                if jerk > self.max_medium_jerk:
                    if self.verbose:
                        print("<jerk truncation>")
                    jerk_truncation_performed = True
                    v = normalized(qdd_eval - self.qdd)
                    qdd_eval = self.qdd + self.max_medium_jerk * v

            if jerk_reduction_performed:
                self.time_at_last_jerk_reduction = now

            self.qdd = qdd_eval

            a = self.speed_scalar_alpha_blend
            self.speed_scalar = a * self.speed_scalar + (1.0 - a) * self.desired_speed_scalar

            q += step_dt * qd
            qd += step_dt * self.qdd

        return q, qd
