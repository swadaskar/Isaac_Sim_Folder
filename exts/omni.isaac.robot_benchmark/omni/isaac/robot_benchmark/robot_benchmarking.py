# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from pxr import UsdPhysics, Sdf, UsdLux, PhysxSchema
import omni.ext
import omni.usd

from omni.isaac.core.utils.stage import set_stage_up_axis
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
from omni.isaac.core import PhysicsContext
from omni.isaac.core import objects

import numpy as np


class RobotBenchmark:
    def __init__(self):
        self._timeline = omni.timeline.get_timeline_interface()

        self._first_step = True  # first step of simulation since things were reset or reloaded
        self.created = False  # robot has been created

        self._robot = None
        self._robot_articulation_controller = None

        self._controller = None

        self._environment = None
        # Variable counting the number of benchmark tests that have run so far to be used when running a specific number of tests from a script
        self.test_ind = 0

    def initialize_test(
        self, environment, robot_loader, controller_name, benchmark_logger=None, start_ind=0, enable_collisions=False
    ):
        """ 
        load robot from USD
        """

        self._stage = omni.usd.get_context().get_stage()

        set_stage_up_axis("z")
        PhysicsContext(physics_dt=1.0 / 60.0)

        self._ground_plane = objects.ground_plane.GroundPlane("/scene/ground_plane")

        self._robot_loader = robot_loader
        self._controller_name = controller_name

        self.robot_path = "/Robot"

        self._robot = self._robot_loader.load_robot(self.robot_path)

        light_prim = UsdLux.DistantLight.Define(self._stage, Sdf.Path("/World/defaultLight"))
        light_prim.CreateIntensityAttr(500)

        # get the frequency in Hz of the simulation
        physxSceneAPI = None
        for prim in self._stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(prim)
        if physxSceneAPI is not None:
            self.fps = physxSceneAPI.GetTimeStepsPerSecondAttr().Get()
        else:
            self.fps = 60

        self._first_step = True
        self.created = True
        self._testing = False
        self._environment = environment

        self._collisions_on = False

        self._benchmark_logger = benchmark_logger

        for _ in range(start_ind):
            self._environment.get_new_scenario()
        self.test_ind = start_ind

        self._ignore_target_orientation = True
        self._enable_collisions = enable_collisions

    def toggle_testing(self):
        if self._testing:
            self._testing = False
        else:
            self._testing = True
            self._initialize_new_scenario()

    def step(self, step):
        """This function is called every timestep in the editor

        Arguments:
            step (float): elapsed time between steps
        """
        if self.created and self._timeline.is_playing():
            if self._first_step:
                self._first_step = False
                self._setup_world()

            if self._testing and not self.start_target_reached:
                """
                test is considered to have started when the initial target is reached
                start_target is conceptually different from a waypoint
                it is a position that the robot is expected to easily acheive in the environment
                """
                self._forward(*self.start_target.get_world_pose())

                self.start_target_reached = self._reached_end_effector_target(*self.start_target.get_world_pose())

                if self.start_target_reached and self.waypoints:
                    waypoint = self.waypoints[self.waypoint_index]
                    waypoint.set_visibility(True)
                    self.start_target.set_visibility(False)

            elif self._testing:
                # environment may change as a function of time once the robot is in place
                self._environment.update()
                self._log_frame()

                if not self.waypoints:
                    # just keep following start_target prim until timeout
                    self._forward(*self.start_target.get_world_pose())
                    if self._test_frame / self.fps >= self.test_timeout:
                        self._environment.reset(new_seed=self._environment.random_seed + 1)
                        self._initialize_new_scenario()
                        self._log_header(None)
                    else:
                        self._test_frame += 1

                else:
                    # follow a series of waypoints until timeout or completion
                    waypoint = self.waypoints[self.waypoint_index]
                    self._forward(*waypoint.get_world_pose())

                    if self._reached_end_effector_target(*waypoint.get_world_pose()):
                        waypoint.set_visibility(False)
                        self.waypoint_index += 1
                        if self.waypoint_index == len(self.waypoints):
                            self._log_header(True)
                            self.test_ind += 1
                            self._initialize_new_scenario()
                        else:
                            waypoint = self.waypoints[self.waypoint_index]
                            waypoint.set_visibility(True)
                    elif self._test_frame / self.fps >= self.test_timeout:
                        waypoint.set_visibility(False)
                        self._log_header(False)
                        self.test_ind += 1
                        self._initialize_new_scenario()
                    else:
                        self._test_frame += 1

    def _forward(self, target_trans, target_rot):
        if self._ignore_target_orientation:
            action = self._controller.forward(target_trans)
        else:
            action = self._controller.forward(target_trans, target_rot)

        self._robot_articulation_controller.apply_action(action)

    def reset(self):
        self._testing = False
        self._test_frame = 0
        self.test_ind = 0

        if self._environment is not None:
            self._environment.reset()

        self._first_step = True

    def stop_tasks(self):
        self._robot = None
        self._first_step = True
        self.created = False

    def _setup_world(self):
        self._robot.initialize()
        default_pos = self._environment.initial_robot_cspace_position
        if default_pos is not None:
            self._robot.set_joints_default_state(positions=default_pos)

        self._robot_articulation_controller = self._robot.get_articulation_controller()

        self._controller = self._robot_loader.load_controller(self._controller_name)
        self._controller.reset()
        self._controller.set_robot_base_pose(*self._robot.get_world_pose())

        self.obstacles = self._environment.get_all_obstacles()

        for obstacle in self.obstacles:
            self._controller.add_obstacle(obstacle)

        self._controller.add_obstacle(self._ground_plane)

        self._toggle_collisions(False)

    def _reached_end_effector_target(self, target_trans, target_orient):
        ee_trans, ee_rot = self._controller.get_current_end_effector_pose()

        trans_thresh, rot_thresh = self._environment.get_target_thresholds()

        if self._ignore_target_orientation:
            target_orient = None

        if target_orient is not None:
            target_rot = quat_to_rot_matrix(target_orient)
        else:
            target_rot = None

        if target_rot is None and target_trans is None:
            return True
        elif target_rot is None:
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_trans)
            return trans_dist < trans_thresh
        elif target_trans is None:
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
            return rot_dist < rot_thresh
        else:
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_trans)
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
            return trans_dist < trans_thresh and rot_dist < rot_thresh

    def _toggle_collisions(self, turn_on=True):
        if self._collisions_on and not turn_on:
            self._environment.disable_collisions()
            self._collisions_on = False

        elif not self._collisions_on and turn_on:
            self._environment.enable_collisions()
            self._collisions_on = True

    def _initialize_new_scenario(self):
        if self._controller is None:
            carb.log_error("Attempted to start new scenario before test was initialized")

        self.start_target, self.waypoints, self.test_timeout = self._environment.get_new_scenario()

        default_robot_state = self._robot.get_joints_default_state()

        self._robot.set_joint_positions(default_robot_state.positions)
        self._robot.set_joint_velocities(default_robot_state.velocities)

        """
        If start_target is None, the scenario is considered to be completed immediately (nothing happens)
        """

        self._toggle_collisions(self._enable_collisions)

        if self.start_target is None:
            self._testing = False
            return

        self.waypoint_index = 0  # on waypoint 0 in test
        self.start_target_reached = False
        self.end_target_reached = False
        self._test_frame = 0  # count of frames passed in test

        if self._benchmark_logger is not None:
            # start saving a new test
            self._benchmark_logger.new_test()

    def _log_frame(self):
        """
        the benchmark logger object accepts dictionaries to describe every frame
        any primitive type or iterable containing primitive types is supported as an argument

        the resulting json is written as a list of dictionaries for the frames of a test
        """

        if self._benchmark_logger is None:
            return

        if not self.waypoints:
            target = self.start_target
        else:
            target = self.waypoints[self.waypoint_index]

        target_pos, target_rot = target.get_world_pose()
        if self._ignore_target_orientation:
            target_rot = None
        ee_pos, ee_rot = self._controller.get_current_end_effector_pose()

        frame_descriptor = {
            "robot_cspace_config": self._robot.get_joint_positions(),
            "ee_pos": ee_pos,
            "ee_rot": ee_rot,
            "target_pos": target_pos,
            "target_rot": target_rot,
            "frame_number": self._test_frame,
        }
        self._benchmark_logger.log_frame(**frame_descriptor)

    def _log_header(self, success):
        """
        each test can have one header associated with it to describe overarching information

        writing a header to a test that already has one will replace the old header
        """

        if self._benchmark_logger is None:
            return
        waypoint_poses = []
        waypoint_rots = []
        if self.waypoints:
            for waypoint in self.waypoints:
                waypoint_pos, waypoint_rot = waypoint.get_world_pose()
                waypoint_poses.append(waypoint_pos)
                waypoint_rots.append(waypoint_rot)
        header = {
            "success": success,
            "waypoint_poses": waypoint_poses,
            "waypoint_rots": waypoint_rots,
            "env_name": self._environment.name,
            "robot_name": self._robot_loader.get_name(),
            "controller_name": self._controller._name,
            "test_index": self.test_ind,
            "fps": self.fps,
        }
        self._benchmark_logger.log_header(**header)
