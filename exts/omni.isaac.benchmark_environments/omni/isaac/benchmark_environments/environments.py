# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


import numpy as np
from scipy.spatial.transform import Rotation as R

from .environment_interface import Environment
from .objects import *

"""
The environment creator is used by the UI in Isaac Sim for the automatic discovery of environments
    that appear in a drop down menu.  Some environments may not be available for certain robots, and 
    the environment creator stores any relevant excluding of robots per environment.

The creator is also used to validate the choice of environment when testing from a standalone script.
"""


class EnvironmentCreator:
    def __init__(self):
        self.environments = ["Static Cage", "Cubby", "Window", "Evasion", "Guillotine"]

        self.robot_exclusion_lists = {"Windmill": ["UR10"]}

        self.policy_exclusion_list = {}

    def create_environment(self, env_name, random_seed=0, **kwargs):
        if env_name == "Cubby":
            return CubbyEnv(random_seed=random_seed, **kwargs)
        elif env_name == "Evasion":
            return EvasionEnv(random_seed=random_seed, **kwargs)
        elif env_name == "Window":
            return WindowEnv(random_seed=random_seed, **kwargs)
        elif env_name == "Windmill":
            return WindmillEnv(random_seed=random_seed, **kwargs)
        elif env_name == "Guillotine":
            return GuillotineEnv(random_seed=random_seed, **kwargs)
        elif env_name == "Static Cage":
            return CageEnv(random_seed=random_seed, **kwargs)
        elif env_name == "Empty":
            return EmptyEnv(random_seed=random_seed, **kwargs)
        else:
            return None

    def has_environment(self, env_name):
        return env_name in self.environments

    def get_environment_names(self):
        return self.environments

    def get_robot_exclusion_list(self, env_name):
        # some robots should not be used in certain environments
        return self.robot_exclusion_lists.get(env_name, [])

    def get_motion_policy_exclusion_list(self, env_name):
        # some motion policies should not be used in certain environments

        # For example, global motion policies with slow update rates may not make sense
        # for environments with quickly moving obstacles or targets.

        return self.policy_exclusion_list.get(env_name, [])


"""
See comments above the Environment class in ./template_classes.py for information about the general behavior of all environments.
    These comments explain some implementation details when implementing the Environment class.

Different environments are implemented below.  Each environment has **kwargs that it can accept as an argument.  Each robot has an environment
    config file with the desired kwargs for each environment.  Any kwargs that are explicitly listed in the environment config file
    override the default kwargs seen in the kwargs.get() lines.

There is not a rigid style for naming kwargs or deciding what kwargs are in a certain environment.  The environments are not made with the 
    assumption that every possible detail will be specified by a kwarg.  

    Most position information can be passed in as a kwarg, but the relative 
    positions of some objects may not be changeable.  There are some magic numbers in the placement of certain objects when it would overcomplicate 
    things to use kwargs and it is not likely that the param will need to change.

    Most parameters that are chosen arbitrarily, such as the speed of moving obstacles, are given as kwargs.

As a convention, environments are displaced from the robot along the positive x axis, and mostly centered about the y axis.
    Some robots (such as the UR10) may need the environment to be rotated about the robot.  The UR10's environment config file
    rotates each environment by pi/2 about the z axis.

In general, the code for these environments are not written with the expectation that the user will never need to touch the code
    for their desired custom behavior.  The code serves as more of an example for how to create different behaviors in different 
    environments as needed.  
"""


class CubbyEnv(Environment):
    def build_environment(self, **kwargs):
        """
        Kwargs:
            name: human-readable name of this environment (default: "Cubby")
            initial_robot_cspace_position: If specified, this parameter can be accessed to
                determine a good starting cspace position for a robot in this environment
            timeout: simulated time (in seconds) before a test times out (default: 10)
            collidable: turn on collision detection for the cubby (default: False)

            target_translation_thresh: threshold for how close the robot end effector must be to
                translation targets in meters (default: .03)
            target_rotation_thresh:  threshold for how close the robot end effector must be to
                rotation targets in radians (default: .1)

            base_rotation: axis-angle rotation of the cubby base (default: [0,0,0])
            base_offset: 3d translation of the cubby base (default [45.0,0,0])
            depth: The depth of the cubby shelves (default 30)
            target_depth: how deep the target is into the cubby (default half the depth)
        """
        self.name = kwargs.get("name", "Cubby")

        self.initial_robot_cspace_position = kwargs.get("initial_robot_cspace_position", None)

        self.timeout = kwargs.get("timeout", 20)

        self.target_translation_thresh = kwargs.get("target_translation_thresh", 0.03)
        self.target_rotation_thresh = kwargs.get("target_rotation_thresh", 0.1)

        cub_rot = R.from_rotvec(kwargs.get("base_rotation", [0, 0, 0]))
        base_offset = np.array(kwargs.get("base_offset", [0.45, 0.0, 0]))
        depth = kwargs.get("depth", 0.30)
        target_depth = kwargs.get("target_depth", depth / 2)

        self.require_orientation = kwargs.get("require_orientation", True)  # add an orientation target

        cub_pos = base_offset + np.array([0, 0, 0.20])
        c = Cubbies(cub_pos, cub_rot.as_matrix(), require_orientation=False, depth=depth, target_depth=target_depth)
        target_pos = cub_pos / 2 + np.array([0.0, 0.0, c.height / 2])
        self.start_target = Target(
            target_pos,
            R.from_rotvec([0, np.pi, 0]).as_matrix(),
            target_color=np.array([0, 0, 1.0]),
            require_orientation=True,
        )

        base_pos = base_offset + np.array([0, 0, 0.10])
        cubby_base = Block(base_pos, cub_rot.as_matrix(), size=np.array([c.depth, c.width, 0.20]))

        self.objects.append(c)
        self.objects.append(cubby_base)
        self.targetable_objects.append(c)

    def get_new_scenario(self):
        t1 = self.get_random_target(make_visible=False)
        t2 = self.get_random_target(make_visible=False)
        while t2 == t1:
            t2 = self.get_random_target(make_visible=False)

        waypoints = [t1, t2]

        return self.start_target.get_target(make_visible=True), waypoints, self.timeout


class EvasionEnv(Environment):
    def build_environment(self, **kwargs):
        """
        Kwargs:
            name: human-readable name of this environment (default: "Evasion")
            timeout: simulated time (in seconds) before a test times out (default: 300)

            target_translation_thresh: threshold for how close the robot end effector must be to
                translation targets in meters (default: .03)
            target_rotation_thresh:  threshold for how close the robot end effector must be to
                rotation targets in radians (default: .1)

            target_speed: speed of the target along a sinusoidal path in rad/sec (default: .2)
            block_speeds: 6d vector of speeds for the blocks moving along
                sinusoidal paths in rad/sec (default: np.linspace(.1, .3, num=6))
            block_sizes: 6d vector of the sizes of each block (default [15,15,15,15,15,15])

            frames_per_update: number of frames that should pass before obstacle/target positions
                are updated (default: 1) -- This approximates lag in perception
            target_position_std_dev: standard deviation of target position offset from the set path
                 on every frame (default: 0)
            obstacle_position_std_dev: standard deviation of obstacle positions offset from their set
                path on every frame (default: 0)

            height: height of target/average heights of obstacles off the ground
        """

        self.name = kwargs.get("name", "Evasion")

        self.initial_robot_cspace_position = kwargs.get("initial_robot_cspace_position", None)

        self.timeout = kwargs.get("timeout", 300)

        self.target_translation_thresh = kwargs.get("target_translation_thresh", 0.03)
        self.target_rotation_thresh = kwargs.get("target_rotation_thresh", 0.1)

        self.target_speed = kwargs.get("target_speed", 0.2)  # rad/sec
        # blocks move sinusoidally with the given frequencies
        self.block_speeds = np.array(kwargs.get("block_speeds", np.linspace(0.1, 0.3, num=6)))
        self.block_sizes = np.array(kwargs.get("block_sizes", [0.15] * 6), dtype=int)  # the size of each block

        # Update postions once every frame by default.
        self.frames_per_update = kwargs.get("frames_per_update", 1)
        # std_dev of gaussian noise to target position
        self.target_position_std_dev = kwargs.get("target_position_std_dev", 0)
        self.obstacle_position_std_dev = kwargs.get("obstacle_position_std_dev", 0)

        height = kwargs.get("height", 0.50)

        self.target = Target(np.array([0.60, 0.0, height]), np.eye(3))
        self.objects.append(self.target)

        self.blocks = []

        for i, position in enumerate(np.linspace([0.40, -0.50, height], [0.40, 0.50, height], num=6)):
            self.blocks.append(Block(position, np.eye(3), size=0.150 * np.ones(3)))
        self.objects.extend(self.blocks)

        self.first_update = True

    def get_new_scenario(self):
        self.first_update = True
        return self.target.get_target(make_visible=True), [], self.timeout

    def update(self):
        if self.first_update:
            self.frame_number = 0
            self.first_update = False
        else:
            self.frame_number += 1

        # Because robot perception can be slow, this slows down the world updates by a factor of self.frames_per_update.
        t = (self.frame_number // self.frames_per_update) * self.frames_per_update

        pos = (
            self.target.initial_base_trans
            + np.array([0, 0.50 * np.sin(self.target_speed / self.fps * t), 0])
            + np.random.normal(0, self.target_position_std_dev)
        )
        self.target.set_base_pose(translation=pos)

        for block, speed in zip(self.blocks, self.block_speeds):
            pos = (
                block.initial_base_trans
                + np.array([0, 0, 0.40 * np.sin(speed / self.fps * t)])
                + np.random.normal(0, self.obstacle_position_std_dev)
            )
            block.set_base_pose(translation=pos)


class WindmillEnv(Environment):
    def build_environment(self, **kwargs):
        """
        Kwargs:
            name: human-readable name of this environment (default: "Windmill")
            timeout: simulated time (in seconds) before a test times out (default: 300)

            target_translation_thresh: threshold for how close the robot end effector must be to
                translation targets in meters (default: .03)
            target_rotation_thresh:  threshold for how close the robot end effector must be to
                rotation targets in radians (default: .1)

            windmill_1_speed: rotational speed of windmill 1 in rad/sec (default: pi/15)
            windmill_2_speed: rotational speed of windmill 2 in rad/sec (default: pi/15)
            
            windmill_1_translation: translational position of windmill 1 (default [35,0,50])
            windmill_2_translation: translational position of windmill 1 (default [40,0,50])

            target_pos: position of target behind windmills (default [50,0,70])

            env_rotation: axis rotation of environmnet at the world origin (default [0,0,0])
        """

        self.name = kwargs.get("name", "Windmill")
        self.timeout = kwargs.get("timeout", 300)
        self.initial_robot_cspace_position = kwargs.get("initial_robot_cspace_position", None)

        self.target_translation_thresh = kwargs.get("target_translation_thresh", 0.03)
        self.target_rotation_thresh = kwargs.get("target_rotation_thresh", 0.1)

        self.speed1 = kwargs.get("windmill_1_speed", np.pi / 15)
        self.speed2 = kwargs.get("windmill_2_speed", np.pi / 25)

        self.bt1 = np.array(kwargs.get("windmill_1_translation", [0.40, 0, 0.50]))
        self.bt2 = np.array(kwargs.get("windmill_2_translation", [0.45, 0, 0.50]))

        self.br1 = np.eye(3)
        self.br2 = np.eye(3)

        self.t_pos = np.array(kwargs.get("target_pos", [0.50, 0, 0.70]))

        self.env_rotation = R.from_rotvec(
            np.array(kwargs.get("env_rotation", [0, 0, 0]))
        ).as_matrix()  # Rotate the entire environment about the robot.

        self.bt1 = self.env_rotation @ self.bt1
        self.bt2 = self.env_rotation @ self.bt2
        self.br1 = self.env_rotation @ self.br1
        self.br2 = self.env_rotation @ self.br2
        self.t_pos = self.env_rotation @ self.t_pos

        self.rot_axs = self.env_rotation @ np.array([1, 0, 0])

        self.w1 = Windmill(self.bt1, self.br1)
        self.w2 = Windmill(self.bt2, self.br2, num_blades=3)

        self.objects.append(self.w1)
        self.objects.append(self.w2)

        self.target = Target(self.t_pos, np.eye(3))
        self.start_target = Target(self.w1.initial_base_trans / 2, np.eye(3), target_color=np.array([0, 0, 1.0]))
        self.mid_target = Target(self.w1.initial_base_trans / 2, np.eye(3))

        self.frame_number = 0

    def update(self):
        self.frame_number += 1

        t = self.frame_number

        p1 = self.w1.initial_base_trans
        p2 = self.w2.initial_base_trans

        rot1 = R.from_rotvec(self.speed1 / self.fps * t * self.rot_axs).as_matrix() @ self.w1.initial_base_rotation
        rot2 = R.from_rotvec(self.speed2 / self.fps * t * self.rot_axs).as_matrix() @ self.w2.initial_base_rotation

        self.w1.set_base_pose(p1, rot1)
        self.w2.set_base_pose(p2, rot2)

    def get_new_scenario(self):
        sg = self.start_target.get_target(make_visible=True)
        mg = self.mid_target.get_target()
        tg = self.target.get_target()
        return sg, [tg, mg, tg], self.timeout


class WindowEnv(Environment):
    def build_environment(self, **kwargs):
        """
        Kwargs:
            name: human-readable name of this environment (default: "Window")
            timeout: simulated time (in seconds) before a test times out (default: 20)

            target_translation_thresh: threshold for how close the robot end effector must be to
                translation targets in meters (default: .03)
            target_rotation_thresh:  threshold for how close the robot end effector must be to
                rotation targets in radians (default: .1)
            
            window_translation: translational position of window (default [45,-30, 50])

            env_rotation: axis rotation of environmnet at the world origin (default [0,0,0])
        """

        self.name = kwargs.get("name", "Window")
        self.timeout = kwargs.get("timeout", 20)
        self.initial_robot_cspace_position = kwargs.get("initial_robot_cspace_position", None)

        self.target_translation_thresh = kwargs.get("target_translation_thresh", 0.03)
        self.target_rotation_thresh = kwargs.get("target_rotation_thresh", 0.1)

        self.window_trans = np.array(kwargs.get("window_translation", [0.45, -0.30, 0.50]))
        self.window_rotation = np.eye(3)

        env_rotation = R.from_rotvec(
            np.array(kwargs.get("env_rotation", [0, 0, 0]))
        ).as_matrix()  # Rotate the entire environment about the robot.

        self.window_trans = env_rotation @ self.window_trans
        self.window_rotation = env_rotation @ self.window_rotation

        self.window = Window(self.window_trans, self.window_rotation, window_width=0.30, window_height=0.30)
        self.start_target = Target(self.window_trans / 2, self.window_rotation, target_color=np.array([0, 0, 1.0]))
        self.objects.append(self.window)

        self.end_target = Target(np.array([*1.3 * self.window_trans[:2], self.window_trans[2]]), self.window_rotation)

    def get_new_scenario(self):
        return (
            self.start_target.get_target(),
            [
                self.window.front_target,
                self.window.center_target,
                self.window.behind_target,
                self.end_target.get_target(make_visible=False),
            ],
            self.timeout,
        )


class GuillotineEnv(Environment):
    def build_environment(self, **kwargs):
        """
        Kwargs:
            name: human-readable name of this environment (default: "Guillotine")
            timeout: simulated time (in seconds) before a test times out (default: 20)

            target_translation_thresh: threshold for how close the robot end effector must be to
                translation targets in meters (default: .03)
            target_rotation_thresh:  threshold for how close the robot end effector must be to
                rotation targets in radians (default: .1)
            
            wall_height: height of wall the robot has to reach through (default: 100)
            windmill_speed: speed of windmill embedded in wall in rad/sec (default: pi/15)
            window_translation: translational position of window embedded in wall (default [50,0, wall_height/2])
            windmill_translation: translational position of windmill (default: [55,0,wall_height/2+30])

            env_rotation: axis rotation of environmnet at the world origin (default [0,0,0])
        """

        self.name = kwargs.get("name", "Guillotine")
        self.timeout = kwargs.get("timeout", 20)
        self.initial_robot_cspace_position = kwargs.get("initial_robot_cspace_position", None)

        self.target_translation_thresh = kwargs.get("target_translation_thresh", 0.03)
        self.target_rotation_thresh = kwargs.get("target_rotation_thresh", 0.1)

        self.wall_height = kwargs.get("wall_height", 1.00)

        self.speed = kwargs.get("windmill_speed", np.pi / 15)

        self.window_trans = np.array(kwargs.get("window_translation", [0.50, 0, self.wall_height / 2]))
        end_target_trans = np.array([1.5 * self.window_trans[0], 0, self.wall_height / 2])

        self.window_rotation = np.eye(3)

        self.windmill_trans = np.array(kwargs.get("windmill_translation", [0.55, 0, self.wall_height / 2 + 0.30]))
        self.windmill_rot_axs = np.array([1, 0, 0])
        self.windmill_rotation = self.window_rotation

        self.env_rotation = R.from_rotvec(
            np.array(kwargs.get("env_rotation", [0, 0, 0]))
        ).as_matrix()  # Rotate the entire environment about the robot.

        self.window_trans = self.env_rotation @ self.window_trans
        self.window_rotation = self.env_rotation @ self.window_rotation

        self.windmill_trans = self.env_rotation @ self.windmill_trans
        self.windmill_rotation = self.env_rotation @ self.windmill_rotation
        self.windmill_rot_axs = self.env_rotation @ self.windmill_rot_axs

        self.window = Window(
            self.window_trans,
            self.window_rotation,
            height=self.wall_height,
            window_width=0.30,
            window_height=0.30,
            depth=0.20,
        )
        self.windmill = Windmill(self.windmill_trans, self.windmill_rotation, blade_width=0.10)
        self.start_target = Target(self.window_trans / 2, self.window_rotation, target_color=np.array([0, 0, 1.0]))
        self.objects.append(self.window)
        self.objects.append(self.windmill)

        self.end_target = Target(self.env_rotation @ end_target_trans, self.window_rotation)
        self.frame_number = 0

        self.camera_position = self.env_rotation @ np.array([2.00, -1.00, 1.00])
        self.camera_target = self.env_rotation @ np.array([0, 0, 0.50])

    def update(self):
        self.frame_number += 1
        t = self.frame_number

        rot = (
            R.from_rotvec(self.speed / self.fps * t * self.windmill_rot_axs).as_matrix()
            @ self.windmill.initial_base_rotation
        )
        self.windmill.set_base_pose(self.windmill.initial_base_trans, rot)

        end_tgt = self.end_target.initial_base_trans + self.env_rotation @ (
            10 * np.array([0, np.cos(t * self.speed / self.fps), np.sin(t * self.speed / self.fps)])
        )
        self.end_target.set_base_pose(end_tgt, self.window_rotation)

    def get_new_scenario(self):
        return (
            self.start_target.get_target(),
            [
                self.window.front_target,
                self.window.center_target,
                self.window.behind_target,
                self.end_target.get_target(make_visible=True),
            ],
            self.timeout,
        )


class CageEnv(Environment):
    def build_environment(self, **kwargs):
        """
        Kwargs:
            name: human-readable name of this environment (default: "Cage")
            timeout: simulated time (in seconds) before a test times out (default: 20)

            target_translation_thresh: threshold for how close the robot end effector must be to
                translation targets in meters (default: .03)
            target_rotation_thresh:  threshold for how close the robot end effector must be to
                rotation targets in radians (default: .1)

            ceiling_height: height of ceiling of cage (default .75 m)
            ceiling_thickness: thickness (along z axis) of ceiling (default .01 m)
            cage_width: width (along x axis) of cage (default .35 m)
            cage_length: length (along y axis) of cage (default .35 m)

            num_pillars: number of pillars defining the "bars" of the cage.  The pillars will be evenly 
                spaced by angle around the elipse defined by cage_width and cage_length
            pillar_thickness: thickness of pillars (default .1 m)

            target_scalar: a scalar defining the distance from the robot to the targets.  Potential targets
                are arranged in an ovoid around the robot with a width of target_scalar*cage_width and a length
                of target_scalar*cage_length.  A value of 1 would place some targets inside the pillars (default 1.15)
        """

        self.name = kwargs.get("name", "Cage")
        self.timeout = kwargs.get("timeout", 20)
        self.initial_robot_cspace_position = kwargs.get("initial_robot_cspace_position", None)

        self.target_translation_thresh = kwargs.get("target_translation_thresh", 0.03)
        self.target_rotation_thresh = kwargs.get("target_rotation_thresh", 0.1)

        self.ceiling_height = kwargs.get("ceiling_height", 0.75)
        self.ceiling_thickenss = kwargs.get("ceiling_thickness", 0.01)

        self.cage_width = kwargs.get("cage_width", 0.35)
        self.cage_length = kwargs.get("cage_length", 0.35)

        self.num_pillars = kwargs.get("num_pillars", 4)
        self.pillar_thickness = kwargs.get("pillar_thickness", 0.1)

        self.target_scalar = kwargs.get("target_scalar", 1.15)

        self.objects = []

        self.cage = Cage(
            np.zeros(3),
            np.eye(3),
            ceiling_height=self.ceiling_height,
            ceiling_thickness=self.ceiling_thickenss,
            cage_width=self.cage_width,
            cage_length=self.cage_length,
            num_pillars=self.num_pillars,
            pillar_thickness=self.pillar_thickness,
            target_scalar=self.target_scalar,
        )
        self.objects.append(self.cage)
        self.start_target = Target(
            np.array([(self.cage_width - self.pillar_thickness / 2) / 1.25, 0, self.ceiling_height / 2]),
            np.eye(3),
            target_color=np.array([0, 0, 1.0]),
        )

    def get_new_scenario(self):
        return (
            self.start_target.get_target(),
            [
                self.cage.get_random_target(False),
                self.cage.get_random_target(False),
                self.cage.get_random_target(False),
            ],
            self.timeout,
        )


class EmptyEnv(Environment):
    def build_environment(self, **kwargs):
        self.name = "Empty"

    def get_new_scenario(self):
        return None, [], self.timeout
