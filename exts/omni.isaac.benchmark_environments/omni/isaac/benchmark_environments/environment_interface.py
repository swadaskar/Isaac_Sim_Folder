# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.usd
from pxr import UsdPhysics, PhysxSchema

"""
Environments have a set of Objects in them that can move or change over time.
The default environment doesn't have any objects in it.

Subclasses of Environment implement 
    build_environment: instantiate all objects/targets in the environment according to **kwargs
    update: called every frame, update moves all moving elements of the environment and does any necessary maintainence
    get_new_scenario: return a series of targets that the robot should follow

Environments have random seeds to ensure repeatability.  This means that all randomness in the environment should
    be achieved through numpy's random module.

As a convention, environments are displaced from the robot along the positive x axis, and mostly centered about the y axis.
    Some robots (such as the UR10) may need the environment to be rotated about the robot.  The UR10's environment config file
    rotates each environment by pi/2 about the z axis.
"""


class Environment:
    def __init__(self, random_seed=0, **kwargs):
        self._stage = omni.usd.get_context().get_stage()
        self.objects = []
        self.targetable_objects = []
        self.random_seed = random_seed
        np.random.seed(random_seed)

        # get the frequency in Hz of the simulation
        physxSceneAPI = None
        for prim in self._stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(prim)
        if physxSceneAPI is not None:
            self.fps = physxSceneAPI.GetTimeStepsPerSecondAttr().Get()
        else:
            self.fps = 60

        self.timeout = 10  # simulated seconds before a test times out

        self.camera_position = [-2.00, -1.00, 1.00]
        self.camera_target = [0.50, 0, 0.50]

        # Set thresholds for robot reaching translation/rotation targets
        self.target_translation_thresh = 0.03
        self.target_rotation_thresh = 0.1

        self.name = ""

        self.initial_robot_cspace_position = None

        self.build_environment(**kwargs)

    def get_timeout(self):
        return self.timeout

    def build_environment(self, **kwargs):
        pass

    def update(self):
        # update positions of moving obstacles/targets in environment as a function of time
        pass

    def get_new_scenario(self):
        """
        Returns
            start_config (USD Geom): desired starting position of the robot (should be easy to reach to start the test)
                if None, the robot should ignore this scenario
            waypoints (USD Geom): desired goal position of the robot
                if None, the robot will follow the start_target until timeout is reached
            timeout (float): stop trial when timeout (seconds) have passes
        """

        return None, [], self.timeout

    def enable_collisions(self):
        for obj in self.objects:
            obj.set_enable_collisions(True)

    def disable_collisions(self):
        for obj in self.objects:
            obj.set_enable_collisions(False)

    def get_all_obstacles(self):
        prims = []
        for obj in self.objects:
            prims.extend(obj.get_all_components())
        return prims

    def get_random_target(self, make_visible=True):
        if len(self.targetable_objects) == 0:
            return None
        obj = np.random.choice(self.targetable_objects)
        return obj.get_random_target(make_visible=make_visible)

    def reset(self, new_seed=None):
        if new_seed is not None:
            self.random_seed = new_seed
        self.first_update = True
        np.random.seed(self.random_seed)
        for obj in self.objects:
            obj.reset()

    def delete_all_objects(self):
        for obj in self.targetable_objects:
            obj.delete()

        for obj in self.objects:
            obj.delete()

    def get_initial_robot_cspace_position(self):
        return self.initial_robot_cspace_position

    def get_target_thresholds(self):
        return self.target_translation_thresh, self.target_rotation_thresh

    def set_target_translation_threshold(self, thresh):
        self.target_translation_thresh = thresh

    def set_target_rotation_threshold(self, thresh):
        self.target_rotation_thresh = thresh
