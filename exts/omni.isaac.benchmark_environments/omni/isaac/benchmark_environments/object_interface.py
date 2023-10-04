# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.client import delete
from pxr import UsdGeom, Gf, UsdPhysics
import numpy as np
import asyncio
import uuid
import omni.kit.app
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.rotations import matrix_to_euler_angles, euler_angles_to_quat
from omni.isaac.core.objects import cuboid, sphere, capsule
from omni.isaac.core.prims import XFormPrim, GeometryPrimView


def matrix_to_quat(rot_mat):
    return euler_angles_to_quat(matrix_to_euler_angles(rot_mat))


class Object:
    """
    An Object is a collection of prims that move and change their properties together.  A base Object has no prims
    in it by default because the construct method is empty.  The Object Class is inherited by specific types of 
    objects in objects.py and the construct function is implemented

    Objects have a base translation and rotation.  All prims that form the object have a 
        (fixed) translation and rotation that is relative to the base.  The base pose can 
        be updated with set_base_pose, and the world coordinate positions of every prim will
        be updated by their relative position to the base.

    Objects have two lists of prims: self.components, and self.targets
        self.components: prims that for the object that the robot must avoid
        self.targets: possible targets to make a robot seek. Ex. the cubby object has possible targets
            in each cubby that are transformed along with the base. A random target can be retrieved with
            self.get_random_target() 

    Objects are made up of three primitives: capsules, spheres, and rectangular prisms
    TODO: allow meshes to be loaded for an object
    """

    def __init__(self, base_translation, base_rotation, color=np.array([1.0, 1.0, 0]), **kwargs):

        self.initial_base_trans = base_translation
        self.initial_base_rotation = base_rotation
        self.components = []
        self.color = color

        self.targets = []
        self.last_target = None

        # make a random USD path for all the prims in this object
        object_id = str(uuid.uuid4())
        self.base_path = ("/scene/object-" + object_id).replace("-", "_")
        self.base_xform = XFormPrim(self.base_path)
        self.base_xform.set_world_pose(base_translation, matrix_to_quat(base_rotation))

        self.construct(**kwargs)
        self._geometry_view = GeometryPrimView(self.base_path + "/.*")
        self._geometry_view.apply_collision_apis()

        self.set_enable_collisions(False)

    def construct(self, **kwargs):
        pass

    def get_random_target(self, make_visible=True):
        if self.last_target is not None:
            self.last_target.set_visibility(False)

        if len(self.targets) > 0:
            target = np.random.choice(self.targets)
            target.set_visibility(make_visible)
            self.last_target = target
            return target

    def get_all_components(self):
        # get all prims that the robot should avoid
        return self.components

    def create_target(
        self,
        relative_translation=np.zeros(3),
        relative_rotation=np.eye(3),
        target_color=np.array([1.0, 0, 0]),
        target_size=0.05,
    ):

        path = self.base_path + "/target_" + str(len(self.targets))
        target = cuboid.VisualCuboid(path, size=target_size, color=target_color)

        target.set_local_pose(relative_translation, matrix_to_quat(relative_rotation))

        target.set_visibility(False)

        self.targets.append(target)

        return target

    def create_block(self, size, relative_translation=np.zeros(3), relative_rotation=np.eye(3)):
        path = self.base_path + "/cuboid_" + str(len(self.components))
        cube = cuboid.FixedCuboid(path, size=1.0, scale=size, color=self.color)
        cube.set_local_pose(relative_translation, matrix_to_quat(relative_rotation))

        self.components.append(cube)

        return cube

    def create_sphere(self, radius, relative_translation=np.zeros(3), relative_rotation=np.eye(3)):
        path = self.base_path + "/sphere_" + str(len(self.components))
        sphere = sphere.FixedSphere(path, radius=radius, color=self.color)
        sphere.set_local_pose(relative_translation, matrix_to_quat(relative_rotation))

        self.components.append(sphere)

        return sphere

    def create_capsule(self, radius, height, relative_translation=np.zeros(3), relative_rotation=np.eye(3)):
        path = self.base_path + "/capsule_" + str(len(self.components))
        capsule = capsule.FixedCapsule(path, radius=radius, height=height, color=self.color)
        capsule.set_local_pose(relative_translation, matrix_to_quat(relative_rotation))

        self.components.append(capsule)

        return capsule

    def set_base_pose(self, translation=np.zeros(3), rotation=np.eye(3)):

        self.base_xform.set_world_pose(translation, matrix_to_quat(rotation))

    def set_visibility(self, on=True):
        for component in self.components:
            component.set_visibility(on)

    def set_enable_collisions(self, collisions_enabled=True):
        if collisions_enabled:
            self._geometry_view.enable_collision()
        else:
            self._geometry_view.disable_collision()

    def delete(self):
        for component in self.components:
            delete_prim(component.prim_path)
        self.components = []

        for target in self.targets:
            delete_prim(target.prim_path)

        self.targets = []

    def reset(self):
        self.set_base_pose(self.initial_base_trans, self.initial_base_rotation)
        for target in self.targets:
            target.set_visibility(False)
