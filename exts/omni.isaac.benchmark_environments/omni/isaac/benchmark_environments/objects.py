# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from pxr import Gf
import numpy as np
from scipy.spatial.transform import Rotation as R

from .object_interface import Object

"""
An Object allows the manipulation of a group of prims as a single unit.  A subclass of Object must implement
the construct() method to specify what prims comprise an object, and what their relative positions are.

When an Object is created, the base pose is specified in the constructor.  Inside the construct() method,
the position of each prim in the object is specified relative to that base pose.

For example, the Cubby Object is constructed from a set of blocks that have translations specified relative to 
the base of the Cubby.  The relative rotation is not specified, and so is assumed to be the identity.

Target, Block, Sphere, and Capsule are the most basic possible variants of Object, as they are comprised of 
just a single prim.
"""


class Target(Object):
    def construct(self, **kwargs):
        size = kwargs.get("size", 0.05)
        target_color = kwargs.get("target_color", np.array([1.0, 0, 0]))

        self.create_target(target_size=size, target_color=target_color)

    def get_target(self, make_visible=True):
        target = self.targets[0]
        target.set_visibility(make_visible)

        return target


class Block(Object):
    def construct(self, **kwargs):
        self.size = kwargs.get("size", 0.10 * np.ones(3))
        # self.scales = kwargs.get("scales", np.array([1.0, 1.0, 1.0]))

        self.create_block(self.size)

    def get_component(self):
        return self.components[0]


class Sphere(Object):
    def construct(self, **kwargs):
        self.radius = kwargs.get("radius", 0.10)
        self.create_sphere(self.radius)

    def get_geom(self):
        return self.components[0]


class Capsule(Object):
    def construct(self, **kwargs):
        self.radius = kwargs.get("radius", 0.05)
        self.height = kwargs.get("height", 0.10)
        self.create_capsule(self.radius, self.height)

    def get_geom(self):
        return self.components[0]


class Cubbies(Object):
    def construct(self, **kwargs):
        self.size = kwargs.get("size", 1)

        self.num_rows = kwargs.get("num_rows", 3)
        self.num_cols = kwargs.get("num_cols", 3)

        self.height = kwargs.get("height", 1.00)  # z axis
        self.width = kwargs.get("width", 1.00)  # y axis
        self.depth = kwargs.get("depth", 0.30)  # x axis

        self.target_depth = kwargs.get("target_depth", self.depth / 2)

        self.cub_height = self.height / self.num_rows
        self.cub_width = self.width / self.num_cols

        back = self.create_block(
            self.size * np.array([0.01, self.width, self.height]),
            relative_translation=np.array([self.depth / 2, 0, self.height / 2]),
        )

        for i in range(self.num_rows + 1):
            shelf = self.create_block(
                self.size * np.array([self.depth, self.width, 0.01]),
                relative_translation=np.array([0, 0, self.cub_height * i]),
            )
        for i in range(self.num_cols + 1):
            shelf = self.create_block(
                self.size * np.array([self.depth, 0.01, self.height]),
                relative_translation=np.array([0, self.cub_width * i - self.width / 2, self.height / 2]),
            )

        # Put a target in each shelf.

        target_x_offset = self.target_depth - self.depth / 2

        target_start = np.array([target_x_offset, -self.width / 2 + self.cub_width / 2, self.cub_height / 2])

        target_rot = R.from_rotvec([0, np.pi / 2, 0]).as_matrix()

        for i in range(self.num_rows):
            for j in range(self.num_cols):
                pos = target_start + np.array([0, self.cub_width * j, self.cub_height * i])
                target = self.create_target(relative_translation=pos, relative_rotation=target_rot)


class Cage(Object):
    def construct(self, **kwargs):
        self.ceiling_height = kwargs.get("ceiling_height", 0.75)
        self.ceiling_thickenss = kwargs.get("ceiling_thickness", 0.01)

        self.cage_width = kwargs.get("cage_width", 0.3)
        self.cage_length = kwargs.get("cage_length", 0.3)

        self.num_pillars = kwargs.get("num_pillars", 3)
        self.pillar_thickness = kwargs.get("pillar_thickness", 0.1)

        self.target_scalar = kwargs.get("target_scalar", 1.15)

        ceiling = self.create_block(
            np.array([2 * self.cage_width, 2 * self.cage_length, self.ceiling_thickenss]),
            np.array([0, 0, self.ceiling_height]),
            np.eye(3),
        )

        for i in range(self.num_pillars):
            angle = 2 * (i + 0.5) * np.pi / self.num_pillars

            pillar_x = self.cage_width * np.cos(angle)
            pillar_y = self.cage_length * np.sin(angle)

            pillar = self.create_block(
                np.array([self.pillar_thickness, self.pillar_thickness, self.ceiling_height]),
                np.array([pillar_x, pillar_y, self.ceiling_height / 2]),
                np.eye(3),
            )

        for angle in np.arange(0, 2 * np.pi, 0.1):
            target_x = (self.cage_width + self.pillar_thickness) * self.target_scalar * np.cos(angle)
            target_y = (self.cage_length + self.pillar_thickness) * self.target_scalar * np.sin(angle)
            self.create_target(np.array([target_x, target_y, self.ceiling_height / 2]))


class Windmill(Object):
    def construct(self, **kwargs):
        self.size = kwargs.get("size", 1)  # scales entire windmill

        self.num_blades = kwargs.get("num_blades", 2)

        self.blade_width = kwargs.get("blade_width", 0.01)  # y axis
        self.blade_height = kwargs.get("blade_height", 1.00)  # z axis
        self.blade_depth = kwargs.get("blade_depth", 0.01)  # x axis

        for i in range(self.num_blades):
            rot = R.from_rotvec([i * np.pi / self.num_blades, 0, 0])
            blade = self.create_block(
                self.size * np.array([self.blade_depth, self.blade_width, self.blade_height]),
                relative_rotation=rot.as_matrix(),
            )


class Window(Object):
    def construct(self, **kwargs):
        self.width = kwargs.get("width", 1.00)  # y axis
        self.depth = kwargs.get("depth", 0.05)  # x axis
        self.height = kwargs.get("height", 1.00)  # z axis

        self.size = kwargs.get("size", 1)  # scales entire window

        self.window_width = kwargs.get("window_width", 0.50)
        self.window_height = kwargs.get("window_height", 0.50)

        side_width = (self.width - self.window_width) / 2
        side = self.create_block(
            self.size * np.array([self.depth, side_width, self.height]),
            relative_translation=np.array([0, -self.width / 2 + side_width / 2, 0]),
        )
        side = self.create_block(
            self.size * np.array([self.depth, side_width, self.height]),
            relative_translation=np.array([0, +self.width / 2 - side_width / 2, 0]),
        )

        top_height = (self.height - self.window_height) / 2
        top = self.create_block(
            self.size * np.array([self.depth, self.width, top_height]),
            relative_translation=np.array([0, 0, self.height / 2 - top_height / 2]),
        )
        bottom = self.create_block(
            self.size * np.array([self.depth, self.width, top_height]),
            relative_translation=np.array([0, 0, -self.height / 2 + top_height / 2]),
        )

        self.center_target = self.create_target()  # in center of window
        self.behind_target = self.create_target(relative_translation=np.array([self.depth, 0, 0]))
        self.front_target = self.create_target(relative_translation=np.array([-self.depth, 0, 0]))
