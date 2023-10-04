# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.commands
import omni.kit.utils
import carb
from pxr import Sdf
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.utils._isaac_utils import transforms
import asyncio


class IsaacSimSpawnPrim(omni.kit.commands.Command):
    """Command to spawn a new prim in the stage and set its transform. This uses dynamic_control to properly handle physics objects and articulation

        Typical usage example:

        .. code-block:: python

            omni.kit.commands.execute(
                "IsaacSimSpawnPrim",
                usd_path="/path/to/file.usd",
                prim_path="/World/Prim,
                translation=(0, 0, 0),
                rotation=(0, 0, 0, 1),
            )
    """

    def __init__(
        self, usd_path: str, prim_path: str, translation: carb.Float3 = (0, 0, 0), rotation: carb.Float4 = (0, 0, 0, 1)
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        pass
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._stage = omni.usd.get_context().get_stage()
        self._context = omni.usd.get_context()
        pass

    def do(self) -> bool:
        self._prim = self._stage.DefinePrim(self._prim_path, "Xform")
        self._prim.GetReferences().AddReference(self._usd_path)
        if self._translation is not None and self._rotation is not None:
            transforms.set_transform(
                self._dc,
                self._context.get_stage_id(),
                str(self._prim.GetPath()),
                tuple(self._translation),
                tuple(self._rotation),
            )

        return True
        pass

    def undo(self):
        pass


class IsaacSimTeleportPrim(omni.kit.commands.Command):
    """Command to set a transform of a prim. This uses dynamic_control to properly handle physics objects and articulation

        Typical usage example:

        .. code-block:: python

            omni.kit.commands.execute(
                "IsaacSimTeleportPrim",
                prim_path="/World/Prim,
                translation=(0, 0, 0),
                rotation=(0, 0, 0, 1),
            )
    """

    def __init__(self, prim_path: str, translation: carb.Float3 = (0, 0, 0), rotation: carb.Float4 = (0, 0, 0, 1)):
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._context = omni.usd.get_context()
        pass

    def do(self) -> bool:
        if self._translation is not None and self._rotation is not None:
            transforms.set_transform(
                self._dc, self._context.get_stage_id(), str(self._prim_path), self._translation, self._rotation
            )
        return True
        pass

    def undo(self):
        pass


class IsaacSimScalePrim(omni.kit.commands.Command):
    """Command to set a scale of a prim

        Typical usage example:

        .. code-block:: python

            omni.kit.commands.execute(
                "IsaacSimScalePrim",
                prim_path="/World/Prim,
                scale=(1.5, 1.5, 1.5),
            )
    """

    def __init__(self, prim_path: str, scale: carb.Float3 = (0, 0, 0)):
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._context = omni.usd.get_context()
        pass

    def do(self) -> bool:
        if self._scale is not None:
            transforms.set_scale(self._dc, self._context.get_stage_id(), str(self._prim_path), self._scale)
        return True
        pass

    def undo(self):
        pass


class IsaacSimDestroyPrim(omni.kit.commands.Command):
    """Command to set a delete a prim. This variant has less overhead than other commands as it doesn't store an undo operation

        Typical usage example:

        .. code-block:: python

            omni.kit.commands.execute(
                "IsaacSimDestroyPrim",
                prim_path="/World/Prim,
            )
    """

    def __init__(self, prim_path: str):
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        pass

    def do(self) -> bool:
        delete_cmd = omni.usd.commands.DeletePrimsCommand([self._prim_path])
        delete_cmd.do()
        pass

    def undo(self):
        pass


omni.kit.commands.register_all_commands_in_module(__name__)
