# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
from typing import Callable, Union

# omniverse
from pxr import Sdf
import omni.kit
import omni.kit.commands

# isaacsim
from omni.isaac.core.utils.stage import get_current_stage


def get_rigid_body_enabled(prim_path: str) -> Union[bool, None]:
    """Get the physics:rigidBodyEnabled attribute from the USD Prim at the given path

    Args:
        prim_path (str): The path to the USD Prim

    Returns:
        Any: The value of physics:rigidBodyEnabled attribute if it exists, and None if it does not exist.
    """
    stage = get_current_stage()
    return stage.GetPrimAtPath(prim_path).GetAttribute("physics:rigidBodyEnabled").Get()


def set_rigid_body_enabled(_value, prim_path):
    """If it exists, set the physics:rigidBodyEnabled attribute on the USD Prim at the given path

    Args:
        _value (Any): Value to set physics:rigidBodyEnabled attribute to
        prim_path (str): The path to the USD Prim
    """
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(f"{prim_path}.physics:rigidBodyEnabled"), value=_value, prev=None
    )


async def simulate_async(seconds: float, steps_per_sec: int = 60, callback: Callable = None) -> None:
    """Helper function to simulate async for seconds * steps_per_sec frames.

    Args:
        seconds (float): time in seconds to simulate for
        steps_per_sec (int, optional): steps per second. Defaults to 60.
        callback (Callable, optional): optional function to run every step. Defaults to None.
    """
    for _ in range(int(steps_per_sec * seconds)):
        await omni.kit.app.get_app().next_update_async()
        if callback is not None:
            callback()
