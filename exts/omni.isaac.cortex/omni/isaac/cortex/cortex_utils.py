# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

""" Cortex-specific utilities and helper methods
"""

import numpy as np
from pxr import Usd
from types import ModuleType
from typing import Union

from omni.isaac.core.utils.nucleus import get_assets_root_path


def load_behavior_module(behavior_filepath: str, module_name: str = "behavior") -> ModuleType:
    """ Load and return a behavior module.

    A behavior module is a python module with a make_decider_network() method. If that method is not
    found, a RuntimeError is raised.

    Args:
        behavior_filepath: The path to the module file to be loaded.
        module_name: An optional name of the module to load. Defaults to "behavior".

    Returns:
        The loaded module. One can call module.make_decider_network(robot) to create and return the
        decider network representing the behavior.

    Raises:
        RuntimeError if the module doesn't have a make_decider_network() method.
    """
    from importlib.machinery import SourceFileLoader

    module = SourceFileLoader(module_name, behavior_filepath).load_module()

    if not hasattr(module, "make_decider_network"):
        raise RuntimeError("Module at path {} does not have a make_decider_network() method.".format(behavior_filepath))

    return module


def get_assets_root_path_or_die() -> str:
    """ Find the assets root path and check for errors.

    Returns: The root path.

    Raises: RuntimeError if the assets folder could not be found.
    """
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        err_str = "Could not find Isaac Sim assets folder"
        carb.log_error(err_str)
        raise RuntimeError(err_str)
    return assets_root_path
