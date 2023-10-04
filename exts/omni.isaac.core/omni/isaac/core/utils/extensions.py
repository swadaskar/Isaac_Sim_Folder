# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.app


def get_extension_id(extension_name: str) -> int:
    """Get extension id for a loaded extension

    Args:
        extension_name (str): name of the extension

    Returns:
        int: Full extension id
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_enabled_extension_id(extension_name)


def get_extension_path(ext_id: int) -> str:
    """Get extension path for a loaded extension

    Args:
        ext_id (id): full id of extension

    Returns:
        str: Path to loaded extension root directory
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(ext_id)


def get_extension_path_from_name(extension_name: str) -> str:
    """Get extension path for a loaded extension

    Args:
        extension_name (str): name of the extension

    Returns:
        str: Path to loaded extension root directory
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(get_extension_id(extension_name))


def enable_extension(extension_name: str) -> bool:
    """Load an extension from the extenstion manager.

    Args:
        extension_name (str): name of the extension

    Returns:
        bool: True if extension could be loaded, False otherwise
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(extension_name, True)


def disable_extension(extension_name: str) -> bool:
    """Unload an extension.

    Args:
        extension_name (str): name of the extension

    Returns:
        bool: True if extension could be unloaded, False otherwise
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(extension_name, False)
