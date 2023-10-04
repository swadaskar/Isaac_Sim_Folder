# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import Usd
import omni
import typing
import carb
import json
import omni.client
from omni.client._omniclient import Result


def build_server_list() -> typing.List:
    """Return list with all known servers to check

    Returns:
        all_servers (typing.List): List of servers found
    """
    mounted_drives = carb.settings.get_settings().get_settings_dictionary("/persistent/app/omniverse/mountedDrives")
    all_servers = []
    if mounted_drives is not None:
        mounted_dict = json.loads(mounted_drives.get_dict())
        for drive in mounted_dict.items():
            all_servers.append(drive[1])
    else:
        carb.log_warn("/persistent/app/omniverse/mountedDrives setting not found")

    return all_servers


def check_server(server: str, path: str) -> bool:
    """Check a specific server for a path

    Args:
        server (str): Name of Nucleus server
        path (str): Path to search

    Returns:
        bool: True if folder is found
    """
    carb.log_info("Checking path: {}{}".format(server, path))
    # Increase hang detection timeout
    omni.client.set_hang_detection_time_ms(10000)
    result, _ = omni.client.stat("{}{}".format(server, path))
    if result == Result.OK:
        carb.log_info("Success: {}{}".format(server, path))
        return True
    else:
        carb.log_info("Failure: {}{} not accessible".format(server, path))
        return False


def get_assets_root_path() -> typing.Union[str, None]:
    """Tries to find the root path to the Isaac Sim assets on a Nucleus server

    Returns:
        url (str): URL of Nucleus server with root path to assets folder.
        Returns None if Nucleus server not found.
    """

    # 1 - Check /persistent/isaac/asset_root/default setting
    carb.log_info("Check /persistent/isaac/asset_root/default setting")
    default_asset_root = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
    if default_asset_root:
        result = check_server(default_asset_root, "/Isaac")
        if result:
            result = check_server(default_asset_root, "/NVIDIA")
            if result:
                carb.log_info("Assets root found at {}".format(default_asset_root))
                return default_asset_root

    # 2 - Check root on mountedDrives setting
    connected_servers = build_server_list()
    if len(connected_servers):
        for server_name in connected_servers:
            # carb.log_info("Found {}".format(server_name))
            result = check_server(server_name, "/Isaac")
            if result:
                result = check_server(server_name, "/NVIDIA")
                if result:
                    carb.log_info("Assets root found at {}".format(server_name))
                    return server_name

    # 3 - Check cloud for /Assets/Isaac/{version_major}.{version_minor} folder
    cloud_assets_url = carb.settings.get_settings().get("/persistent/isaac/asset_root/cloud")
    carb.log_info("Checking {}...".format(cloud_assets_url))
    if cloud_assets_url:
        result = check_server(cloud_assets_url, "/Isaac")
        if result:
            result = check_server(cloud_assets_url, "/NVIDIA")
            if result:
                carb.log_info("Assets root found at {}".format(cloud_assets_url))
                return cloud_assets_url

    carb.log_warn("Could not find assets root folder")
    return None


async def open_stage_async(usd_path: str) -> bool:
    """
    Open the given usd file and replace currently opened stage
    Args:
        usd_path (str): Path to open
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be loaded with this method")
    usd_context = omni.usd.get_context()
    usd_context.disable_save_to_recent_files()
    (result, error) = await omni.usd.get_context().open_stage_async(usd_path)
    usd_context.enable_save_to_recent_files()
    return (result, error)
