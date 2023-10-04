# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import sys
import os.path
import typing
import carb.settings
import carb.tokens


class Version:
    def __init__(self):
        self.core = ""
        self.prerelease = ""
        self.major = ""
        self.minor = ""
        self.patch = ""
        self.pretag = ""
        self.prebuild = ""
        self.buildtag = ""


def parse_version(full_version: Version):
    parsed_version = Version()
    if "+" in full_version:
        full_version, parsed_version.buildtag = full_version.split("+")
    if "-" in full_version:
        parsed_version.core, parsed_version.prerelease = full_version.split("-", maxsplit=1)
        parsed_version.major, parsed_version.minor, parsed_version.patch = parsed_version.core.split(".", maxsplit=2)
        parsed_version.pretag, parsed_version.prebuild = parsed_version.prerelease.split(".", maxsplit=1)
    else:
        parsed_version.major, parsed_version.minor, parsed_version.patch = full_version.split(".", maxsplit=2)
        parsed_version.core = full_version
    return parsed_version


def get_version() -> typing.Tuple[str, str, str, str, str, str, str, str]:
    """Retrieve version from file

    Returns:
        Core version (str)
        Pre-release tag and build number (str)
        Major version (str)
        Minor version (str)
        Patch version (str)
        Pre-release tag (str)
        Build number (str)
        Build tag (str)
    """
    app_folder = carb.settings.get_settings().get_as_string("/app/folder")
    if not app_folder:
        app_folder = carb.tokens.get_tokens_interface().resolve("${app}")
    app_start_folder = os.path.normpath(os.path.join(app_folder, os.pardir))
    app_version = open(f"{app_start_folder}/VERSION").readline().strip()
    parsed_version = parse_version(app_version)
    return (
        parsed_version.core,
        parsed_version.prerelease,
        parsed_version.major,
        parsed_version.minor,
        parsed_version.patch,
        parsed_version.pretag,
        parsed_version.prebuild,
        parsed_version.buildtag,
    )
