# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import carb

SETTINGS_PATH = "/persistent/exts/omni.isaac.onshape.settings"

if carb.settings.get_settings().get("{}/filter_unsupported".format(SETTINGS_PATH)) is None:
    carb.settings.get_settings().set("{}/filter_unsupported".format(SETTINGS_PATH), False)
if carb.settings.get_settings().get("{}/import_physics".format(SETTINGS_PATH)) is None:
    carb.settings.get_settings().set("{}/import_physics".format(SETTINGS_PATH), True)


def get_import_physics():
    if carb.settings.get_settings().get("{}/import_physics".format(SETTINGS_PATH)) is None:
        carb.settings.get_settings().set("{}/import_physics".format(SETTINGS_PATH), True)
    return carb.settings.get_settings().get("{}/import_physics".format(SETTINGS_PATH))


from .scripts.extension import OnshapeImporter
