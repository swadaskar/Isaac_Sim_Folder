# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import platform

# this file own the settings Strings so they can easily be shared and access from the other part of the App Selector
SHOW_CONSOLE_SETTING = "/persistent/ext/omni.isaac.selector/show_console"
PERSISTENT_SELECTOR_SETTING = "/persistent/ext/omni.isaac.selector/persistent_selector"

APPS_SETTING = "/ext/omni.isaac.selector/apps"
EXPERIMENTAL_APPS_SETTING = "/ext/omni.isaac.selector/experimental_apps"
AUTO_START_SETTING = "/persistent/ext/omni.isaac.selector/auto_start"

DEFAULT_APP_SETTING = "/persistent/ext/omni.isaac.selector/default_app"
EXTRA_ARGS_SETTING = "/persistent/ext/omni.isaac.selector/extra_args"

PERSISTENT_ROS_BRIDGE_SETTING = "/persistent/ext/omni.isaac.selector/ros_bridge_extension"
if platform.system().lower() == "windows":
    ROS_BRIDGE_EXTENSIONS = ["", "omni.isaac.ros2_bridge"]
else:
    ROS_BRIDGE_EXTENSIONS = ["omni.isaac.ros_bridge", "omni.isaac.ros2_bridge", "omni.isaac.ros2_bridge-humble", ""]
