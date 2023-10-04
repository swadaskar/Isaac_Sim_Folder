# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

from pxr import Plug

pluginsRoot = os.path.join(os.path.dirname(__file__), "../../../plugins")

Plug.Registry().RegisterPlugins(pluginsRoot + "/RangeSensorSchema/resources")
Plug.Registry().RegisterPlugins(pluginsRoot + "/IsaacSensorSchema/resources")
Plug.Registry().RegisterPlugins(pluginsRoot + "/RobotEngineBridgeSchema/resources")
Plug.Registry().RegisterPlugins(pluginsRoot + "/RosBridgeSchema/resources")

ISAAC_NAME_OVERRIDE = "isaac:nameOverride"
