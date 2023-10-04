# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
import sys
import carb
from omni.isaac.kit import SimulationApp

HOSPITAL_USD_PATH = "/Isaac/Samples/ROS/Scenario/multiple_robot_carter_hospital_navigation.usd"
OFFICE_USD_PATH = "/Isaac/Samples/ROS/Scenario/multiple_robot_carter_office_navigation.usd"

# Default environment: Hospital
ENV_USD_PATH = HOSPITAL_USD_PATH

if len(sys.argv) > 1:

    if sys.argv[1] == "office":
        # Choosing Office environment
        ENV_USD_PATH = OFFICE_USD_PATH

    elif sys.argv[1] != "hospital":
        carb.log_warn("Environment name is invalid. Choosing default Hospital environment.")
else:
    carb.log_warn("Environment name not specified. Choosing default Hospital environment.")


CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS bridge sample demonstrating the manual loading of Multiple Robot Navigation scenario
simulation_app = SimulationApp(CONFIG)
import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from pxr import Sdf

from omni.isaac.core.utils.extensions import enable_extension

# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

# Locate assets root folder to load sample
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

usd_path = assets_root_path + ENV_USD_PATH
omni.usd.get_context().open_stage(usd_path, None)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

simulation_app.update()

simulation_context.play()

simulation_app.update()

while simulation_app.is_running():

    # runs with a realtime clock
    simulation_app.update()

simulation_context.stop()
simulation_app.close()
