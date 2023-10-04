# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse
from omni.isaac.kit import SimulationApp
import numpy as np
import omni

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}


# Set up command line arguments
parser = argparse.ArgumentParser("Usd Load sample")
parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")

args, unknown = parser.parse_known_args()
# Start the omniverse application
CONFIG["headless"] = args.headless
simulation_app = SimulationApp(launch_config=CONFIG)

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# open stage
omni.usd.get_context().open_stage("simple_case.usd")

# wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

world = World(stage_units_in_meters=1.0)
robot = world.scene.add(Robot(prim_path="/World/panda", name="robot"))

world.reset()

while simulation_app.is_running():
    world.step(render=not args.headless)

    # deal with pause/stop
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()

        # apply actions
        robot.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=np.random.random(9))
        )

simulation_app.close()
