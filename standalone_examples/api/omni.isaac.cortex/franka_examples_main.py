# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.kit import SimulationApp

import argparse

parser = argparse.ArgumentParser("franka_examples")
parser.add_argument(
    "--behavior",
    type=str,
    default="behaviors/franka/block_stacking_behavior.py",
    help="Which behavior to run. See behavior/franka for available behavior files.",
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import numpy as np

from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.cortex.cortex_world import CortexWorld, LogicalStateMonitor, Behavior
from omni.isaac.cortex.robot import add_franka_to_stage
from omni.isaac.cortex.tools import SteadyRate
from omni.isaac.cortex.cortex_utils import load_behavior_module


class CubeSpec:
    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)


def main():
    world = CortexWorld()
    robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

    obs_specs = [
        CubeSpec("RedCube", [0.7, 0.0, 0.0]),
        CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
        CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
        CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
    ]
    width = 0.0515
    for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
        obj = world.scene.add(
            DynamicCuboid(
                prim_path="/World/Obs/{}".format(spec.name),
                name=spec.name,
                size=width,
                color=spec.color,
                position=np.array([x, -0.4, width / 2]),
            )
        )
        robot.register_obstacle(obj)
    world.scene.add_default_ground_plane()

    print()
    print("loading behavior: {}".format(args.behavior))
    print()
    decider_network = load_behavior_module(args.behavior).make_decider_network(robot)
    world.add_decider_network(decider_network)

    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
