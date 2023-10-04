# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.kit import SimulationApp

import argparse

parser = argparse.ArgumentParser("example_cortex_sync_sim")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import numpy as np

from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.objects import DynamicCuboid

from omni.isaac.cortex.cortex_world import CortexWorld
from omni.isaac.cortex.robot import add_franka_to_stage
from omni.isaac.cortex.cortex_utils import load_behavior_module

enable_extension("omni.isaac.cortex_sync")
from omni.isaac.cortex_sync.cortex_ros import cortex_init_ros_node, CortexSimRobotRos, CortexSimObjectsRos


class CubeSpec:
    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)


def main():
    cortex_init_ros_node("example_cortex_sync_sim")

    world = CortexWorld()

    sim_robot = world.add_robot(
        add_franka_to_stage(name="franka_sim", prim_path="/Sim/Franka", use_motion_commander=False)
    )

    obs_specs = [
        CubeSpec("RedCube", [0.7, 0.0, 0.0]),
        CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
        CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
        CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
    ]
    width = 0.0515
    sim_objects = {}
    for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
        sim_obj = world.scene.add(
            DynamicCuboid(
                prim_path="/Sim/Obs/{}".format(spec.name),
                name="{}_sim".format(spec.name),
                size=width,
                color=spec.color,
                translation=np.array([x, -0.4, width / 2]),
            )
        )
        sim_objects[spec.name] = sim_obj
    world.scene.add_default_ground_plane()

    cortex_sim = CortexSimRobotRos(sim_robot)
    cortex_sim_objects_ros = CortexSimObjectsRos(sim_objects)

    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
