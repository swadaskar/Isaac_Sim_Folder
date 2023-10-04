# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.kit import SimulationApp

import argparse

parser = argparse.ArgumentParser("example_cortex_sync")
parser.add_argument(
    "--behavior",
    type=str,
    default="behaviors/franka/block_stacking_behavior.py",
    help="Which behavior to run. See behavior/franka for available behavior files.",
)
parser.add_argument(
    "--auto_sync_objects", action="store_true", help="Automatically sync the objects with their measured poses."
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import numpy as np

from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim

from omni.isaac.cortex.cortex_object import CortexObject
from omni.isaac.cortex.cortex_world import CortexWorld
from omni.isaac.cortex.robot import add_franka_to_stage
from omni.isaac.cortex.cortex_utils import load_behavior_module

enable_extension("omni.isaac.cortex_sync")
from omni.isaac.cortex_sync.cortex_ros import (
    cortex_init_ros_node,
    CortexControlRos,
    CortexSimRobotRos,
    CortexObjectsRos,
    CortexSimObjectsRos,
)


class CubeSpec:
    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)


def main():
    cortex_init_ros_node("example_cortex_sync")

    world = CortexWorld()
    robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

    sim_prim = XFormPrim(prim_path="/Sim")
    sim_prim.set_world_pose(position=np.array([-2.0, 0.0, 0.0]))
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
    cortex_objects = {}
    sim_objects = {}
    for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
        obj = world.scene.add(
            DynamicCuboid(
                prim_path="/World/Obs/{}".format(spec.name),
                name=spec.name,
                size=width,
                color=spec.color,
                translation=np.array([x, -0.4, width / 2]),
            )
        )
        cortex_objects[spec.name] = CortexObject(obj)
        robot.register_obstacle(cortex_objects[spec.name])

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
    cortex_control = CortexControlRos(robot)
    cortex_objects_ros = CortexObjectsRos(cortex_objects, auto_sync_objects=args.auto_sync_objects)

    decider_network = load_behavior_module(args.behavior).make_decider_network(robot)
    world.add_decider_network(decider_network)

    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
