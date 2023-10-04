# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.prims import GeometryPrim, XFormPrim, RigidPrim
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.universal_robots.controllers import PickPlaceController
from omni.isaac.core.objects import DynamicCuboid
import carb
import sys
import numpy as np
import argparse
from omni.physx.scripts.utils import setRigidBody
import omni.replicator.core as rep

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

def add_part_custom(world, part_name, prim_name, scale, position, orientation):
    base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

    add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/{prim_name}") # gives asset ref path
    part= world.scene.add(RigidPrim(prim_path=f'/{prim_name}', name=f"q{prim_name}", mass=0.2)) # declares in the world

    ## add part
    part.set_local_scale(scale)
    part.set_world_pose(position=position, orientation=orientation)
    rep.physics.collider(approximation_shape = "convexDecomposition")
    return part

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)

asset_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")

gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
ur10 = my_world.scene.add(
    SingleManipulator(prim_path="/World/UR10", name="my_ur10", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([0,0,0]), orientation=np.array([1, 0, 0, 0]), scale=np.array([1, 1, 1]))
)
ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))
engine = add_part_custom(my_world,"engine", "engine_small", np.array([0.001,0.001,0.001]), np.array([-1.05875, -0.28123, 0.09381]), np.array([0.70711, -0.70711, 0, 0]))

cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([0.5,0.5,0.5]),
        prim_path="/World/Cube",
        scale=np.array([0.025, 0.025, 0.025]),
        size=1.0,
        color=np.array([0, 0, 1]),
    )
)



my_world.scene.add_ground_plane(size = 100, static_friction=1, dynamic_friction=1)
ur10.gripper.set_default_state(opened=True)
my_world.reset()

my_controller = PickPlaceController(name="pick_place_controller", gripper=ur10.gripper, robot_articulation=ur10)
articulation_controller = ur10.get_articulation_controller()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()

        engine_position = engine.get_local_pose()[0]
        # engine_position[2] -= 0.015
        print(engine_position)

        actions = my_controller.forward(
            picking_position=engine_position,
            placing_position=np.array([0.89988, -0.18931, 0.0515 / 2.0]),
            current_joint_positions=ur10.get_joint_positions(),
            end_effector_offset=np.array([0, 0, 0.02]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
    if args.test is True:
        break


simulation_app.close()
