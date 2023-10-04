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

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/robot_arm_nj.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/kuka_kr120r2500pro")
gripper_usd = assets_root_path + "/Isaac/Robots/kuka_kr120r2500pro/Props/short_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/kuka_kr120r2500pro/ee_link")
gripper = SurfaceGripper(end_effector_prim_path="/World/kuka_kr120r2500pro/ee_link", translate=0.1611, direction="x")
kuka_kr120r2500pro = my_world.scene.add(
    SingleManipulator(prim_path="/World/kuka_kr120r2500pro", name="my_kuka_kr120r2500pro", end_effector_prim_name="ee_link", gripper=gripper)
)
kuka_kr120r2500pro.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))
cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([1.76, 0.4, 0.025]),
        prim_path="/World/Cube",
        scale=np.array([0.0515, 0.0515, 0.0515]),
        size=1.0,
        color=np.array([0, 0, 1]),
    )
)
my_world.scene.add_default_ground_plane()
kuka_kr120r2500pro.gripper.set_default_state(opened=True)
my_world.reset()

my_controller = PickPlaceController(name="pick_place_controller", gripper=kuka_kr120r2500pro.gripper, robot_articulation=kuka_kr120r2500pro)
articulation_controller = kuka_kr120r2500pro.get_articulation_controller()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=cube.get_local_pose()[0],
            placing_position=np.array([0.7, 0.7, 0.0515 / 2.0]),
            current_joint_positions=kuka_kr120r2500pro.get_joint_positions(),
            end_effector_offset=np.array([0, 0, 0.02]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
    if args.test is True:
        break


simulation_app.close()
