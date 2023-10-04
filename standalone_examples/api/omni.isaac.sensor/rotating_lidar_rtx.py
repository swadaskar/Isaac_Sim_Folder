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
import sys
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import numpy as np
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import LidarRtx
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "/Isaac/Robots/Carter/carter_v2.usd"
my_carter = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=True,
        usd_path=asset_path,
        position=np.array([0, 0.0, 0.5]),
    )
)

# config_file_name="Example_Rotary"
my_lidar = my_world.scene.add(LidarRtx(prim_path="/World/Carter/chassis_link/stereo_cam_right/lidar", name="lidar"))

cube_1 = my_world.scene.add(
    DynamicCuboid(prim_path="/World/cube", name="cube_1", position=np.array([2, 2, 2.5]), scale=np.array([20, 0.2, 5]))
)

cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube_2", name="cube_2", position=np.array([2, -2, 2.5]), scale=np.array([20, 0.2, 5])
    )
)

my_controller = DifferentialController(name="simple_control", wheel_radius=0.04295, wheel_base=0.4132)

my_world.reset()
my_lidar.add_range_data_to_frame()
my_lidar.add_point_cloud_data_to_frame()
my_lidar.enable_visualization()
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        if i >= 0 and i < 1000:
            print(my_lidar.get_current_frame())
            # forward
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
        elif i >= 1000 and i < 1265:
            # rotate
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.0, np.pi / 12]))
        elif i >= 1265 and i < 2000:
            # forward
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
        elif i == 2000:
            i = 0
        i += 1
    if args.test is True:
        break
simulation_app.close()
