# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
import numpy as np
import carb

my_world = World(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
my_kaya = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Kaya",
        name="my_kaya",
        wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
        create_robot=True,
        usd_path=kaya_asset_path,
        position=np.array([0, 0.0, 0.02]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
)
my_world.scene.add_default_ground_plane()

kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path=my_kaya.prim_path, com_prim_path="/World/Kaya/base_link/control_offset"
)
wheel_radius, wheel_positions, wheel_orientations, mecanum_angles, wheel_axis, up_axis = (
    kaya_setup.get_holonomic_controller_params()
)
my_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        if i >= 0 and i < 500:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.4, 0.0, 0.0]))
        elif i >= 500 and i < 1000:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.4, 0.0]))
        elif i >= 1000 and i < 1200:
            my_kaya.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.05]))
        elif i == 1200:
            i = 0
        i += 1


simulation_app.close()
