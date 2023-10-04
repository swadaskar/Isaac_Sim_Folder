# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

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
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import ContactSensor
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np
import carb
import argparse
import sys

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
asset_path = assets_root_path + "/Isaac/Robots/Ant/ant.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Ant")

ant = my_world.scene.add(Articulation(prim_path="/World/Ant/torso", name="ant", translation=np.array([0, 0, 1.5])))

ant_foot_prim_names = ["right_back_foot", "left_back_foot", "front_right_foot", "front_left_foot"]

translations = np.array(
    [[0.38202, -0.40354, -0.0887], [-0.4, -0.40354, -0.0887], [-0.4, 0.4, -0.0887], [0.4, 0.4, -0.0887]]
)

ant_sensors = []
for i in range(4):
    ant_sensors.append(
        my_world.scene.add(
            ContactSensor(
                prim_path="/World/Ant/" + ant_foot_prim_names[i] + "/contact_sensor",
                name="ant_contact_sensor_{}".format(i),
                min_threshold=0,
                max_threshold=10000000,
                radius=0.1,
                translation=translations[i],
            )
        )
    )

ant_sensors[0].add_raw_contact_data_to_frame()
my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        print(ant_sensors[0].get_current_frame())
        if my_world.current_time_step_index == 0:
            my_world.reset()

simulation_app.close()
