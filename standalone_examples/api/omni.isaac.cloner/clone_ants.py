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
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.cloner import GridCloner

import numpy as np
import carb
import sys

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# create initial robot
asset_path = assets_root_path + "/Isaac/Robots/Ant/ant.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Ants/Ant_0")

# create GridCloner instance
cloner = GridCloner(spacing=2)

# generate paths for clones
target_paths = cloner.generate_paths("/World/Ants/Ant", 4)

# clone
position_offsets = np.array([[0, 0, 1]] * 4)
cloner.clone(
    source_prim_path="/World/Ants/Ant_0",
    prim_paths=target_paths,
    position_offsets=position_offsets,
    replicate_physics=True,
    base_env_path="/World/Ants",
)

# create ArticulationView
ants = ArticulationView(prim_paths_expr="/World/Ants/.*/torso", name="ants_view")
my_world.scene.add(ants)

my_world.reset()
for i in range(1000):
    print(ants.get_world_poses())
    my_world.step()
simulation_app.close()
