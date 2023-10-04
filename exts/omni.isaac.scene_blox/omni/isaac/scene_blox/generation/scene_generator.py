"""
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

import math
import os
import random
from typing import Any, Dict, List

import numpy as np
import yaml
from omni.isaac.core import World
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.stage import add_reference_to_stage, save_stage
from pxr import Usd
from omni.isaac.scene_blox.grid_utils.grid import Grid
from .node_generator import NodeGenerator


class SceneGenerator:
    def __init__(self, configuration_path: str, collision_check=False) -> None:
        """
        Helper class to generate a full scene from a collapsed grid. Takes a configuration
        dictionary as input to specify the base usd and generation parameters for each tile

        Args:
            generation_config (Dict[str, Any]): Configuration dictionary
        """
        with open(configuration_path, "r") as yaml_file:
            generation_config = yaml.safe_load(yaml_file)
        self.configuration_path = os.path.dirname(configuration_path)
        self.generation_config = generation_config
        self.do_collision_check = collision_check

    def generate_scene(self, grid: Grid, world: World, save_path: str):
        """
        Generate a usd corresponding to the input grid, with tile level generation applied
        according to the input configuration

        Args:
            grid (Grid): Collapsed grid solved by the wavefunction collapse
            world (World): Helper class to define the current world
            save_path (str): Path to the target usd. May be local or in server
        """
        if self.do_collision_check:
            world.play()
        tile_size = self.generation_config["tile_size"]
        world.clear()
        # First add fixed prims.
        if "fixed_prims" in self.generation_config:
            self.generate_fixed_prims(world, self.generation_config["fixed_prims"])
        ground_plane = GroundPlane("/World/ground_plane", visible=False)
        world.scene.add(ground_plane)
        for i in range(grid.rows):
            for j in range(grid.cols):
                # Get the collapsed results
                result = grid.superpositions[i][j].tile_list[0]
                tile_config = self.generation_config[result.identifier]
                # Create the base tile at the desired position and orientation
                tile_name = f"/World/{result.identifier}_{i}_{j}"
                usd_path = get_assets_root_path() + tile_config["usd"]
                if usd_path is None:
                    print(f"Cannot find tile base usd {tile_config['usd']}")
                    continue
                tile = add_reference_to_stage(usd_path, tile_name)
                prim = XFormPrim(
                    tile_name,
                    name=tile_name,
                    position=np.array([-i * tile_size, -j * tile_size, 0.0]),
                    orientation=euler_angles_to_quat([0.0, 0.0, math.pi / 2 * result.rotation]),
                )
                # If necessary, add tile level generation.
                if "generation" in tile_config:
                    self.generate_tile(tile_config, world, tile)
                # Add the new prim to the world
                world.scene.add(prim)
                print(f"Spawning {prim.name}")
        if self.do_collision_check:
            world.stop()
        # Save the scene
        save_stage(save_path)

    def generate_tile(self, tile_config: Dict[str, Any], world: World, root_prim: Usd.Prim):
        """
        Apply tile level generation according to the input configuration

        Args:
            tile_config (Dict[str, Any]): Configuration specifying the generation applied
            world (World): Helper class for spawning objects
            root_prim (Usd.Prim): Prim that will be considered as the root for the generation
        """
        for generation_data in tile_config["generation"]:
            yaml_path = None
            # If we have a list of configs to choose, choose one at random with the input weighting
            if isinstance(generation_data["config"], list):
                yaml_path = random.choices(
                    population=generation_data["config"], weights=generation_data["weights"], k=1
                )[0]
            else:
                yaml_path = generation_data["config"]
            # Special case if the generation is empty.
            if yaml_path == "None":
                continue
            # Build full path to yaml.
            yaml_path = os.path.join(self.configuration_path, yaml_path)
            # Build a generator and generate the tile
            generator = NodeGenerator.from_yaml(yaml_path)
            generator.do_collision_check = self.do_collision_check
            generator.generate(world, root_prim)

    def generate_fixed_prims(self, world: World, prims_config: List[Dict[str, Any]]):
        for config in prims_config:
            # Add the usd to the scene
            usd_path = get_assets_root_path() + config["usd"]
            fixed_prim = add_reference_to_stage(usd_path, config["prim_path"])
            # Move the xform prim to a position if required
            if "world_pose" in config:
                position = np.array(config["world_pose"]["position"])
                orientation = np.array(config["world_pose"]["orientation"])
                scale = None
                if "scale" in config:
                    scale = np.array(config["scale"])
                prim = XFormPrim(
                    config["prim_path"],
                    name=config["prim_path"],
                    position=position,
                    orientation=euler_angles_to_quat(orientation, degrees=True),
                    scale=scale,
                )
                world.scene.add(prim)
            # Add semantic label if required
            if "semantic" in config:
                add_update_semantics(fixed_prim, config["semantic"])
