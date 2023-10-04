"""
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

from __future__ import annotations

import random
import re
from os.path import splitext
from typing import Any, Dict, List, Tuple, Union

import numpy as np
import omni
import yaml
from omni.isaac.core.prims import GeometryPrim, RigidPrim, XFormPrim
from omni.isaac.core.utils.nucleus import get_full_asset_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.world.world import World
from pxr import PhysicsSchemaTools, Usd, UsdGeom


def is_excluded(exclude: Union[List[str], str], path: str) -> bool:
    """
    Check if a path should be excluded or not

    Args:
        exclude (Union[List[str], str]): Exclude pattern or list of patterns
        path (str): Path to check

    Returns:
        bool: True if excluded
    """
    if isinstance(exclude, list):
        for pattern in exclude:
            if pattern in path:
                return True
        return False
    else:
        if exclude is None:
            return False
        return exclude in path


def retrieve_assets(
    root_path: str, depth: int, all_assets: List[str], pattern: str = None, exclude: Union[List[str], str] = None
):
    """
    Retrieve all .usd file from a root folder on Omniverse at a given depth with
    matching patterns or exclusions

    Args:
        root_path (str): Url of the root folder
        depth (int): Number of folders to go down before looking for .usd
        all_assets (List[str]):  List of all found .usd urls
        pattern (str, optional): If not None, regular expression must match the usd name.
                                 Defaults to None.
        exclude (Union[List[str], str], optional): If not None, pattern or list
                                                   of patterns to exclude. Defaults to None.
    """
    if is_excluded(exclude, root_path):
        return
    result, content = omni.client.list(root_path)
    if result != omni.client.Result.OK:
        return
    # If we are at the desired level, we list all content ending with .usd
    if depth == 0:
        for i in range(len(content)):
            asset_path = content[i].relative_path
            _, ext = splitext(asset_path)
            if ext == ".usd":
                is_valid = True
                if pattern is not None:
                    match = re.match(pattern, asset_path)
                    is_valid = match is not None
                if exclude is not None:
                    is_valid = is_valid and not is_excluded(exclude, asset_path)
                if is_valid:
                    all_assets.append(f"{root_path}/{asset_path}")
    # Else we recursively query the sub folders.
    else:
        for i in range(len(content)):
            folder_path = content[i].relative_path
            retrieve_assets(f"{root_path}/{folder_path}", depth - 1, all_assets, pattern, exclude)


def sample_position(position_config: Dict[str, Any]) -> np.array:
    """
    Sample a vector according to the noise configuration. Supported noise models currently
    are uniform and normal. Noise is relative to a base position.

    Args:
        position_config (Dict[str, Any]): Sampling parameters. The default value is specified in
                                        "base" while the noise parameters are specified in "noise"

    Raises:
        NotImplementedError: If position_config["noise"]["type"] is not supported

    Returns:
        np.array: Sampled vector equal to base + noise
    """
    base_position = np.array(position_config["base"])
    if "noise" not in position_config:
        return base_position
    noise_config = position_config["noise"]
    params = noise_config["params"]
    if noise_config["type"] == "normal":
        noise = np.random.normal(params["mean"], params["stddev"])
    elif noise_config["type"] == "uniform":
        noise = np.random.uniform(params["low"], params["high"])
    elif noise_config["type"] == "choice":
        choice = random.randrange(1, len(params) + 1)
        noise = params[choice]
    else:
        raise NotImplementedError
    return base_position + noise


def get_collision_subprims(prim: Usd.Prim, children: List[str]) -> None:
    """
    Get paths to children of a prim to which collision can be added.

    Args:
        prim (Usd.Prim): Root prim to search for collisions
        children (List[str]): List of children prim names
    """
    for child in prim.GetChildren():
        # Select only meshes
        if child.IsInstanceable() or child.IsA(UsdGeom.Mesh):
            children.append(child.GetPath().pathString)
            continue
        # Recursively call the method on subprims
        get_collision_subprims(child, children)


def get_collision_check_meshes(prim: Usd.Prim, meshes: List[str]) -> None:
    """
    Get paths to children of a prim to which collision can be added. Temporarily removes instancing
    to find subprims that are meshes.

    Args:
        prim (Usd.Prim): Root prim for the collision check
        meshes (List[str]): List of children that are UsdGeomMesh for collision check. Includes
                            subprims of instanceable prims.
    """
    instanceables = []
    for child in prim.GetChildren():
        # If the prim is instanceable, temporarily make it not instanced to get subprims.
        if child.IsInstanceable():
            child.SetInstanceable(False)
            instanceables.append(child)
        # Select only meshes
        if child.IsA(UsdGeom.Mesh):
            meshes.append(child.GetPath().pathString)
            continue
        # Recursively call the method on subprims
        get_collision_check_meshes(child, meshes)
    for modified_prim in instanceables:
        modified_prim.SetInstanceable(True)


def select_variant(variant_sets: Usd.VariantSets, variant_name: str, chosen_names: List[str] = None) -> None:
    """
    Select a value for a variant at random from a given set of variant sets, a variant name
    and optionally a list of possible names to choose from

    Args:
        variant_sets (Usd.VariantSets): Variant sets of the current prim
        variant_name (str): Name of the variant where we want to choose a value
        chosen_names (List[str]): If not None, only choose between value names in the list
    """
    # Get the specified variant
    variants = variant_sets.GetVariantSet(variant_name)
    variant_names = variants.GetVariantNames()
    # Sometimes we have empty variants
    if len(variant_names) == 0:
        return
    possible_indexes = []
    # Select possible indexes according to chosen variants if specified
    for idx, name in enumerate(variant_names):
        if chosen_names is None or name in chosen_names:
            possible_indexes.append(idx)
    # Set one of the possible variants at random
    chosen = random.choice(possible_indexes)
    variants.SetVariantSelection(variant_names[chosen])


class NodeGenerator:
    """
    Helper class to generate assets on a given node from a user configuration

    Args:
        config_dict (Dict[str, Any]): Generation configuration (loaded from yaml)
        collision_check (bool, optional): If True, assets to be generated with collision and
            rigid body phsyics are checked for collisions before being added to the scene.
            They are not generated if they cause a collision. Defaults to False.
    """

    def __init__(self, config_dict: Dict[str, Any], collision_check: bool = False) -> None:
        self.config = config_dict
        self.do_collision_check = collision_check

    def generate(self, world: World, root_prim: Usd.Prim) -> None:
        """
        Execute asset generation and randomization under the given root prim

        Args:
            world (World): Current world being generated
            root_prim (Usd.Prim): Prim that will be used as root for the generation. All created
                prims will be children of the root prim.
        """
        # Find the prims matching the description under the target root prim.
        root_prims = self.find_prims_in_stage(root_prim, self.config["root_prim"])
        # Apply generation for all of them
        for root_prim in root_prims:
            root_path = root_prim.GetPath().pathString
            # Iterate on all applied generations
            for prim_config in self.config["generated"]:
                spawn_count = 1
                if "spawn_count" in prim_config:
                    spawn_count = prim_config["spawn_count"]
                for i in range(spawn_count):
                    self.generate_object(prim_config, root_path, world, i)

    def generate_object(self, prim_config: Dict[str, Any], root_path: str, world: World, index: int) -> bool:
        """
        Run the generation for a specific object in the configuration. The generation first checks
        spawn probaility, then retrieves possible usds in the configuration. Then a prim is spawned
        at the target position and orientation (with noise applied if necessary). Then if collision
        check is turned on and the object has physics enabled, the object is added only if it does
        not enter in collision with something else in the scene other than the ground. Finally,
        semantic segmentation and variants are applied if required.

        Args:
            prim_config (Dict[str, Any]): Object generation configuration
            root_path (str): Path of the parent prim to spawn the object
            world (World): World being generated
            index (int): Index of the spawned object (if several are spawned)

        Returns:
            bool: True if an objects has been spawned, False else
        """
        # First, check if the object should spawn.
        if "spawn_proba" in prim_config:
            needs_spawn = np.random.uniform()
            if needs_spawn > prim_config["spawn_proba"]:
                return False
        # Then retrieve usds matching the criteria
        possible_usds = []
        usd_config = prim_config["usd_config"]
        root_on_server = get_full_asset_path(usd_config["root"])
        retrieve_assets(
            root_on_server,
            usd_config["search_depth"],
            possible_usds,
            usd_config.get("filter"),
            usd_config.get("exclude"),
        )
        if len(possible_usds) == 0:
            print(f"Warning, cannot find usds matching {usd_config}")
            return False
        # Choose one at random.
        chosen_usd = random.choice(possible_usds)
        target_path = f"{root_path}/{prim_config['path']}_{index}"
        # Sample the position and orientation
        position = sample_position(prim_config["position"])
        orientation = sample_position(prim_config["orientation"])
        scale = None
        if "scale" in prim_config:
            scale = np.array(prim_config["scale"])
        # Place the prim in the world
        xform_prim = XFormPrim(target_path, name=target_path, scale=scale)
        xform_prim.set_local_pose(position, euler_angles_to_quat(orientation, degrees=True))
        world.scene.add(xform_prim)
        if self.do_collision_check and "physics" in prim_config:
            if prim_config["physics"].get("rigid_body", False):
                in_collision, collision_path = self.check_collision(world, chosen_usd, target_path)
                if in_collision:
                    print(
                        f"Failed to add {chosen_usd} at {target_path}: physics enabled and collision detected with {collision_path}"
                    )
                    return False
        # Create the prim
        generated = add_reference_to_stage(chosen_usd, target_path)
        # Add semantic if needed
        if "semantic" in prim_config:
            add_update_semantics(generated, prim_config["semantic"], "class")
        # Select variant if desired
        if "variant" in prim_config:
            variant_config = prim_config["variant"]
            # Retrieve all possible variants for the prim
            if "sub_prim" in variant_config:
                variant_prim = world.stage.GetPrimAtPath(target_path + "/" + variant_config["sub_prim"])
                variant_sets = variant_prim.GetVariantSets()
            else:
                variant_sets = generated.GetVariantSets()
            # Get the specified variant
            if variant_config["name"] != "*":
                select_variant(variant_sets, variant_config["name"], variant_config.get("choice", None))
            else:
                for variant_name in variant_sets.GetNames():
                    select_variant(variant_sets, variant_name, None)
        if "physics" in prim_config:
            physics_config = prim_config["physics"]
            apply_children = physics_config.get("apply_children", False)
            if apply_children:
                children_paths = []
                get_collision_subprims(generated, children_paths)
                for child_path in children_paths:
                    if "exclude" in physics_config and physics_config["exclude"] in child_path:
                        continue
                    self.apply_physics_config(physics_config, child_path)
            else:
                self.apply_physics_config(physics_config, target_path)
        return True

    def apply_physics_config(self, prim_config: Dict[str, Any], target_path: str):
        """
        Apply physics configuration to a given prim (collision and rigid body)

        Args:
            prim_config (Dict[str, Any]): Physics configuration
            target_path (str): Path of the target prim
        """
        if "collision" in prim_config:
            collision_prim = GeometryPrim(target_path, name=target_path, collision=True)
            collision_prim.set_collision_approximation(prim_config["collision"])
        if prim_config.get("rigid_body", False) and self.do_collision_check:
            rigid_prim = RigidPrim(target_path, target_path)
            rigid_prim.enable_rigid_body_physics()

    def check_collision(self, world: World, chosen_usd: str, target_path: str) -> Tuple[bool, List[str]]:
        """
        Check if adding a given usd to the scene would result in a collision

        Args:
            world (World): World being generated
            chosen_usd (str): Path to the usd to be added
            target_path (str): Path of the already placed xform prim where the asset will be added

        Returns:
            Tuple[bool, List[str]]: collision_detected, list of colliding prim paths
        """
        # Add the prim to the stage
        prim = add_reference_to_stage(chosen_usd, target_path)
        # Do physics step
        world.step(render=False)
        self.overlaps = []
        self.collision_root_path = target_path
        import omni.physx

        interface = omni.physx.get_physx_scene_query_interface()
        meshes = []
        get_collision_check_meshes(prim, meshes)
        # Iterate on all meshes in the added prim
        for mesh_path in meshes:
            # Get the encoded path
            path_id_0, path_id_1 = PhysicsSchemaTools.encodeSdfPath(mesh_path)
            # Look for collisions. We cannot use the direct hit count because of ground plane
            interface.overlap_mesh(path_id_0, path_id_1, self.on_hit, False)
            # If we have some non ground overlaps, we do not want to keep the prim here so we remove it.
            if len(self.overlaps) > 0:
                prim.GetReferences().ClearReferences()
                break
        # Return detected overlaps
        return len(self.overlaps) > 0, self.overlaps

    def on_hit(self, hit) -> bool:
        """
        Callback method for checking a given mesh for collisions

        Args:
            hit (SceneQueryHits): Structure containing information about collision hits

        Returns:
            bool: True to continue scanning for collisions
        """
        # Check that we are not hitting the ground plane (this is normal), and not having self collision
        if (
            hit.rigid_body not in self.overlaps
            and "ground_plane" not in hit.rigid_body
            and self.collision_root_path not in hit.rigid_body
        ):
            self.overlaps.append(hit.rigid_body)
        return True

    def find_prims_in_stage(self, root_prim: Usd.Prim, prim_pattern: str) -> List[Usd.Prim]:
        """
        Look for prim matching the input pattern which are children of the root prim

        Args:
            root_prim (Usd.Prim): Root prim (search will look in children)
            prim_pattern (str): Regex pattern to match the prim name

        Returns:
            List[Usd.Prim]: All prims with matching names
        """
        matched_prims = []
        for prim in root_prim.GetAllChildren():
            prim_name = prim.GetName()
            if re.match(prim_pattern, prim_name) is not None:
                matched_prims.append(prim)
        return matched_prims

    @staticmethod
    def from_yaml(yaml_path: str) -> NodeGenerator:
        """
        Build a NodeGenerator from a yaml

        Args:
            yaml_path (str): Path to the yaml file

        Returns:
            NodeGenerator: Generator initialized with the yaml config
        """
        with open(yaml_path, "r") as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        return NodeGenerator(yaml_data)
