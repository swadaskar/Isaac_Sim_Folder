import argparse
import copy
import math
import re
from typing import List, Tuple

import numpy as np
import yaml
from omni.isaac.kit import SimulationApp


def get_ids_list(equivalent_ids: List[List[str]], tile_id: str) -> List[str]:
    """
    For a given tile identifier, get all equivalent tile identifiers

    Args:
        equivalent_ids (List[List[str]]): List of all equivalence classes
        tile_id (str): Input tile identifier

    Returns:
        List[str]: List of all tile identifier equivalent to the given one
    """
    ids_list = [tile_id]
    for equivalent_list in equivalent_ids:
        if tile_id in equivalent_list:
            return equivalent_list
    return ids_list


def main(args):
    if args.rules_config is not None:
        with open(args.rules_config, "r") as config_file:
            equivalent_ids = yaml.safe_load(config_file)
    else:
        equivalent_ids = []

    simulation_app = SimulationApp()

    from omni.isaac.core import World
    from omni.isaac.core.utils.rotations import quat_to_euler_angles
    from omni.isaac.core.utils.stage import open_stage
    from pxr import Gf, Usd, UsdGeom

    def get_identifier(prim: Usd.Prim) -> str:
        """
        Get the base name of the prim, assuming it has the format <primname>_<number>

        Args:
            prim (Usd.Prim): Input prim

        Returns:
            str: Base name (or full name if no match)
        """
        match = re.match("(.*)_\d+", prim.GetName())
        if prim.GetCustomData():
            return prim.GetCustomData()["tile_name"]
        if match is None:
            tile_name = prim.GetName()
        else:
            tile_name = match.group(1)
        return tile_name

    def get_prim_pose_data(prim: Usd.Prim) -> Tuple[np.array, int]:
        """
        Get prim translation and rotation (as an index of 90 degrees rotation)

        Args:
            prim (Usd.Prim): Input prim

        Returns:
            Tuple[np.array, int]: Translation and rotation index
        """
        prim_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        transform = Gf.Transform()
        transform.SetMatrix(prim_tf)
        prim_pose = transform.GetTranslation()
        quat = transform.GetRotation().GetQuat()
        quat_array = [quat.GetReal()] + list(quat.GetImaginary())
        quat_array = np.array(quat_array)
        prim_orientation = round(math.degrees(quat_to_euler_angles(quat_array)[2]), 0)
        return prim_pose, int((prim_orientation / 90) % 4)

    world = World()
    print(f"Opening {args.stage}")
    open_stage(args.stage)
    stage = world.stage

    max_distance = args.tile_size + 0.1

    root: Usd.Prim = stage.GetPrimAtPath("/World")
    all_prims = root.GetChildren()
    adjacencies = {}
    for i in range(len(all_prims)):
        prim = all_prims[i]
        if prim.GetTypeName() != "Xform":
            continue
        tile_name = get_identifier(prim)

        prim_pose, prim_orientation = get_prim_pose_data(prim)
        for j in range(len(all_prims)):
            if i == j:
                continue
            other_prim = all_prims[j]
            if other_prim.GetTypeName() != "Xform":
                continue
            other_pose, other_prim_orientation = get_prim_pose_data(other_prim)
            diff = other_pose - prim_pose
            distance = np.linalg.norm(diff)
            if distance < max_distance:
                angle = math.degrees(math.atan2(diff[1], diff[0]))
                # -90 degrees means the neighbor tile is right, so the rotation index should be 0
                diff_to_left = int((angle + 90) / 90)
                if tile_name not in adjacencies:
                    adjacencies[tile_name] = set()
                # Put the adjacency back in the convention of self left, neighbor right
                # Add 4 to avoid negative indexes (does not affect positive)
                self_rotation = (prim_orientation - diff_to_left + 4) % 4
                neighbor_rotation = (other_prim_orientation - diff_to_left + 4) % 4
                neighbor_name = get_identifier(other_prim)
                # Add the adjacency
                adjacencies[tile_name].add((self_rotation, neighbor_name, neighbor_rotation))

    all_adjacencies = []
    all_tiles = []
    for name, data in adjacencies.items():
        to_add = get_ids_list(equivalent_ids, name)
        for tile_name in to_add:
            tile_data = {"id": tile_name, "weights": [1, 1, 1, 1]}
            all_tiles.append(tile_data)
            adj_data = {"id": tile_name, "neighbors": []}
            for neighbor_data in data:
                self_rotation, neighbor_id, neighbor_rotation = neighbor_data
                neighbor_list = get_ids_list(equivalent_ids, neighbor_id)
                for neighbor_name in neighbor_list:
                    adj_info = {
                        "self_rotation": self_rotation,
                        "neighbor_id": neighbor_name,
                        "neighbor_rotation": neighbor_rotation,
                    }
                    adj_data["neighbors"].append(adj_info)
            all_adjacencies.append(adj_data)
    yaml_data = {"tiles": all_tiles, "adjacencies": all_adjacencies}

    print(f"Saving adjacencies to {args.save_path}")
    with open(args.save_path, "w") as y_file:
        yaml.dump(yaml_data, y_file)

    simulation_app.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", help="Path to the stage used as an example for rule generation")
    parser.add_argument("save_path", help="Path where the generated yaml will be saved")
    parser.add_argument("tile_size", type=float, help="Size of a tile (in scene units)")
    parser.add_argument("--rules_config", default=None, help="If not empty contains tile equivalence")
    args = parser.parse_args()
    main(args)
