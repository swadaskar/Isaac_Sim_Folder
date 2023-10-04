"""
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

from __future__ import annotations

import copy
from typing import Dict, List, Tuple

import yaml

TILE_T_SYM = {0: [0, 1, 2], 1: [0, 1, 3], 2: [0, 2, 3], 3: [0]}
TILE_L_SYM = {0: [0, 3], 1: [0, 1], 2: [0, 3], 3: [0, 1]}
TILE_I_SYM = {0: [0, 2], 1: [0, 2], 2: [0, 2], 3: [0, 2]}
TILE_X_SYM = {i: [j for j in range(4)] for i in range(4)}
TILE_D_SYM = {0: [0, 1, 3], 1: [0, 2, 3], 2: [0], 3: [0, 1, 2]}


class Tile:
    def __init__(self, identifier: str, symetries: Dict[int, List[int]] = None) -> None:
        """
        Tile with an identifier, and symetries to be added when adding compatible tiles.
        A tile has a rotation state, which corresponds to the number of 90 degrees rotations
        counter-clockwise (0: 0 degrees, 1: 90, 2: 180, 3: 270)

        Args:
            identifier (str): Name of the tile
            symetries (Dict[int, List[int]], optional): Rotations that can be applied to the tile
                while keeping connectivity, for each of the tile possible rotation. For example,
                if the symetries contain 0: [0, 1] it means the tile in position 0 can be rotated
                to position 1 (90 degrees counter-clockwise) and still have the same connection on
                the right side of the tile. Defaults to None.
        """
        self.identifier = identifier
        self.rotation: int = 0
        self.adjacency = set()
        if symetries is not None:
            self.symetries = symetries
        else:
            self.symetries = {i: [0] for i in range(4)}

    def is_tile_compatible(self, relative_pos: int, identifier: str, rotation: int) -> bool:
        """
        Check if it is possible to have another tile next to the current one

        Args:
            relative_pos (int): Relative position of the neighbor tile. 0 means the tile is on the
                left, 1 on top, 2 on the right and 3 on the bottom
            identifier (str): Identifier of the neighbor tile
            rotation (int): Rotation state of the neighbor tile

        Returns:
            bool: True if both tiles are compatible
        """
        is_compatible = False
        # Iterate over all possibile configuration
        for self_rotation, possible_tile, possible_rotation in self.adjacency:
            # We are looking for the good initial tile rotation.
            if (self.rotation - relative_pos) % 4 != self_rotation:
                continue
            # Then we check if the asked tile is in the list
            if identifier != possible_tile:
                continue
            # If we have the correct tile, we correct the relative positionning
            if (rotation - relative_pos) % 4 == possible_rotation:
                is_compatible = True
                break
        return is_compatible

    def add_compatible_tile(self, self_rotation: int, possible_tile: str, possible_rotation: int) -> None:
        """
        Add a compatible neighbor tile. If existing, symetries are applied.

        Args:
            self_rotation (int): Rotation applied to the current tile
            possible_tile (str): Identifier of the neighbor tile
            possible_rotation (int): Rotation state of the neighbor tile
        """
        symetries = self.symetries[self_rotation]
        compatible = []
        for sym in symetries:
            compatible_tuple = ((sym + self_rotation) % 4, possible_tile, possible_rotation)
            self.adjacency.add(compatible_tuple)
            compatible.append(compatible_tuple)
        return compatible

    def to_dict(self) -> Dict:
        """
        Serialize the tile to a dictionary

        Returns:
            Dict: Serializable dictionary
        """
        return {"identifier": self.identifier, "rotation": self.rotation, "adjacency": list(self.adjacency)}

    @staticmethod
    def from_dict(dictionary: Dict) -> Tile:
        """
        Build a tile from a saved dictionary

        Args:
            dictionary (Dict): Serialized dictionary
        """
        tile = Tile(dictionary["identifier"])
        tile.rotation = dictionary["rotation"]
        # Small fix needed because set() is not serializable
        tile.adjacency = set()
        for adjacency_data in dictionary["adjacency"]:
            tile.adjacency.add(tuple(adjacency_data))
        return tile


def tile_loader(json_file_path: str) -> Tuple[List[Tile], List[float], float]:
    with open(json_file_path, "r") as j_file:
        tiles_data = yaml.safe_load(j_file)
    tiles = {}
    weights = {}
    for tile_data in tiles_data["tiles"]:
        tile_id = tile_data["id"]
        symetries = None
        if "symetry" in tile_data:
            tile_symetry = tile_data["symetry"]
            if tile_symetry == "T":
                symetries = TILE_T_SYM
            elif tile_symetry == "L":
                symetries = TILE_L_SYM
            elif tile_symetry == "I":
                symetries = TILE_I_SYM
            elif tile_symetry == "X":
                symetries = TILE_X_SYM
            elif tile_symetry == "D":
                symetries = TILE_D_SYM
        tiles[tile_id] = Tile(tile_id, symetries)
        weights[tile_id] = tile_data["weights"]
    for adjacency_data in tiles_data["adjacencies"]:
        current_id = adjacency_data["id"]
        for neighbor_info in adjacency_data["neighbors"]:
            neighbor_id = neighbor_info["neighbor_id"]
            # Add listed adjacencies
            added = tiles[current_id].add_compatible_tile(
                neighbor_info["self_rotation"], neighbor_id, neighbor_info["neighbor_rotation"]
            )
            # Add flipped adjacencies
            for self_rot, _, neighbor_rot in added:
                tiles[neighbor_id].add_compatible_tile((neighbor_rot + 2) % 4, current_id, (self_rot + 2) % 4)
    all_tiles = []
    all_weights = []
    for tile_id, tile in tiles.items():
        for rotation in range(4):
            current_tile = copy.deepcopy(tile)
            current_tile.rotation = rotation
            all_tiles.append(current_tile)
        all_weights += weights[tile_id]
    return all_tiles, all_weights
