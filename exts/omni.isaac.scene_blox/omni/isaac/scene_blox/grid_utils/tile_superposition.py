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
from typing import List, Set, Tuple

import numpy as np
from .tile import Tile


class TileSuperposition:
    def __init__(self, tile_list: List[Tile], tile_weights: List[float]) -> None:
        """
        Initialize a tile superposition from a list of possible tiles and weights

        Args:
            tile_list (List[Tile]): List of possible tiles
            tile_weights (List[float]): Corresponding weights for sampling (will be normalized)
        """
        self.tile_list = tile_list
        weights_sum = sum(tile_weights)
        self.tile_weights = [w / weights_sum for w in tile_weights]

    def collapse(self, valid_indexes, excluded_indexes) -> None:
        """
        Randomly choose a possible tile, following the desired distribution.
        """
        index_choices = [i for i in valid_indexes if i not in excluded_indexes]
        weights_choices = [self.tile_weights[i] for i in valid_indexes if i not in excluded_indexes]
        chosen_index = random.choices(population=index_choices, weights=weights_choices, k=1)[0]
        return chosen_index, self.tile_list[chosen_index]

    def get_compatible_indexes(self, neighbor_tile: Tile, neighbor_position: int) -> List[int]:
        """
        List the indexes of all compatible tiles with a given neighbor tile at a given
        relative position

        Args:
            neighbor_tile (Tile): Neighbor tile to be compared with the possibilities
            neighbor_position (int): Relative position of the tile wrt the superposition

        Returns:
            List[int]: Indexes of the tiles in the superposition which are compatible
        """
        good_indexes = []
        for tile_idx, tile in enumerate(self.tile_list):
            compatible = tile.is_tile_compatible(neighbor_position, neighbor_tile.identifier, neighbor_tile.rotation)
            if compatible:
                good_indexes.append(tile_idx)
        return good_indexes

    def select_indexes(self, good_indexes: List[int]) -> bool:
        # If all indexes are compatible, no need to do anything
        if len(good_indexes) == len(self.tile_list):
            return False
        # Select the desired tiles.
        new_tiles = [self.tile_list[i] for i in good_indexes]
        new_weights = [self.tile_weights[i] for i in good_indexes]
        # Normalize the new weights to 1.
        summed_total = sum(new_weights)
        # Replace tiles and weights
        self.tile_list = new_tiles
        self.tile_weights = [w / summed_total for w in new_weights]
        return True

    def get_possibilities_count(self) -> int:
        """
        Get the current number of possibilities in the stack

        Returns:
            int: Number of potential tiles
        """
        return len(self.tile_list)

    def get_entropy(self, valid_indexes: List[int] = None) -> float:
        """
        Computes the entropy of the current tile superposition

        Returns:
            float: Entropy value
        """
        if valid_indexes is None:
            valid_indexes = np.arange(len(self.tile_list))
        entropy = 0.0
        for index in valid_indexes:
            entropy += -self.tile_weights[index] * np.log(self.tile_weights[index])
        return entropy

    def filter_by_type(self, filter_types: List[str]) -> Tuple[Set[int], Set[int]]:
        """
        Filter possible indexes by tile type: select only the tiles of the given type

        Args:
            filter_types (List[str]): List of tile identifiers

        Returns:
            Tuple[Set[int], Set[int]]: compatible_indexes, excluded_indexes
                                       Indexes matching the given type or not
        """
        filtered_indexes = []
        excluded_indexes = []
        for k, tile in enumerate(self.tile_list):
            if tile.identifier in filter_types:
                filtered_indexes.append(k)
            else:
                excluded_indexes.append(k)
        return set(filtered_indexes), set(excluded_indexes)

    def filter_by_rotation(self, tile_identifiers: List[str], orientations: List[int]) -> Tuple[Set[int], Set[int]]:
        """
        Filter possible indexes by rotation for a given tile type.

        Args:
            tile_identifiers (List[str]): List of the tiles with rotation filtering
            orientations (List[int]): List of rotation indexes to filter

        Returns:
            Tuple[Set[int], Set[int]]: Indexes matching the given rotation and type
                                       (or of other type) and complement set
        """
        filtered_indexes = set()
        excluded_indexes = set()
        for k, tile in enumerate(self.tile_list):
            if tile.identifier not in tile_identifiers:
                filtered_indexes.add(k)
                continue
            if tile.rotation in orientations:
                filtered_indexes.add(k)
            else:
                excluded_indexes.add(k)
        return filtered_indexes, excluded_indexes
