"""
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

from __future__ import annotations

from typing import Set, Tuple

import yaml
from .tile_superposition import TileSuperposition


class GridConstraints:
    """
    Wrapper class to express constraints on the grid. Constraints are expressed on row/col ranges
    and can be either "exclude" (prevent a tile type from being picked) or "restrict" (pick only
    in the given tile types set)
    """

    def __init__(self) -> None:
        self.constraints_list = []
        self.tiles_types = {}

    def reset(self) -> None:
        self.tiles_types = {}

    def get_compatible_indexes(
        self, superposition: TileSuperposition, valid_indexes: Set[int], grid_position: Tuple[int, int]
    ) -> Tuple[Set[int], Set[int]]:
        """
        For a given superposition and its position in the grid, get the compatible and incompatible
        indexes with the given constraints. (exclude or restrict)

        Args:
            superposition (TileSuperposition): Current tile superposition to check
            valid_indexes (Set[int]): If not None, the indexes of the tiles to restrict to
            grid_position (Tuple[int, int]): Position of the tile in the grid (row, col)

        Returns:
            Tuple[Set[int], Set[int]]: compatible_indexes, incompatible_indexes sets
        """
        row, col = grid_position
        if valid_indexes is not None:
            compatible_indexes = valid_indexes.copy()
        else:
            compatible_indexes = set([i for i in range(superposition.get_possibilities_count())])
        incompatible_indexes = compatible_indexes.copy()
        # Check all constraints
        for constraint in self.constraints_list:
            area_data = constraint["area"]
            for row_range, col_range in zip(area_data["rows"], area_data["cols"]):
                if row < row_range[0] or row > row_range[1] or col < col_range[0] or col > col_range[1]:
                    continue
                constraint_type = constraint["type"]
                if "type" in constraint_type:
                    matching, _ = superposition.filter_by_type(constraint["identifiers"])
                    # If this constraint is a restriction, we keep only the matching indexes
                    if constraint_type == "restrict_type":
                        compatible_indexes.intersection_update(matching)
                    # If this constraint is an exclusion, we remove the matching indexes
                    elif constraint_type == "exclude_type":
                        for to_exclude in matching:
                            # Use discard to avoid errors if it is already excluded
                            compatible_indexes.discard(to_exclude)
                elif "rotation" in constraint_type:
                    matching, _ = superposition.filter_by_rotation(constraint["identifier"], constraint["rotations"])
                    if constraint_type == "restrict_rotation":
                        compatible_indexes.intersection_update(matching)
                    else:
                        for to_exclude in matching:
                            compatible_indexes.discard(to_exclude)
                elif "count" in constraint_type:
                    for tile_type, max_count in zip(constraint["identifiers"], constraint["max_count"]):
                        current_count = self.get_type_count(tile_type)
                        if current_count >= max_count:
                            matching, _ = superposition.filter_by_type([tile_type])
                            for to_exclude in matching:
                                compatible_indexes.discard(to_exclude)
                else:
                    raise ValueError(f"Unrecognized constraint type {constraint_type}")
        # For convenience, also compute incompatible indexes
        for compatible in compatible_indexes:
            incompatible_indexes.discard(compatible)
        return compatible_indexes, incompatible_indexes

    def update_constraints(self, grid_position: Tuple[int, int], tile_identifier: str) -> None:
        """
        Add the selected tile type and position for constraints checking

        Args:
            grid_position (Tuple[int, int]): Collapsed tile position
            tile_identifier (str): Collapsed tile type identifier
        """
        self.tiles_types[grid_position] = tile_identifier

    def get_type_count(self, tile_identifier: str) -> int:
        """
        Count the number of tiles of a certain type already collapsed

        Args:
            tile_identifier (str): Tile type identifier

        Returns:
            int: Number of tiles of the given type
        """
        type_count = 0
        for _, tile_type in self.tiles_types.items():
            if tile_identifier == tile_type:
                type_count += 1
        return type_count

    @staticmethod
    def from_yaml(yaml_path: str, grid_rows: int, grid_cols: int) -> GridConstraints:
        """
        Build a constraints from a yaml file for given grid dimensions

        Args:
            yaml_path (str): Path to the constraints definition yaml file
            grid_rows (int): Number of rows of the generated grid
            grid_cols (int): Number of cols of the generated grid

        Returns:
            GridConstraints: Resulting constraints
        """
        constraints = GridConstraints()
        with open(yaml_path, "r") as input_file:
            constraints.constraints_list = yaml.safe_load(input_file)
        for constraint in constraints.constraints_list:
            area_data = constraint["area"]
            for idx, (row_range, col_range) in enumerate(zip(area_data["rows"], area_data["cols"])):
                for i in range(2):
                    # If constraints are given in negative, adapt it to actual grid size
                    if row_range[i] < 0:
                        area_data["rows"][idx][i] += grid_rows
                    if col_range[i] < 0:
                        area_data["cols"][idx][i] += grid_cols
        return constraints
