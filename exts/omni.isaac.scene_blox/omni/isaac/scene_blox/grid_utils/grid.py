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
import random
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from .grid_constraints import GridConstraints
from .tile_superposition import TileSuperposition


class Grid:
    def __init__(self, rows: int, cols: int, init_superposition: TileSuperposition) -> None:
        """
        Grid of superposed possible tiles, to be solved into a single tile per cell or a contradiction

        Args:
            rows (int): Number of rows in the grid
            cols (int): Number of cells in the grid
            init_superposition (TileSuperposition): Initial superposition before solving
        """
        # Initialized the grid (with a deep copy to avoid having all tiles linked)
        self.superpositions: List[List[TileSuperposition]] = [
            [copy.deepcopy(init_superposition) for j in range(cols)] for i in range(rows)
        ]
        self.rows = rows
        self.cols = cols
        # Initialized the entropy value.
        self.entropy = np.zeros((rows, cols))
        for i in range(self.rows):
            for j in range(self.cols):
                superposition = self.superpositions[i][j]
                self.entropy[i, j] = superposition.get_entropy()
        self.display = np.zeros((self.rows, self.cols))

    def reset(self, init_superposition: TileSuperposition):
        # Initialized the grid (with a deep copy to avoid having all tiles linked)
        self.superpositions: List[List[TileSuperposition]] = [
            [copy.deepcopy(init_superposition) for j in range(self.cols)] for i in range(self.rows)
        ]
        # Initialized the entropy value.
        self.entropy = np.zeros((self.rows, self.cols))
        for i in range(self.rows):
            for j in range(self.cols):
                superposition = self.superpositions[i][j]
                self.entropy[i, j] = superposition.get_entropy()
        self.display = np.zeros((self.rows, self.cols))

    def solve(self, constraints=None, display: bool = False) -> bool:
        """
        Solve the grid. The solver picks a tile at random with the lowest entropy, collapses it
        to a possible tile, then propagates the changes until all tiles are collapsed or a
        contradiction is reached. This process is iteratively repeated until a solution is
        found or a contradiction is reached

        Returns:
            bool: True if no contradiction was found and all tiles are collapsed
        """
        grid_solved = False
        excluded_indexes = set()
        backtrack = False
        last_collapsed = None
        last_valid = None
        num_possibilities = 0
        if display:
            plt.ion()
        step = 0

        # At the beginning, all indexes are valid.
        self.valid_indexes = [
            [set([k for k in range(len(self.superpositions[row][col].tile_list))]) for col in range(self.cols)]
            for row in range(self.rows)
        ]
        grid_possible = True
        # Initial constraint check.
        for row in range(self.rows):
            for col in range(self.cols):
                superposition = self.superpositions[row][col]
                if constraints is not None:
                    previous_count = len(self.superpositions[row][col].tile_list)
                    compatible_indexes, _ = constraints.get_compatible_indexes(superposition, None, (row, col))
                    superposition.select_indexes(compatible_indexes)
                    possible_count = len(self.superpositions[row][col].tile_list)
                    # If the constraints implied a reduction in the number of possibilities
                    # propagate the changes to the rest of the grid
                    if possible_count < previous_count:
                        self.valid_indexes[row][col] = set([k for k in range(possible_count)])
                        grid_possible = self.propagate_changes(row, col, constraints)
                        if not grid_possible:
                            break
        self.apply_changes(constraints)
        if display:
            self.display_entropy()
        if not grid_possible:
            print("Initial constraints not feasible, cannot solve")
            return False

        self.indexes_history = [copy.deepcopy(self.valid_indexes)]
        self.entropy_history = [np.copy(self.entropy)]
        steps = 0

        while not grid_solved and steps < self.rows * self.cols:
            steps += 1
            if not backtrack:
                # Pick a minimum non zero entropy position
                collapse_pos = self.get_min_entropy_position()
                # If no position is found, we have solved the grid.
                if collapse_pos is None:
                    grid_solved = True
                    break
                last_collapsed = collapse_pos
                i, j = collapse_pos
                last_valid = copy.deepcopy(self.valid_indexes)
                excluded_indexes = set()
            else:
                collapse_pos = last_collapsed
                i, j = collapse_pos
                self.valid_indexes = copy.deepcopy(last_valid)

            # Collapse the tile.
            num_possibilities = len(self.valid_indexes[i][j])
            if num_possibilities == 0:
                return False

            if constraints is not None and not backtrack:
                _, constraints_exclude = constraints.get_compatible_indexes(
                    self.superpositions[i][j], self.valid_indexes[i][j], collapse_pos
                )
                excluded_indexes = excluded_indexes.union(constraints_exclude)
            if len(excluded_indexes) == num_possibilities:
                return False
            chosen_index, _ = self.superpositions[i][j].collapse(self.valid_indexes[i][j], excluded_indexes)

            # Set entropy to infinity to rule it out from minimum
            self.entropy[i, j] = np.inf
            # For the chosen collapsed tile, only the selected one is valid.
            self.valid_indexes[i][j] = set([chosen_index])
            grid_possible = self.propagate_changes(i, j, constraints)
            # If we have reached a contradiction, exclude the chosen index and backtrack
            if not grid_possible:
                excluded_indexes.add(chosen_index)
                backtrack = True
                # If we exhausted all possibilities, we are in a contradiction
                if len(excluded_indexes) == num_possibilities:
                    backtrack = False
                    if len(self.indexes_history) <= 1:
                        return False
                    self.indexes_history.pop()
                    self.entropy_history.pop()
                    self.valid_indexes = copy.deepcopy(self.indexes_history[-1])
                    self.entropy = np.copy(self.entropy_history[-1])
                    continue
                continue
            backtrack = False
            # Once all changes have stopped, apply the remaining valid indexes
            self.apply_changes(constraints)
            self.indexes_history.append(copy.deepcopy(self.valid_indexes))
            self.entropy_history.append(np.copy(self.entropy))

            if display:
                self.display_entropy()
            step += 1
        for row in range(self.rows):
            for col in range(self.cols):
                current_tile = self.superpositions[row][col]
                current_tile.select_indexes(self.valid_indexes[row][col])
        return grid_solved

    def get_neighbors(self, i: int, j: int) -> List[int]:
        """
        Get a list of neighbors position, in a convenient order

        Args:
            i (int): Row index of the origin tile
            j (int): Column index of the origin tile

        Returns:
            List[int]: Left, bottom, right and up neighbor (in that order)
        """
        # Left bottom right up
        return [(i, j - 1), (i + 1, j), (i, j + 1), (i - 1, j)]

    def get_min_entropy_position(self) -> Tuple[int, int]:
        """
        Get a position with a minimal entropy. Select one at random if there are several
        tiles with the same entropy

        Returns:
            Tuple[int, int]: Minimal entropy tile position
        """
        # Get current minimal entropy
        min_entropy = self.entropy.min()
        # If all positions are collapsed, we can return.
        if np.isposinf(min_entropy):
            return None
        # In case of several tiles with the same entropy select one at random.
        min_positions = np.where(self.entropy == min_entropy)
        rand_index = random.randint(0, len(min_positions[0]) - 1)
        return min_positions[0][rand_index], min_positions[1][rand_index]

    def propagate_changes(self, i: int, j: int, constraints: GridConstraints) -> bool:
        """
        Propagate current valid indexes from a certain grid position. Updates neighbors
        until no more change is required.

        Args:
            i (int): Row index of the position to start from
            j (int): Column index of the position to start from
            constraints (GridConstraints): Constraints to be applied to the selection

        Returns:
            bool: True if the grid is still consistent after propagation
        """
        # Propagate the changes once
        grid_possible, changed_positions = self.update_position(constraints, i, j)
        # Continue propagating from the changed positions as long as changes are required.
        while grid_possible and len(changed_positions) > 0:
            new_changes = []
            # Iterate over all current changed positions. This is not optimal but simpler
            # to maintain
            for pos in changed_positions:
                grid_possible, neighbor_changes = self.update_position(constraints, pos[0], pos[1])
                # If we reach a contradiction, break.
                if not grid_possible:
                    return False
                # Append induced changes to the list of new changes
                new_changes += neighbor_changes
            # Make them unique to save some time
            changed_positions = set(new_changes)
        return grid_possible

    def apply_changes(self, constraints: GridConstraints) -> None:
        """
        Apply the currently valid indexes to the entropy computation, and count
        collapsed tile types for constraints on count.

        Args:
            constraints (GridConstraints): Current constraints for the grid
        """
        for row in range(self.rows):
            for col in range(self.cols):
                current_tile = self.superpositions[row][col]
                # If we have a fully collapsed tile, set the entropy to inf and update constraints
                if len(self.valid_indexes[row][col]) == 1:
                    self.entropy[row, col] = np.inf
                    selected_index = list(self.valid_indexes[row][col])[0]
                    if constraints is not None:
                        constraints.update_constraints((row, col), current_tile.tile_list[selected_index].identifier)
                # Else just update the entropy
                else:
                    self.entropy[row, col] = current_tile.get_entropy(self.valid_indexes[row][col])

    def update_position(self, constraints: GridConstraints, tile_i: int, tile_j: int) -> Tuple[bool, List[int]]:
        """
        Update all neighbors from a given position, following constraints if not None.
        Retrieve list of neighbors positions that have changed after the tile update
        (number of possible tiles have decreased), and check if the grid is possible

        Args:
            constraints (GridConstraints): Current grid constraints. If None, ignored
            tile_i (int): Row position of the tile to update from
            tile_j (int): Column position of the tile to update from

        Returns:
            Tuple[bool, List[int]]: grid_possible (False if we reach a contradiction),
                                    changed_indexes (list of neighbors that were changed)
        """
        changed_positions = []
        grid_possible = True
        for idx, pos in enumerate(self.get_neighbors(tile_i, tile_j)):
            # Skip out of bounds positions
            if pos[0] < 0 or pos[0] >= self.rows or pos[1] < 0 or pos[1] >= self.cols:
                continue
            good_indexes = set()
            neighbor_superposition = self.superpositions[pos[0]][pos[1]]
            # The possible tiles for the neighbors is the union of all tiles that are compatible
            # with each of the start tiles
            for valid_index in self.valid_indexes[tile_i][tile_j]:
                valid_tile = self.superpositions[tile_i][tile_j].tile_list[valid_index]
                good_indexes = set.union(good_indexes, neighbor_superposition.get_compatible_indexes(valid_tile, idx))
            # We intersect this with the indexes that were already selected
            remaining_tiles = self.valid_indexes[pos[0]][pos[1]].intersection(good_indexes)
            if constraints is not None:
                compatible_indexes, _ = constraints.get_compatible_indexes(
                    neighbor_superposition, self.valid_indexes[pos[0]][pos[1]], pos
                )
                remaining_tiles.intersection_update(compatible_indexes)
            # If we do not have any remaining tiles, this is a contradiction
            if len(remaining_tiles) == 0:
                grid_possible = False
                break
            # If we have reduced the number of remaining tiles, this is a change and
            # we will need a new update.
            if len(remaining_tiles) < len(self.valid_indexes[pos[0]][pos[1]]):
                changed_positions.append(pos)
                self.valid_indexes[pos[0]][pos[1]] = remaining_tiles
        return grid_possible, changed_positions

    def display_entropy(self):
        self.display[:] = 0
        for i in range(self.rows):
            for j in range(self.cols):
                self.display[i, j] = len(self.valid_indexes[i][j])
        plt.imshow(self.display)
        plt.draw()
        plt.pause(0.01)
