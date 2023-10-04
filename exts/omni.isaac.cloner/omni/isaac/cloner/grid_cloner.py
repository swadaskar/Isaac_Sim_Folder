from typing import List

from omni.isaac.cloner import Cloner
import omni.usd
from pxr import Gf, UsdGeom

import numpy as np


class GridCloner(Cloner):

    """ This is a specialized Cloner class that will automatically generate clones in a grid fashion. """

    def __init__(self, spacing: float, num_per_row: int = -1):
        """ 
        Args:
            spacing (float): Spacing between clones.
            num_per_row (int): Number of clones to place in a row. Defaults to sqrt(num_clones).
        """
        self._spacing = spacing
        self._num_per_row = num_per_row

        Cloner.__init__(self)

    def clone(
        self,
        source_prim_path: str,
        prim_paths: List[str],
        position_offsets: np.ndarray = None,
        orientation_offsets: np.ndarray = None,
        replicate_physics: bool = False,
        base_env_path: str = None,
        root_path: str = None,
    ):

        """ Creates clones in a grid fashion. Positions of clones are computed automatically.

        Args:
            source_prim_path (str): Path of source object.
            prim_paths (List[str]): List of destination paths.
            position_offsets (np.ndarray): Positions to be applied as local translations on top of computed clone position.
                                           Defaults to None, no offset will be applied.
            orientation_offsets (np.ndarray): Orientations to be applied as local rotations for each clone.
                                           Defaults to None, no offset will be applied.
            replicate_physics (bool): Uses omni.physics replication. This will replicate physics properties directly for paths beginning with root_path and skip physics parsing for anything under the base_env_path.
            base_env_path (str): Path to namespace for all environments. Required if replicate_physics=True and define_base_env() not called.
            root_path (str): Prefix path for each environment. Required if replicate_physics=True and generate_paths() not called.

        Returns:
            positions (List): Computed positions of all clones.
        """

        num_clones = len(prim_paths)

        self._num_per_row = int(np.sqrt(num_clones)) if self._num_per_row == -1 else self._num_per_row
        num_rows = np.ceil(num_clones / self._num_per_row)
        num_cols = np.ceil(num_clones / num_rows)

        row_offset = 0.5 * self._spacing * (num_rows - 1)
        col_offset = 0.5 * self._spacing * (num_cols - 1)

        stage = omni.usd.get_context().get_stage()

        positions = []
        orientations = []

        for i in range(num_clones):
            # compute transform
            row = i // num_cols
            col = i % num_cols
            x = row_offset - row * self._spacing
            y = col * self._spacing - col_offset

            up_axis = UsdGeom.GetStageUpAxis(stage)
            position = [x, y, 0] if up_axis == UsdGeom.Tokens.z else [x, 0, y]
            orientation = Gf.Quatd.GetIdentity()

            if position_offsets is not None:
                translation = position_offsets[i] + position
            else:
                translation = position

            if orientation_offsets is not None:
                orientation = (
                    Gf.Quatd(orientation_offsets[i][0].item(), Gf.Vec3d(orientation_offsets[i][1:].tolist()))
                    * orientation
                )

            else:
                orientation = [
                    orientation.GetReal(),
                    orientation.GetImaginary()[0],
                    orientation.GetImaginary()[1],
                    orientation.GetImaginary()[2],
                ]

            positions.append(translation)
            orientations.append(orientation)

        super().clone(
            source_prim_path=source_prim_path,
            prim_paths=prim_paths,
            positions=positions,
            orientations=orientations,
            replicate_physics=replicate_physics,
            base_env_path=base_env_path,
            root_path=root_path,
        )

        return positions
