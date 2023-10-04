# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Tuple, Sequence
from pxr import Usd
from omni.isaac.core.utils.types import XFormPrimState
from omni.isaac.core.materials import VisualMaterial
import numpy as np


class _SinglePrimWrapper(object):
    def __init__(self, view) -> None:
        self._prim_view = view
        return

    def initialize(self, physics_sim_view=None) -> None:
        self._prim_view.initialize(physics_sim_view=physics_sim_view)
        return

    @property
    def prim_path(self) -> str:
        """
        Returns:
            str: prim path in the stage.
        """
        return self._prim_view.prim_paths[0]

    @property
    def name(self) -> Optional[str]:
        """
        Returns:
            str: name given to the prim when instantiating it. Otherwise None.
        """
        return self._prim_view.name

    @property
    def prim(self) -> Usd.Prim:
        """
        Returns:
            Usd.Prim: USD Prim object that this object holds.
        """
        return self._prim_view.prims[0]

    @property
    def non_root_articulation_link(self) -> bool:
        """_summary_

        Returns:
            bool: _description_
        """
        return self._prim_view._non_root_link

    def set_visibility(self, visible: bool) -> None:
        """Sets the visibility of the prim in stage.

        Args:
            visible (bool): flag to set the visibility of the usd prim in stage.
        """
        self._prim_view.set_visibilities(
            self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)
        )
        return

    def get_visibility(self) -> bool:
        """
        Returns:
            bool: true if the prim is visible in stage. false otherwise.
        """
        return self._prim_view.get_visibilities()[0]

    def post_reset(self) -> None:
        """Resets the prim to its default state (position and orientation).
        """
        self._prim_view.post_reset()
        return

    def get_default_state(self) -> XFormPrimState:
        """
        Returns:
            XFormPrimState: returns the default state of the prim (position and orientation) that is used after each reset.
        """
        view_default_state = self._prim_view.get_default_state()
        default_state = self._view_state_conversion(view_default_state)
        return default_state

    def set_default_state(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Sets the default state of the prim (position and orientation), that will be used after each reset.

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        if position is not None:
            position = self._backend_utils.convert(position, device=self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        self._prim_view.set_default_state(positions=position, orientations=orientation)
        return

    def apply_visual_material(self, visual_material: VisualMaterial, weaker_than_descendants: bool = False) -> None:
        """Used to apply visual material to the held prim and optionally its descendants.

        Args:
            visual_material (VisualMaterial): visual material to be applied to the held prim. Currently supports
                                              PreviewSurface, OmniPBR and OmniGlass.
            weaker_than_descendants (bool, optional): True if the material shouldn't override the descendants  
                                                      materials, otherwise False. Defaults to False.
        """
        self._prim_view.apply_visual_materials(
            visual_materials=[visual_material], weaker_than_descendants=[weaker_than_descendants]
        )
        return

    def get_applied_visual_material(self) -> VisualMaterial:
        """Returns the current applied visual material in case it was applied using apply_visual_material OR
           it's one of the following materials that was already applied before: PreviewSurface, OmniPBR and OmniGlass.

        Returns:
            VisualMaterial: the current applied visual material if its type is currently supported.
        """
        return self._prim_view.get_applied_visual_materials()[0]

    def is_visual_material_applied(self) -> bool:
        """
        Returns:
            bool: True if there is a visual material applied. False otherwise.
        """
        return self._prim_view.is_visual_material_applied()[0]

    def set_world_pose(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Sets prim's pose with respect to the world's frame.

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        if position is not None:
            position = self._backend_utils.convert(position, device=self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        self._prim_view.set_world_poses(positions=position, orientations=orientation)
        return

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Gets prim's pose with respect to the world's frame.

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is position in the world frame of the prim. shape is (3, ). 
                                           second index is quaternion orientation in the world frame of the prim.
                                           quaternion is scalar-first (w, x, y, z). shape is (4, ).
        """
        positions, orientations = self._prim_view.get_world_poses()
        return positions[0], orientations[0]

    def get_local_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Gets prim's pose with respect to the local frame (the prim's parent frame).

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is position in the local frame of the prim. shape is (3, ). 
                                           second index is quaternion orientation in the local frame of the prim.
                                           quaternion is scalar-first (w, x, y, z). shape is (4, ).
        """
        translations, orientations = self._prim_view.get_local_poses()
        return translations[0], orientations[0]

    def set_local_pose(
        self, translation: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Sets prim's pose with respect to the local frame (the prim's parent frame).

        Args:
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                          (with respect to its parent prim). shape is (3, ).
                                                          Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the local frame of the prim. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        if translation is not None:
            translation = self._backend_utils.convert(translation, device=self._device)
            translation = self._backend_utils.expand_dims(translation, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        self._prim_view.set_local_poses(translations=translation, orientations=orientation)
        return

    def get_world_scale(self) -> np.ndarray:
        """Gets prim's scale with respect to the world's frame.

        Returns:
            np.ndarray: scale applied to the prim's dimensions in the world frame. shape is (3, ).
        """
        return self._prim_view.get_world_scales()[0]

    def set_local_scale(self, scale: Optional[Sequence[float]]) -> None:
        """Sets prim's scale with respect to the local frame (the prim's parent frame).

        Args:
            scale (Optional[Sequence[float]]): scale to be applied to the prim's dimensions. shape is (3, ).
                                          Defaults to None, which means left unchanged.
        """
        scale = self._backend_utils.convert(scale, device=self._device)
        scale = self._backend_utils.expand_dims(scale, 0)
        self._prim_view.set_local_scales(scales=scale)
        return

    def get_local_scale(self) -> np.ndarray:
        """Gets prim's scale with respect to the local frame (the parent's frame).

        Returns:
            np.ndarray: scale applied to the prim's dimensions in the local frame. shape is (3, ).
        """
        return self._prim_view.get_local_scales()[0]

    def is_valid(self) -> bool:
        """
        Returns:
            bool: True is the current prim path corresponds to a valid prim in stage. False otherwise.
        """
        return self._prim_view.is_valid()

    def _view_state_conversion(self, view_state):
        # TODO: a temp function
        position = None
        orientation = None
        if view_state.positions is not None:
            position = view_state.positions[0]
        if view_state.orientations is not None:
            orientation = view_state.orientations[0]
        return XFormPrimState(position=position, orientation=orientation)
