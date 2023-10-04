# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from pxr import Usd, UsdShade
from typing import List


class VisualMaterial(object):
    """[summary]

        Args:
            name (str): [description]
            prim_path (str): [description]
            prim (Usd.Prim): [description]
            shaders_list (list[UsdShade.Shader]): [description]
            material (UsdShade.Material): [description]
        """

    def __init__(
        self,
        name: str,
        prim_path: str,
        prim: Usd.Prim,
        shaders_list: List[UsdShade.Shader],
        material: UsdShade.Material,
    ) -> None:
        self._shaders_list = shaders_list
        self._material = material
        self._name = name
        self._prim_path = prim_path
        self._prim = prim
        return

    @property
    def material(self) -> UsdShade.Material:
        """[summary]

        Returns:
            UsdShade.Material: [description]
        """
        return self._material

    @property
    def shaders_list(self) -> List[UsdShade.Shader]:
        """[summary]

        Returns:
            [type]: [description]
        """
        return self._shaders_list

    @property
    def name(self) -> str:
        """[summary]

        Returns:
            str: [description]
        """
        return self._name

    @property
    def prim_path(self) -> str:
        """[summary]

        Returns:
            str: [description]
        """
        return self._prim_path

    @property
    def prim(self) -> Usd.Prim:
        """[summary]

        Returns:
            Usd.Prim: [description]
        """
        return self._prim
