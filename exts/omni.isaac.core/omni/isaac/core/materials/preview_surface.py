# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.materials import VisualMaterial
import omni.kit.app
import carb
from pxr import Gf, UsdShade, Sdf
from typing import Optional
import numpy as np


class PreviewSurface(VisualMaterial):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "preview_surface".
            shader (Optional[UsdShade.Shader], optional): [description]. Defaults to None.
            color (Optional[np.ndarray], optional): [description]. Defaults to None.
            roughness (Optional[float], optional): [description]. Defaults to None.
            metallic (Optional[float], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "preview_surface",
        shader: Optional[UsdShade.Shader] = None,
        color: Optional[np.ndarray] = None,
        roughness: Optional[float] = None,
        metallic: Optional[float] = None,
    ) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(prim_path).IsValid():
            carb.log_info("Material Prim already defined at path: {}".format(prim_path))
            material = UsdShade.Material(stage.GetPrimAtPath(prim_path))
        else:
            material = UsdShade.Material.Define(stage, prim_path)

        if shader is None:
            if stage.GetPrimAtPath(f"{prim_path}/shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/shader"))
            elif stage.GetPrimAtPath(f"{prim_path}/Shader").IsValid():
                carb.log_info("Shader Prim already defined at path: {}".format(f"{prim_path}/shader"))
                shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/Shader"))
            else:
                shader = UsdShade.Shader.Define(stage, f"{prim_path}/shader")
        VisualMaterial.__init__(
            self,
            prim_path=prim_path,
            prim=stage.GetPrimAtPath(prim_path),
            shaders_list=[shader],
            material=material,
            name=name,
        )
        shader.CreateIdAttr("UsdPreviewSurface")
        if color is not None:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*color.tolist()))
        if roughness is not None:
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        if metallic is not None:
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        material.CreateSurfaceOutput().ConnectToSource(shader, "surface")
        return

    def set_color(self, color: np.ndarray) -> None:
        """[summary]

        Args:
            color (np.ndarray): [description]
        """
        if self.shaders_list[0].GetInput("diffuseColor").Get() is None:
            self.shaders_list[0].CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*color.tolist()))
        else:
            self.shaders_list[0].GetInput("diffuseColor").Set(Gf.Vec3f(*color.tolist()))
        return

    def get_color(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        if self.shaders_list[0].GetInput("diffuseColor").Get() is None:
            carb.log_warn("A color attribute is not set yet")
            return None
        else:
            return np.array(self.shaders_list[0].GetInput("diffuseColor").Get())

    def set_roughness(self, roughness: float) -> None:
        """[summary]

        Args:
            roughness (float): [description]
        """
        if self.shaders_list[0].GetInput("roughness").Get() is None:
            self.shaders_list[0].CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        else:
            self.shaders_list[0].GetInput("roughness").Set(roughness)
        return

    def get_roughness(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        if self.shaders_list[0].GetInput("roughness").Get() is None:
            carb.log_warn("A roughness attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("roughness").Get()

    def set_metallic(self, metallic: float) -> None:
        """[summary]

        Args:
            metallic (float): [description]
        """
        if self.shaders_list[0].GetInput("metallic").Get() is None:
            self.shaders_list[0].CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        else:
            self.shaders_list[0].GetInput("metallic").Set(metallic)
        return

    def get_metallic(self) -> float:
        """[summary]

        Returns:
            float: [description]
        """
        if self.shaders_list[0].GetInput("metallic").Get() is None:
            carb.log_warn("A metallic attribute is not set yet")
            return None
        else:
            return self.shaders_list[0].GetInput("metallic").Get()
