# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from pxr import Usd, Sdf, UsdRender, Gf
from omni.isaac.core.utils.prims import set_prim_hide_in_stage_window, set_prim_no_delete
from omni.isaac.core.utils.stage import get_current_stage
from typing import Tuple
import omni.usd


def create_hydra_texture(resolution: Tuple[int], camera_prim_path: str):
    """Create a hydra texture and return the texture and the render product path

    Args:
        resolution (Tuple[int]): Resolution used to create the render product
        camera_prim_path (str): Camera prim to attach to render product

    Returns:
        hydra_texture, str: returns the hydratexture and render product prim path
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        factory = omni.hydratexture.acquire_hydra_texture_factory_interface()
        render_product_path = omni.usd.get_stage_next_free_path(stage, "/Render/RenderProduct_Isaac", False)
        name = render_product_path.split("/Render/RenderProduct_")[-1]
        texture = factory.create_hydra_texture(name, resolution[0], resolution[1], "", camera_prim_path, "rtx")
        return texture, render_product_path


def add_aov(render_product_path: str, aov_name: str):
    """Adds an AOV/Render Var to an existing render product

    Args:
        render_product_path (str): path to the render product prim
        aov_name (str): Name of the render var we want to add to this render product

    Raises:
        RuntimeError: If the render product path is invalid
        RuntimeError: If the renderVar could not be created
        RuntimeError: If the renderVar could not be added to the render product

    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_var_prim_path = Sdf.Path(f"/Render/Vars/{aov_name}")
        render_var_prim = stage.GetPrimAtPath(render_var_prim_path)
        if not render_var_prim:
            render_var_prim = stage.DefinePrim(render_var_prim_path)
        if not render_var_prim:
            raise RuntimeError(f'Cannot create renderVar "{render_var_prim_path}"')
        render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(aov_name)
        render_prod_var_rel = render_prod_prim.GetOrderedVarsRel()
        if not render_prod_var_rel:
            render_prod_prim.CreateOrderedVarsRel()
        if not render_prod_var_rel:
            raise RuntimeError(f'cannot set orderedVars relationship for renderProduct "{render_product_path}"')
        render_prod_var_rel.AddTarget(render_var_prim_path)

        set_prim_hide_in_stage_window(render_var_prim, True)
        set_prim_no_delete(render_var_prim, True)


def get_camera_prim_path(render_product_path: str):
    """Get the current camera for a render product

    Args:
        render_product_path (str): path to the render product prim

    Raises:
        RuntimeError: If the render product path is invalid

    Returns:
        str : Path to the camera prim attached to this render product
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        return render_prod_prim.GetCameraRel().GetTargets()[0]


def set_camera_prim_path(render_product_path: str, camera_prim_path: str):
    """Sets the camera prim path for a render product

    Args:
        render_product_path (str):  path to the render product prim
        camera_prim_path (str):  path to the camera prim

    Raises:
        RuntimeError: If the render product path is invalid
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_prod_prim.GetCameraRel().SetTargets([camera_prim_path])


def get_resolution(render_product_path: str):
    """Get resolution for a render product

    Args:
        render_product_path (str): path to the render product prim

    Raises:
        RuntimeError: If the render product path is invalid

    Returns:
        Tuple[int]: (width,height)
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        return render_prod_prim.GetResolutionAttr().Get()


def set_resolution(render_product_path: str, resolution: Tuple[int]):
    """Set resolution for a render product

    Args:
        render_product_path (str): path to the render product prim
        resolution (Tuple[float]): width,height for render product

    Raises:
        RuntimeError: If the render product path is invalid
    """
    stage = get_current_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(resolution[0], resolution[1]))
