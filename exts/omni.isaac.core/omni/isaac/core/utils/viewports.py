# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
from typing import Any, List
import numpy as np

# omniverse
import carb
import omni
from pxr import UsdGeom, Usd, Gf, Sdf
import omni.kit.app

# isaacsim
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import set_prim_hide_in_stage_window, set_prim_no_delete


def set_camera_view(
    eye: np.array, target: np.array, camera_prim_path: str = "/OmniverseKit_Persp", viewport_api=None
) -> None:
    """Set the location and target for a camera prim in the stage given its path

    Args:
        eye (np.ndarray): Location of camera.
        target (np.ndarray,): Location of camera target.
        camera_prim_path (str, optional): Path to camera prim being set. Defaults to "/OmniverseKit_Persp".
    """
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        from omni.kit.viewport.utility import get_active_viewport

        if viewport_api is None:
            viewport_api = get_active_viewport()
    except ImportError:
        carb.log_warn("omni.kit.viewport.utility needs to be enabled before using this function")
        return

    if viewport_api is None:
        carb.log_warn("could not get active viewport, cannot set camera view")
        return

    camera_position = np.asarray(eye, dtype=np.double)
    camera_target = np.asarray(target, dtype=np.double)
    prim = viewport_api.stage.GetPrimAtPath(camera_prim_path)

    coi_prop = prim.GetProperty("omni:kit:centerOfInterest")
    if not coi_prop or not coi_prop.IsValid():
        prim.CreateAttribute(
            "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, True, Sdf.VariabilityUniform
        ).Set(Gf.Vec3d(0, 0, -10))
    camera_state = ViewportCameraState(camera_prim_path, viewport_api)
    camera_state.set_position_world(Gf.Vec3d(camera_position[0], camera_position[1], camera_position[2]), True)
    camera_state.set_target_world(Gf.Vec3d(camera_target[0], camera_target[1], camera_target[2]), True)
    return


def get_viewport_names(usd_context_name: str = None) -> List[str]:
    """Get list of all viewport names

    Args:
        usd_context_name (str, optional):  usd context to use. Defaults to None.

    Returns:
        List[str]: List of viewport names
    """
    viewport_names = []
    try:
        from omni.kit.viewport.window import get_viewport_window_instances

        for window in get_viewport_window_instances(usd_context_name):
            viewport_names.append(window.title)
        return viewport_names
    except ImportError:
        pass

    try:
        import omni.kit.viewport_legacy as vp_legacy

        vp_iface = vp_legacy.get_viewport_interface()
        for viewport_handle in vp_iface.get_instance_list():
            if usd_context_name and (
                usd_context_name != vp_iface.get_viewport_window(viewport_handle).get_usd_context_name()
            ):
                continue
            viewport_names.append(vp_iface.get_viewport_window_name(viewport_handle))
        return viewport_names
    except ImportError:
        pass

    return viewport_names


def get_id_from_index(index):
    """Get the viewport id for a given index. 
    This function was added for backwards compatibility for VP2 as viewport IDs are not the same as the viewport index

    Args:
        index (_type_): viewport index to retrieve ID for

    Returns:
        viewport id : Returns None if window index was not found
    """
    try:
        from omni.kit.viewport.window import get_viewport_window_instances

        instances = []
        for window in get_viewport_window_instances(None):
            instances.append(window)

        if len(instances) > index:
            return instances[index].viewport_api.id
        else:
            return None
    except ImportError:
        pass

    try:
        import omni.kit.viewport_legacy as vp_legacy

        vp_iface = vp_legacy.get_viewport_interface()
        instances = vp_iface.get_instance_list()
        if len(instances) > index:
            return vp_iface.get_viewport_window(instances[index]).get_id()
        else:
            return None
    except ImportError:
        pass

    return None


def get_window_from_id(id, usd_context_name: str = None):
    """Find window that matches a given viewport id

    Args:
        id (_type_): Viewport ID to get window for
        usd_context_name (str, optional): usd context to use. Defaults to None.

    Returns:
        Window : Returns None if window with matching ID was not found
    """
    if id is None:
        return None

    try:
        from omni.kit.viewport.window import get_viewport_window_instances

        for window in get_viewport_window_instances(usd_context_name):
            if window.viewport_api.id == id:
                return window
    except ImportError:
        pass

    try:
        import omni.kit.viewport_legacy as vp_legacy

        vp_iface = vp_legacy.get_viewport_interface()
        for viewport_handle in vp_iface.get_instance_list():
            if usd_context_name and (
                usd_context_name != vp_iface.get_viewport_window(viewport_handle).get_usd_context_name()
            ):
                continue
            if vp_iface.get_viewport_window(viewport_handle).get_id() == id:
                from omni.kit.viewport.utility.legacy_viewport_window import LegacyViewportWindow

                return LegacyViewportWindow(vp_iface.get_viewport_window_name(viewport_handle))
    except ImportError:
        pass

    return None


def destroy_all_viewports(usd_context_name: str = None, destroy_main_viewport=True):
    """Destroys all viewport windows

    Args:
        usd_context_name (str, optional): usd context to use. Defaults to None.
        destroy_main_viewport (bool, optional): set to true to not destroy the default viewport. Defaults to False.
    """
    from omni.kit.viewport.window import get_viewport_window_instances

    for window in get_viewport_window_instances(usd_context_name):
        if window:
            if window.title == "Viewport" and not destroy_main_viewport:
                continue
            window.destroy()


def add_aov_to_viewport(viewport_api, aov_name: str):
    if hasattr(viewport_api, "legacy_window"):
        return viewport_api.legacy_window.add_aov(aov_name)

    from pxr import Usd, UsdRender

    stage = viewport_api.stage
    render_product_path = viewport_api.render_product_path
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = stage.GetPrimAtPath(render_product_path)
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_var_prim_path = Sdf.Path(f"/Render/Vars/{aov_name}")
        render_var_prim = stage.GetPrimAtPath(render_var_prim_path)
        if not render_var_prim:
            render_var_prim = stage.DefinePrim(render_var_prim_path)
        if not render_var_prim:
            raise RuntimeError(f'Cannot create renderVar "{render_var_prim_path}"')
        render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(aov_name)
        render_prod_var_rel = render_prod_prim.GetRelationship("orderedVars")
        if not render_prod_var_rel:
            render_prod_prim.CreateRelationship("orderedVars")
        if not render_prod_var_rel:
            raise RuntimeError(f'cannot set orderedVars relationship for renderProduct "{render_product_path}"')
        render_prod_var_rel.AddTarget(render_var_prim_path)

        set_prim_hide_in_stage_window(render_var_prim, True)
        set_prim_no_delete(render_var_prim, True)

    return True


def get_intrinsics_matrix(viewport_api: Any) -> np.ndarray:
    """Get intrinsic matrix for the camera attached to a specific viewport

    Args:
        viewport (Any): Handle to viewport api

    Returns:
        np.ndarray: the intrinsic matrix associated with the specified viewport
                The following image convention is assumed:
                    +x should point to the right in the image
                    +y should point down in the image
    """
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(viewport_api.get_active_camera())
    focal_length = prim.GetAttribute("focalLength").Get()
    horizontal_aperture = prim.GetAttribute("horizontalAperture").Get()
    vertical_aperture = prim.GetAttribute("verticalAperture").Get()
    (width, height) = viewport_api.get_texture_resolution()
    fx = width * focal_length / horizontal_aperture
    fy = height * focal_length / vertical_aperture
    cx = width * 0.5
    cy = height * 0.5
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def set_intrinsics_matrix(viewport_api: Any, intrinsics_matrix: np.ndarray, focal_length: float = 1.0) -> None:
    """Set intrinsic matrix for the camera attached to a specific viewport

    Note:
        We assume cx and cy are centered in the camera
        horizontal_aperture_offset and vertical_aperture_offset are computed and set on the camera prim but are not used

    Args:
        viewport (Any): Handle to viewport api
        intrinsics_matrix (np.ndarray): A 3x3 intrinsic matrix
        focal_length (float, optional): Default focal length to use when computing aperture values. Defaults to 1.0.

    Raises:
        ValueError: If intrinsic matrix is not a 3x3 matrix.
        ValueError: If camera prim is not valid
    """

    if intrinsics_matrix.shape != (3, 3):
        raise ValueError("intrinsics_matrix must be 3x3")

    fx = intrinsics_matrix[0, 0]
    fy = intrinsics_matrix[1, 1]
    cx = intrinsics_matrix[0, 2]
    cy = intrinsics_matrix[1, 2]

    stage = get_current_stage()
    prim = UsdGeom.Camera(stage.GetPrimAtPath(viewport_api.get_active_camera()))
    print(prim)
    if prim is None:
        raise ValueError("Viewport does not have a valid camera prim")

    (width, height) = viewport_api.get_texture_resolution()

    horizontal_aperture = width * focal_length / fx
    vertical_aperture = height * focal_length / fy
    # TODO: this should be set_attr_val
    # We have to do it this way because the camera might be on a different layer (default cameras are on session layer),
    # and this is the simplest way to set the property on the right layer.
    omni.usd.utils.set_prop_val(prim.GetFocalLengthAttr(), focal_length)
    omni.usd.utils.set_prop_val(prim.GetHorizontalApertureAttr(), horizontal_aperture)
    omni.usd.utils.set_prop_val(prim.GetVerticalApertureAttr(), vertical_aperture)
    omni.usd.utils.set_prop_val(prim.GetHorizontalApertureOffsetAttr(), (cx - width / 2) / fx)
    omni.usd.utils.set_prop_val(prim.GetVerticalApertureOffsetAttr(), (cy - height / 2) / fy)


def backproject_depth(depth_image: np.array, viewport_api: Any, max_clip_depth: float) -> np.array:
    """Backproject depth image to image space

    Args:
        depth_image (np.array): Depth image buffer
        viewport_api (Any): Handle to viewport api
        max_clip_depth (float): Depth values larger than this will be clipped

    Returns:
        np.array: [description]
    """

    intrinsics_matrix = get_intrinsics_matrix(viewport_api)
    fx = intrinsics_matrix[0][0]
    fy = intrinsics_matrix[1][1]
    cx = intrinsics_matrix[0][2]
    cy = intrinsics_matrix[1][2]
    height = depth_image.shape[0]
    width = depth_image.shape[1]
    input_x = np.arange(width)
    input_y = np.arange(height)
    input_x, input_y = np.meshgrid(input_x, input_y)
    input_x = input_x.flatten()
    input_y = input_y.flatten()
    input_z = depth_image.flatten()
    input_z[input_z > max_clip_depth] = 0
    output_x = (input_x * input_z - cx * input_z) / fx
    output_y = (input_y * input_z - cy * input_z) / fy
    raw_pc = np.stack([output_x, output_y, input_z], -1).reshape([height * width, 3])
    return raw_pc


def project_depth_to_worldspace(depth_image: np.array, viewport_api: Any, max_clip_depth: float) -> List[carb.Float3]:
    """Project depth image to world space

    Args:
        depth_image (np.array): Depth image buffer
        viewport_api (Any): Handle to viewport api
        max_clip_depth (float): Depth values larger than this will be clipped

    Returns:
        List[carb.Float3]: List of points from depth in world space
    """
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(viewport_api.get_active_camera())
    prim_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode())
    units_per_meter = 1.0 / UsdGeom.GetStageMetersPerUnit(stage)

    depth_data = depth_image * units_per_meter
    depth_data = -np.clip(depth_data, 0, max_clip_depth)

    pc = backproject_depth(depth_data, viewport_api, max_clip_depth)
    points = []
    for pts in pc:
        p = prim_tf.Transform(Gf.Vec3d(-pts[0], pts[1], pts[2]))
        points.append(carb.Float3(p[0], p[1], p[2]))

    return points
