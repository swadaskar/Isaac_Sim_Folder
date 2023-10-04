# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
from omni.kit.viewport.utility import get_viewport_from_window_name, get_active_viewport
from omni.isaac.core.utils.render_product import get_camera_prim_path, get_resolution


class OgnIsaacReadCameraInfo:
    """
    Isaac Sim Camera Info Node
    """

    @staticmethod
    def compute(db) -> bool:
        stage = omni.usd.get_context().get_stage()

        if db.inputs.viewport:
            db.log_warn(
                "viewport input is deprecated, please use renderProductPath and the IsaacGetViewportRenderProduct to get a viewports render product path"
            )
            viewport_name = db.inputs.viewport
            if viewport_name:
                viewport_api = get_viewport_from_window_name(viewport_name)
            else:
                viewport_api = get_active_viewport()
            if viewport_api:
                camera_path = viewport_api.get_active_camera()
                (db.outputs.width, db.outputs.height) = viewport_api.get_texture_resolution()
            else:
                db.log_warn("viewport not found")
                return False
        elif db.inputs.renderProductPath:
            render_product_path = db.inputs.renderProductPath
            camera_path = get_camera_prim_path(render_product_path)
            (db.outputs.width, db.outputs.height) = get_resolution(render_product_path)
        else:
            db.log_warn(f"renderProductPath must be specified {db.inputs.renderProductPath}")
            return False

        camera = stage.GetPrimAtPath(camera_path)
        if not camera:
            db.log_error(f"camera at path {camera_path} not found")
            return False

        db.outputs.focalLength = camera.GetAttribute("focalLength").Get()

        db.outputs.horizontalAperture = camera.GetAttribute("horizontalAperture").Get()
        db.outputs.verticalAperture = camera.GetAttribute("verticalAperture").Get()

        db.outputs.horizontalOffset = camera.GetAttribute("horizontalApertureOffset").Get()
        db.outputs.verticalOffset = camera.GetAttribute("verticalApertureOffset").Get()

        projection_type = camera.GetAttribute("cameraProjectionType").Get()
        if projection_type is None:
            projection_type = "pinhole"

        db.outputs.projectionType = projection_type
        db.outputs.cameraFisheyeParams = [0.0] * 19
        if projection_type is not "pinhole":
            db.outputs.cameraFisheyeParams[0] = camera.GetAttribute("fthetaWidth").Get()
            db.outputs.cameraFisheyeParams[1] = camera.GetAttribute("fthetaHeight").Get()
            db.outputs.cameraFisheyeParams[2] = camera.GetAttribute("fthetaCx").Get()
            db.outputs.cameraFisheyeParams[3] = camera.GetAttribute("fthetaCy").Get()
            db.outputs.cameraFisheyeParams[4] = camera.GetAttribute("fthetaMaxFov").Get()
            db.outputs.cameraFisheyeParams[5] = camera.GetAttribute("fthetaPolyA").Get()
            db.outputs.cameraFisheyeParams[6] = camera.GetAttribute("fthetaPolyB").Get()
            db.outputs.cameraFisheyeParams[7] = camera.GetAttribute("fthetaPolyC").Get()
            db.outputs.cameraFisheyeParams[8] = camera.GetAttribute("fthetaPolyD").Get()
            db.outputs.cameraFisheyeParams[9] = camera.GetAttribute("fthetaPolyE").Get()
            db.outputs.cameraFisheyeParams[10] = camera.GetAttribute("fthetaPolyF").Get()
            db.outputs.cameraFisheyeParams[11] = camera.GetAttribute("p0").Get()
            db.outputs.cameraFisheyeParams[12] = camera.GetAttribute("p1").Get()
            db.outputs.cameraFisheyeParams[13] = camera.GetAttribute("s0").Get()
            db.outputs.cameraFisheyeParams[14] = camera.GetAttribute("s1").Get()
            db.outputs.cameraFisheyeParams[15] = camera.GetAttribute("s2").Get()
            db.outputs.cameraFisheyeParams[16] = camera.GetAttribute("s3").Get()
            db.outputs.cameraFisheyeParams[17] = camera.GetAttribute("fisheyeResolutionBudget").Get()
            db.outputs.cameraFisheyeParams[18] = camera.GetAttribute("fisheyeFrontFaceResolutionScale").Get()

        return True
