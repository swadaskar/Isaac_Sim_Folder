# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from omni.isaac.kit import SimulationApp
import sys

CAMERA_STAGE_PATH = "/Camera"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)
import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage, extensions, nucleus
from pxr import Gf, UsdGeom, Usd
from omni.kit.viewport.utility import get_active_viewport
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils.render_product import set_camera_prim_path
import omni.replicator.core as rep
import warp as wp
import omni.syntheticdata._syntheticdata as sd
import numpy as np

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros_bridge")

simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# Creating a Camera prim
camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim)
xform_api.SetTranslate(Gf.Vec3d(-1, 5, 1))
xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim.GetHorizontalApertureAttr().Set(21)
camera_prim.GetVerticalApertureAttr().Set(16)
camera_prim.GetProjectionAttr().Set("perspective")
camera_prim.GetFocalLengthAttr().Set(24)
camera_prim.GetFocusDistanceAttr().Set(400)

simulation_app.update()

# grab our render product and directly set the camera prim
render_product_path = get_active_viewport().get_render_product_path()
set_camera_prim_path(render_product_path, CAMERA_STAGE_PATH)

# GPU Noise Kernel for illustrative purposes
@wp.kernel
def image_gaussian_noise_warp(
    data_in: wp.array(dtype=wp.uint8, ndim=3),
    data_out: wp.array(dtype=wp.uint8, ndim=3),
    kernel_seed: int,
    sigma: float = 25.0,
):
    i, j = wp.tid()
    state = wp.rand_init(kernel_seed, wp.tid())
    data_out[i, j, 0] = wp.uint8(wp.int32(data_in[i, j, 0]) + wp.int32(sigma * wp.randn(state)))
    data_out[i, j, 1] = wp.uint8(wp.int32(data_in[i, j, 1]) + wp.int32(sigma * wp.randn(state)))
    data_out[i, j, 2] = wp.uint8(wp.int32(data_in[i, j, 2]) + wp.int32(sigma * wp.randn(state)))


# CPU noise kernel
def image_gaussian_noise_np(data_in: np.ndarray, kernel_seed, sigma: float = 25.0):
    np.random.seed(kernel_seed)
    return data_in + sigma * np.random.randn(*data_in.shape)


# get render car name for rgb data
rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
rgba_to_rgb_annotator = f"{rv_rgb}IsaacConvertRGBAToRGB"

# register new augmented annotator that adds noise to rgb
# the image_gaussian_noise_warp variable can be replaced with image_gaussian_noise_np to use the cpu version
rep.annotators.register(
    name="rgb_gaussian_noise",
    annotator=rep.annotators.augment_compose(
        source_annotator=rgba_to_rgb_annotator,
        augmentations=[rep.Augmentation.from_function(image_gaussian_noise_warp, kernel_seed=1234, sigma=25)],
    ),
)

# replace the existing IsaacConvertRGBAToRGB annotator with the new noise augmented annotator
writer = rep.writers.get(f"{rv_rgb}" + "ROS1PublishImage")
writer.annotators[0] = "rgb_gaussian_noise"
writer.initialize(topicName="rgb_augmented", frameId="sim_camera")
writer.attach([render_product_path])

simulation_app.update()
# Need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
simulation_context.play()

frame = 0

while simulation_app.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)

    # Rotate camera by 0.5 degree every frame
    xform_api.SetRotate((90, 0, frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    frame = frame + 1

simulation_context.stop()
simulation_app.close()
