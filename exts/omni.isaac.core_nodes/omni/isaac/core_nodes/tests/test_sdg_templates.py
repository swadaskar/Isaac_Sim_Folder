# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni.kit.test
import carb

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes
from omni.isaac.core.utils.render_product import create_hydra_texture
from omni.isaac.core_nodes.scripts.utils import submit_node_template_activation, submit_writer_attach
from omni.syntheticdata import sensors
import omni.syntheticdata._syntheticdata as sd


class TestSDGTemplates(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._core_nodes = _omni_isaac_core_nodes.acquire_interface()
        # add franka robot for test
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_template_activation(self):
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        texture, render_product_path = create_hydra_texture((512, 512), "/OmniverseKit_Persp")
        await omni.kit.app.get_app().next_update_async()
        submit_node_template_activation("IsaacReadCameraInfo", 0, [render_product_path])
        submit_node_template_activation("IsaacReadSimulationTime", 0, [render_product_path])
        for rv in sensors.get_synthetic_data()._ogn_rendervars:
            if sensors.get_synthetic_data().is_node_template_registered(rv + "ExportRawArray"):
                submit_node_template_activation(rv + "IsaacSimulationGate", 0, [render_product_path])
        sensor_names = {
            "instance_segmentation": "InstanceSegmentation",
            "semantic_segmentation": "SemanticSegmentation",
            "bounding_box_2d_tight": "BoundingBox2DTight",
            "bounding_box_2d_loose": "BoundingBox2DLoose",
            "bounding_box_3d": "BoundingBox3D",
            "PostProcessDispatch": "PostProcessDispatch",
        }

        for name in sensor_names.items():
            submit_node_template_activation(name[1] + "IsaacSimulationGate", 0, [render_product_path])

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        submit_node_template_activation(rv + "IsaacConvertRGBAToRGB", 0, [render_product_path])

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        submit_node_template_activation(rv + "IsaacConvertDepthToPointCloud", 0, [render_product_path])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
