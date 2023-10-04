# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.unitree_quadruped import QuadrupedExample


class QuadrupedExampleExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)

        overview = "This Example shows how to simulate an Unitree A1 quadruped in Isaac Sim."

        super().start_extension(
            menu_name="",
            submenu_name="",
            name="cell 2 suspension",
            title="Unitree A1 Quadruped Example",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_quadruped.html",
            overview=overview,
            file_path=os.path.abspath(__file__),
            sample=QuadrupedExample(),
        )
        return
