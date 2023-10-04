# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.franka_nut_and_bolt import FrankaNutAndBolt


class FrankaNutAndBoltExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="Manipulation",
            submenu_name="",
            name="Franka Nut and Bolt",
            title="Franka Nut and Bolt",
            doc_link="",
            overview="Franka robot arms picking and screwing nuts onto bolts",
            file_path=os.path.abspath(__file__),
            sample=FrankaNutAndBolt(),
        )
        return
