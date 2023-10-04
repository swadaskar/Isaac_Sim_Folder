# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np


class OgnIsaacGenerateRGBA:
    """
    Test Isaac Sim RGBA Node
    """

    @staticmethod
    def compute(db) -> bool:
        """Simple compute function to generate constant color buffer"""
        db.outputs.data = np.full((db.inputs.height, db.inputs.width, 4), db.inputs.color * 255, np.uint8)
        db.outputs.width = db.inputs.width
        db.outputs.height = db.inputs.height
        db.outputs.encoding = "rgba8"
        return True
