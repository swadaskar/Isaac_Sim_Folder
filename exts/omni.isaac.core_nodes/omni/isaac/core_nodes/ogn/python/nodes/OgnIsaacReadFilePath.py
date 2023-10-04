# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
from os.path import exists


class OgnIsaacReadFilePath:
    """
        look for file at path given, and return its contents
    """

    @staticmethod
    def compute(db) -> bool:

        # Empty input:
        if len(db.inputs.path) == 0 or not exists(db.inputs.path):
            db.outputs.fileContents = ""

        else:
            with open(db.inputs.path, "r") as f:
                db.outputs.fileContents = f.read()
