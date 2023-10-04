# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy
import os
import omni


class OgnIsaacReadEnvVar:
    """
         look for environment variable on OS, and return it.
    """

    @staticmethod
    def compute(db) -> bool:

        # Empty input case:
        if len(db.inputs.envVar) == 0:
            db.outputs.value = ""

        else:
            # Get environment variable
            envv = os.getenv(db.inputs.envVar)

            if envv is None:
                db.outputs.value = ""
            else:
                db.outputs.value = envv

        return True
