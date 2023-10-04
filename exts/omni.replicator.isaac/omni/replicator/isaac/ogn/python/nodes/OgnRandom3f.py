# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import random


class OgnRandom3f:
    @staticmethod
    def compute(db) -> bool:
        min_range = db.inputs.minimum
        max_range = db.inputs.maximum
        db.outputs.output = (
            random.uniform(min_range[0], max_range[0]),
            random.uniform(min_range[1], max_range[1]),
            random.uniform(min_range[2], max_range[2]),
        )

        return True
