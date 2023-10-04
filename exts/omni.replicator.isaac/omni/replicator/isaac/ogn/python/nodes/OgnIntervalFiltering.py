# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.graph.core as og
import numpy as np


class OgnIntervalFiltering:
    @staticmethod
    def compute(db) -> bool:
        interval = db.inputs.interval
        frame_num = np.array(db.inputs.frameCounts)
        indices = np.array(db.inputs.indices)
        ignore_interval = db.inputs.ignoreInterval
        if (not ignore_interval and (interval is None or len(frame_num) == 0)) or (
            ignore_interval and len(indices) == 0
        ):
            db.outputs.indices = []
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False

        if ignore_interval:
            output_inds = indices
            db.outputs.on_reset = True
        else:
            output_inds = np.nonzero(np.logical_and(frame_num % interval == 0, frame_num > 0))
            db.outputs.on_reset = False

        if len(output_inds) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        else:
            db.outputs.indices = output_inds
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
