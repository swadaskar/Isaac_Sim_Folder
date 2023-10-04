# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.graph.core as og
from omni.isaac.core.utils.stage import get_stage_units


class OgnIsaacScaleToFromStageUnit:
    """
    Isaac Sim Scale To and From Stage Units
    """

    @staticmethod
    def compute(db) -> bool:
        conversion = db.inputs.conversion
        value = db.inputs.value.value

        if conversion:
            if conversion not in (db.tokens.toStage, db.tokens.toMeters):
                db.log_error("Unable to use custom conversion option")
                return False
        else:
            db.log_error("No conversion option detected")
            return False

        if conversion in db.tokens.toStage:
            db.outputs.result.value = value / get_stage_units()

        elif conversion in db.tokens.toMeters:
            db.outputs.result.value = value * get_stage_units()

        return True

    @staticmethod
    def on_connection_type_resolve(node) -> None:
        int_types = (
            og.BaseDataType.UCHAR,
            og.BaseDataType.INT,
            og.BaseDataType.UINT,
            og.BaseDataType.INT64,
            og.BaseDataType.UINT64,
        )

        valtype = node.get_attribute("inputs:value").get_resolved_type()
        resultattr = node.get_attribute("outputs:result")
        resulttype = resultattr.get_resolved_type()

        # If Value is resolved, resolve result to match Value
        if valtype.base_type != og.BaseDataType.UNKNOWN and resulttype.base_type == og.BaseDataType.UNKNOWN:
            # If Value is an integral, then set it to a double
            new_type = valtype
            if valtype.base_type in int_types:
                new_type.base_type = og.BaseDataType.DOUBLE
            resultattr.set_resolved_type(new_type)
