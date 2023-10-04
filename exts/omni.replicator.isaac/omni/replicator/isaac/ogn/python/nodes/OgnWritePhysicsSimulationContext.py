# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import torch
import numpy as np
import carb

import omni.graph.core as og

from omni.replicator.isaac import SIMULATION_CONTEXT_ATTRIBUTES
from omni.replicator.isaac import physics_view as physics


OPERATION_TYPES = ["direct", "additive", "scaling"]


def apply_randomization_operation(operation, attribute_name, samples, on_reset):
    if on_reset:
        return physics._simulation_context_reset_values[attribute_name]
    if operation == "additive":
        return physics._simulation_context_reset_values[attribute_name] + samples
    elif operation == "scaling":
        return physics._simulation_context_reset_values[attribute_name] * samples
    else:
        return samples


def modify_initial_values(operation, attribute_name, samples):
    if operation == "additive":
        physics._simulation_context_reset_values[attribute_name] = (
            physics._simulation_context_initial_values[attribute_name] + samples
        )
    elif operation == "scaling":
        physics._simulation_context_reset_values[attribute_name] = (
            physics._simulation_context_initial_values[attribute_name] * samples
        )
    else:
        physics._simulation_context_reset_values[attribute_name] = samples


class OgnWritePhysicsSimulationContext:
    @staticmethod
    def compute(db) -> bool:
        view_name = db.inputs.prims
        attribute_name = db.inputs.attribute
        operation = db.inputs.operation
        values = db.inputs.values
        if db.inputs.indices is None or len(db.inputs.indices) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False
        indices = np.array(db.inputs.indices)
        on_reset = db.inputs.on_reset

        try:
            simulation_context = physics._simulation_context
            if simulation_context is None:
                raise ValueError(f"Expected a registered simulation_context")
            if attribute_name not in SIMULATION_CONTEXT_ATTRIBUTES:
                raise ValueError(
                    f"Expected an attribute in {SIMULATION_CONTEXT_ATTRIBUTES}, but instead received {attribute_name}"
                )
            if operation not in OPERATION_TYPES:
                raise ValueError(f"Expected an operation type in {OPERATION_TYPES}, but instead received {operation}")

            samples = np.array(values).reshape(len(indices), -1)[0]
        except Exception as error:
            db.log_error(f"WritePhysics Error: {error}")
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        if on_reset:
            modify_initial_values(operation, attribute_name, samples)

        if attribute_name == "gravity":
            gravity = apply_randomization_operation(operation, attribute_name, samples, on_reset)
            simulation_context._physics_sim_view.set_gravity(carb.Float3(gravity[0], gravity[1], gravity[2]))

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
