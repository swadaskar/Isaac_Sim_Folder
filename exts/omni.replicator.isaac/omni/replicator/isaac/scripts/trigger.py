# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.replicator.core.scripts.utils import ReplicatorWrapper, create_node
from .context import initialize_context


@ReplicatorWrapper
def on_rl_frame(num_envs: int):
    """ 
        Args:
            num_envs (int): The number of environments corresponding to the number of prims 
                            encapsulated in the RigidPrimViews and ArticulationViews.
    """
    node = create_node("omni.replicator.isaac.OgnOnRLFrame")
    node.get_attribute("inputs:num_envs").set(num_envs)

    initialize_context(num_envs, node)

    return node
